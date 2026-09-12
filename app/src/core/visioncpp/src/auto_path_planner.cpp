#include "visioncpp/auto_path_planner.hpp"

#include "visioncpp/calibration.hpp"
#include "visioncpp/kdtree.hpp"
#include "visioncpp/mask_set.hpp"
#include "visioncpp/normal_smoother.hpp"
#include "visioncpp/plane_slicer.hpp"
#include "visioncpp/tool_frame.hpp"

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <unordered_set>

namespace visioncpp {
namespace {

constexpr int kNone = 0, kLeft = 1, kRight = 2, kOverlap = 3;

Vec3 pcaMainAxis(const std::vector<Vec3>& vertices) {
    Vec3 mean = Vec3::Zero();
    for (const auto& v : vertices) mean += v;
    mean /= static_cast<double>(vertices.size());
    Mat3 cov = Mat3::Zero();
    for (const auto& v : vertices) {
        const Vec3 d = v - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(std::max<size_t>(1, vertices.size() - 1));
    Eigen::SelfAdjointEigenSolver<Mat3> es(cov);
    Vec3 main = es.eigenvectors().col(2);
    if (main.y() < 0) main = -main;
    if (main.norm() < 1e-9) return Vec3::UnitY();
    return main.normalized();
}

Vec3 transverseAxis(const Vec3& main) {
    Vec3 t = Vec3::UnitX().cross(main);
    if (t.norm() < 1e-5) return Vec3::UnitY();
    return t.normalized();
}

std::pair<Vec3, bool> fitOuterEdgeAxis(const std::vector<Vec3>& vertices) {
    const Vec3 rough_main = pcaMainAxis(vertices);
    const Vec3 rough_trans = transverseAxis(rough_main);
    std::vector<double> v_long, v_trans;
    v_long.reserve(vertices.size());
    v_trans.reserve(vertices.size());
    for (const auto& v : vertices) {
        v_long.push_back(v.dot(rough_main));
        v_trans.push_back(v.dot(rough_trans));
    }
    const double min_l = *std::min_element(v_long.begin(), v_long.end());
    const double max_l = *std::max_element(v_long.begin(), v_long.end());
    std::vector<Eigen::Vector2d> left_edge, right_edge;
    for (int i = 0; i < 20; ++i) {
        const double a = min_l + (max_l - min_l) * i / 20.0;
        const double b = min_l + (max_l - min_l) * (i + 1) / 20.0;
        int imin = -1, imax = -1;
        double tmin = 0, tmax = 0;
        for (size_t j = 0; j < vertices.size(); ++j) {
            if (v_long[j] < a || v_long[j] > b) continue;
            if (imin < 0 || v_trans[j] < tmin) {
                tmin = v_trans[j];
                imin = static_cast<int>(j);
            }
            if (imax < 0 || v_trans[j] > tmax) {
                tmax = v_trans[j];
                imax = static_cast<int>(j);
            }
        }
        if (imin >= 0) left_edge.emplace_back(tmin, v_long[imin]);
        if (imax >= 0) right_edge.emplace_back(tmax, v_long[imax]);
    }

    auto fit_edge = [&](const std::vector<Eigen::Vector2d>& edge) -> std::pair<Vec3, double> {
        if (edge.size() < 2) return {rough_main, 1e300};
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const auto& p : edge) mean += p;
        mean /= static_cast<double>(edge.size());
        Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
        for (const auto& p : edge) {
            const Eigen::Vector2d d = p - mean;
            A += d * d.transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(A, Eigen::ComputeFullV);
        double d_trans = svd.matrixV()(0, 0);
        double d_long = svd.matrixV()(1, 0);
        if (d_long < 0) {
            d_trans = -d_trans;
            d_long = -d_long;
        }
        const double err = svd.singularValues().size() > 1 ? svd.singularValues()(1) : 0.0;
        Vec3 axis = d_trans * rough_trans + d_long * rough_main;
        if (axis.norm() < 1e-9) return {rough_main, err};
        return {axis.normalized(), err};
    };

    const auto left = fit_edge(left_edge);
    const auto right = fit_edge(right_edge);
    if (left.second <= right.second) return {left.first, true};
    return {right.first, false};
}

std::vector<Vec3> uniqueRound4(const std::vector<Vec3>& pts) {
    struct Key {
        long long x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct Hash {
        size_t operator()(const Key& k) const {
            return std::hash<long long>{}(k.x) ^ (std::hash<long long>{}(k.y) << 1)
                   ^ (std::hash<long long>{}(k.z) << 2);
        }
    };
    std::unordered_set<Key, Hash> seen;
    std::vector<Vec3> out;
    out.reserve(pts.size());
    for (const auto& p : pts) {
        Key k{std::llround(p.x() * 10000.0), std::llround(p.y() * 10000.0), std::llround(p.z() * 10000.0)};
        if (seen.insert(k).second) {
            out.emplace_back(k.x / 10000.0, k.y / 10000.0, k.z / 10000.0);
        }
    }
    std::sort(out.begin(), out.end(), [](const Vec3& a, const Vec3& b) {
        if (a.x() != b.x()) return a.x() < b.x();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.z() < b.z();
    });
    return out;
}

std::vector<OrientedSample> zigzagSample(const Mesh& slice_mesh,
                                         const std::vector<Vec3>& pca_verts,
                                         const KdTree& full_tree,
                                         const std::vector<Vec3>& vertex_normals,
                                         const AutoPathOptions& opt) {
    const double row_s = opt.row_spacing_mm / 1000.0;
    const double pt_s = opt.point_spacing_mm / 1000.0;
    Vec3 edge_axis;
    bool is_left = true;
    if (opt.align_outer_edge) {
        auto r = fitOuterEdgeAxis(pca_verts);
        edge_axis = r.first;
        is_left = r.second;
    } else {
        edge_axis = pcaMainAxis(pca_verts);
    }

    Vec3 plane_normal = Vec3::UnitX().cross(edge_axis);
    if (plane_normal.norm() < 1e-8) plane_normal = transverseAxis(edge_axis);
    else plane_normal.normalize();
    if (plane_normal.y() < 0) plane_normal = -plane_normal;

    double min_proj = 1e300, max_proj = -1e300;
    for (const auto& v : pca_verts) {
        const double p = v.dot(plane_normal);
        min_proj = std::min(min_proj, p);
        max_proj = std::max(max_proj, p);
    }

    std::vector<double> slice_projs;
    if (is_left) {
        for (double p = min_proj + row_s / 2.0; p <= max_proj + 1e-12; p += row_s) slice_projs.push_back(p);
    } else {
        for (double p = max_proj - row_s / 2.0; p >= min_proj - 1e-12; p -= row_s) slice_projs.push_back(p);
    }

    std::vector<OrientedSample> zigzag;
    bool direction_forward = true;
    for (double proj : slice_projs) {
        const auto segs = PlaneSlicer::meshPlane(slice_mesh, plane_normal, plane_normal * proj);
        if (segs.empty()) continue;
        std::vector<Vec3> raw;
        raw.reserve(segs.size() * 2);
        for (const auto& s : segs) {
            raw.push_back(s[0]);
            raw.push_back(s[1]);
        }
        auto pts = uniqueRound4(raw);
        if (pts.size() < 2) continue;
        std::sort(pts.begin(), pts.end(), [&](const Vec3& a, const Vec3& b) {
            return a.dot(edge_axis) < b.dot(edge_axis);
        });
        std::vector<double> cum(pts.size(), 0);
        for (size_t i = 1; i < pts.size(); ++i) cum[i] = cum[i - 1] + (pts[i] - pts[i - 1]).norm();
        const double total = cum.back();
        if (total < pt_s * 0.25) continue;
        std::vector<Vec3> sampled;
        for (double d = 0; d < total; d += pt_s) {
            size_t i = 1;
            while (i < cum.size() && cum[i] < d) ++i;
            if (i >= cum.size()) i = cum.size() - 1;
            const double c0 = cum[i - 1], c1 = cum[i];
            const double t = (c1 > c0) ? (d - c0) / (c1 - c0) : 0;
            sampled.push_back(pts[i - 1] + t * (pts[i] - pts[i - 1]));
        }
        // 补行末端点: 上面 d<total 的严格循环会丢掉每行沿主轴 (edge_axis) 的极值端点,
        // 即 PCA 上下两个顶点所在行够不到真正的边界。显式并入终点 pts.back();
        // 与已采样末点几乎重合 (< 1/4 点距) 时跳过, 避免生成重复航点。
        if (!sampled.empty() && (pts.back() - sampled.back()).norm() > pt_s * 0.25) {
            sampled.push_back(pts.back());
        }
        if (!direction_forward) std::reverse(sampled.begin(), sampled.end());
        for (size_t i_pt = 0; i_pt < sampled.size(); ++i_pt) {
            const int idx = full_tree.nearest(sampled[i_pt]);
            Vec3 nrm = (idx >= 0 && idx < static_cast<int>(vertex_normals.size())) ? vertex_normals[idx]
                                                                                  : Vec3::UnitX();
            const double nlen = nrm.norm();
            if (nlen < 1e-9) nrm = Vec3::UnitX();
            else nrm /= nlen;
            OrientedSample s;
            s.point = sampled[i_pt];
            s.normal = nrm;
            s.is_jump = !zigzag.empty() && i_pt == 0;
            zigzag.push_back(s);
        }
        direction_forward = !direction_forward;
    }
    return zigzag;
}

std::vector<int> facesForLabels(const Mesh& mesh, const std::vector<int>& labels, const std::set<int>& accept) {
    std::vector<char> ok(labels.size(), 0);
    for (int lab : accept) {
        for (size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] == lab) ok[i] = 1;
        }
    }
    std::vector<int> keep;
    for (int i = 0; i < mesh.faceCount(); ++i) {
        const auto& f = mesh.faces[i];
        if (ok[f[0]] || ok[f[1]] || ok[f[2]]) keep.push_back(i);
    }
    return keep;
}

Mesh submesh(const Mesh& mesh, const std::vector<int>& face_ids) {
    Mesh out;
    out.vertices = mesh.vertices;
    out.faces.reserve(face_ids.size());
    for (int i : face_ids) out.faces.push_back(mesh.faces[i]);
    return out;
}

std::vector<OrientedSample> sampleLeg(const Mesh& mesh,
                                      const std::vector<int>& face_ids,
                                      const KdTree& tree,
                                      const std::vector<Vec3>& vnorm,
                                      int leg_id,
                                      const AutoPathOptions& opt) {
    if (face_ids.empty()) return {};
    Mesh slice = submesh(mesh, face_ids);
    std::vector<char> used(mesh.vertices.size(), 0);
    for (int fi : face_ids) {
        used[mesh.faces[fi][0]] = used[mesh.faces[fi][1]] = used[mesh.faces[fi][2]] = 1;
    }
    std::vector<Vec3> pca;
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        if (used[i]) pca.push_back(mesh.vertices[i]);
    }
    if (pca.size() < 10) return {};
    auto samples = zigzagSample(slice, pca, tree, vnorm, opt);
    for (auto& s : samples) s.leg_id = leg_id;
    return samples;
}

std::vector<int> projectLabels(const std::vector<Vec3>& verts,
                               const CameraIntrinsics& k,
                               const Mat4& T_cb,
                               const std::vector<cv::Mat>& legs) {
    const Mat4 T_bc = T_cb.inverse();
    const Mat3 R = T_bc.block<3, 3>(0, 0);
    const Vec3 t = T_bc.block<3, 1>(0, 3);
    std::vector<int> labels(verts.size(), kNone);
    bool any = false;
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec3 pcam = R * verts[i] + t;
        if (pcam.z() <= 1e-6) continue;
        const int u = static_cast<int>(std::lround(k.fx * pcam.x() / pcam.z() + k.cx));
        const int v = static_cast<int>(std::lround(k.fy * pcam.y() / pcam.z() + k.cy));
        if (u < 0 || v < 0 || u >= k.width || v >= k.height) continue;
        if (legs.size() == 1) {
            if (legs[0].at<uint8_t>(v, u)) {
                labels[i] = kLeft;
                any = true;
            }
        } else {
            const bool inl = legs[0].at<uint8_t>(v, u) != 0;
            const bool inr = legs[1].at<uint8_t>(v, u) != 0;
            if (inl && !inr) labels[i] = kLeft;
            else if (inr && !inl) labels[i] = kRight;
            else if (inl && inr) labels[i] = kOverlap;
            if (inl || inr) any = true;
        }
    }
    if (!any) throw VisionError("no mesh vertices project into the image; check K/T and mesh frame");
    return labels;
}

std::vector<OrientedSample> concatDedup(std::vector<OrientedSample> left,
                                        std::vector<OrientedSample> right,
                                        double radius_mm) {
    if (right.empty()) return left;
    if (left.empty()) return right;
    const double r = std::max(radius_mm, 0.0) / 1000.0;
    if (r > 0) {
        std::vector<Vec3> lxyz;
        lxyz.reserve(left.size());
        for (const auto& p : left) lxyz.push_back(p.point);
        KdTree lt;
        lt.build(lxyz);
        std::vector<OrientedSample> kept;
        for (const auto& p : right) {
            const int j = lt.nearest(p.point);
            if (j >= 0 && (lxyz[j] - p.point).norm() <= r) continue;
            kept.push_back(p);
        }
        right.swap(kept);
    }
    if (!right.empty()) right.front().is_jump = true;
    left.insert(left.end(), right.begin(), right.end());
    return left;
}

std::vector<Waypoint> samplesToWaypoints(const std::vector<OrientedSample>& samples,
                                         const CameraIntrinsics& k,
                                         const Mat4& T_cb,
                                         double spray_mm) {
    const Mat4 T_bc = T_cb.inverse();
    const Mat3 Rbc = T_bc.block<3, 3>(0, 0);
    const Vec3 tbc = T_bc.block<3, 1>(0, 3);
    ToolFrame tf;
    std::vector<Waypoint> points;
    points.reserve(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        Vec3 n = samples[i].normal;
        if (n.norm() < 1e-9) n = Vec3::UnitX();
        else n.normalize();
        const Vec3 p_m = samples[i].point;
        const Vec3 p_surf_mm = p_m * 1000.0;
        const Vec3 p_tcp_mm = p_surf_mm + spray_mm * n;
        const Mat3 R = tf.compute(n);
        const Vec3 euler = ToolFrame::rpyXyzDeg(R);

        const Vec3 p_cam = Rbc * p_m + tbc;
        Vec3 n_cam = Rbc * n;
        if (n_cam.norm() > 1e-9) n_cam.normalize();
        double u = 0, v = 0, du = 0, dv = 0;
        if (p_cam.z() > 1e-6) {
            u = k.fx * p_cam.x() / p_cam.z() + k.cx;
            v = k.fy * p_cam.y() / p_cam.z() + k.cy;
            const Vec3 p_tcp_cam = p_cam + (spray_mm / 1000.0) * n_cam;
            if (p_tcp_cam.z() > 1e-6) {
                du = k.fx * p_tcp_cam.x() / p_tcp_cam.z() + k.cx - u;
                dv = k.fy * p_tcp_cam.y() / p_tcp_cam.z() + k.cy - v;
            }
        }
        Waypoint w;
        w.index = static_cast<int>(i + 1);
        w.pixel_u = static_cast<int>(std::lround(u));
        w.pixel_v = static_cast<int>(std::lround(v));
        w.surface_point_cam_mm = p_cam * 1000.0;
        w.surface_point_base_mm = p_surf_mm;
        w.surface_normal_base = n;
        w.surface_normal_cam = n_cam;
        w.standoff_distance_mm = spray_mm;
        w.tcp_xyz_mm = p_tcp_mm;
        w.tcp_rpy_deg = euler;
        w.n2d_u = std::round(du * 10.0) / 10.0;
        w.n2d_v = std::round(dv * 10.0) / 10.0;
        w.is_jump = samples[i].is_jump;
        w.leg_id = samples[i].leg_id;
        points.push_back(w);
    }
    return points;
}

}  // namespace

PathDoc AutoPathPlanner::plan(const Mesh& mesh,
                              const MaskSet& masks,
                              const CameraIntrinsics& k,
                              const Mat4& T_camera_to_base,
                              const AutoPathOptions& opt) {
    if (!k.valid()) throw VisionError("camera_intrinsics required");
    if (Calibration::isIdentity(T_camera_to_base)) {
        throw VisionError("T_camera_to_base is Identity; refuse to plan");
    }
    if (mesh.vertexCount() < 10 || mesh.faceCount() < 1) {
        throw VisionError("mesh is empty or too small");
    }
    const int n_faces0 = mesh.faceCount();

    Mesh work = mesh;
    work.computeVertexNormals();
    KdTree tree;
    tree.build(work.vertices);

    auto legs = masks.splitLegs(opt.depth_threshold_ratio, 0.0);
    auto labels = projectLabels(work.vertices, k, T_camera_to_base, legs);

    std::vector<OrientedSample> samples;
    if (legs.size() == 1) {
        samples = sampleLeg(work, facesForLabels(work, labels, {kLeft, kRight, kOverlap}), tree,
                            work.vertex_normals, 0, opt);
        if (samples.empty()) throw VisionError("no waypoints sampled on the single-leg region");
    } else {
        auto left = sampleLeg(work, facesForLabels(work, labels, {kLeft, kOverlap}), tree,
                              work.vertex_normals, 0, opt);
        auto right = sampleLeg(work, facesForLabels(work, labels, {kRight, kOverlap}), tree,
                               work.vertex_normals, 1, opt);
        if (left.empty() && right.empty()) throw VisionError("no waypoints sampled on either leg");
        samples = concatDedup(std::move(left), std::move(right), opt.dedup_radius_mm);
    }
    if (work.faceCount() != n_faces0) throw VisionError("mesh faces were mutated");

    NormalSmoother(opt.normal_smooth_window).smooth(samples);
    PathDoc doc;
    doc.standoff_distance_mm = opt.spray_dist_mm;
    doc.points = samplesToWaypoints(samples, k, T_camera_to_base, opt.spray_dist_mm);
    if (doc.points.empty()) throw VisionError("waypoint conversion produced an empty path");
    return doc;
}

void AutoPathPlanner::writeYaml(const std::string& path, const PathDoc& doc) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "standoff_distance_mm" << YAML::Value << doc.standoff_distance_mm;
    out << YAML::Key << "type" << YAML::Value << "auto";
    out << YAML::Key << "coordinate_frame" << YAML::Value << "base_link";
    out << YAML::Key << "paths" << YAML::Value << YAML::BeginSeq;
    out << YAML::BeginMap;
    out << YAML::Key << "path_id" << YAML::Value << 1;
    out << YAML::Key << "name" << YAML::Value << "Auto Path";
    out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
    for (const auto& p : doc.points) {
        out << YAML::BeginMap;
        out << YAML::Key << "index" << YAML::Value << p.index;
        out << YAML::Key << "pixel" << YAML::Value << YAML::Flow << YAML::BeginSeq << p.pixel_u << p.pixel_v
            << YAML::EndSeq;
        out << YAML::Key << "surface_point_base_mm" << YAML::Value << YAML::Flow << YAML::BeginSeq
            << std::round(p.surface_point_base_mm.x() * 100.0) / 100.0
            << std::round(p.surface_point_base_mm.y() * 100.0) / 100.0
            << std::round(p.surface_point_base_mm.z() * 100.0) / 100.0 << YAML::EndSeq;
        out << YAML::Key << "surface_point_cam_mm" << YAML::Value << YAML::Flow << YAML::BeginSeq
            << std::round(p.surface_point_cam_mm.x() * 100.0) / 100.0
            << std::round(p.surface_point_cam_mm.y() * 100.0) / 100.0
            << std::round(p.surface_point_cam_mm.z() * 100.0) / 100.0 << YAML::EndSeq;
        out << YAML::Key << "surface_normal_base" << YAML::Value << YAML::Flow << YAML::BeginSeq
            << std::round(p.surface_normal_base.x() * 10000.0) / 10000.0
            << std::round(p.surface_normal_base.y() * 10000.0) / 10000.0
            << std::round(p.surface_normal_base.z() * 10000.0) / 10000.0 << YAML::EndSeq;
        out << YAML::Key << "surface_normal_cam" << YAML::Value << YAML::Flow << YAML::BeginSeq
            << std::round(p.surface_normal_cam.x() * 10000.0) / 10000.0
            << std::round(p.surface_normal_cam.y() * 10000.0) / 10000.0
            << std::round(p.surface_normal_cam.z() * 10000.0) / 10000.0 << YAML::EndSeq;
        out << YAML::Key << "standoff_distance_mm" << YAML::Value
            << std::round(p.standoff_distance_mm * 10.0) / 10.0;
        out << YAML::Key << "tcp_pose_base" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << std::round(p.tcp_xyz_mm.x() * 100.0) / 100.0;
        out << YAML::Key << "y" << YAML::Value << std::round(p.tcp_xyz_mm.y() * 100.0) / 100.0;
        out << YAML::Key << "z" << YAML::Value << std::round(p.tcp_xyz_mm.z() * 100.0) / 100.0;
        out << YAML::Key << "rx" << YAML::Value << std::round(p.tcp_rpy_deg.x() * 100.0) / 100.0;
        out << YAML::Key << "ry" << YAML::Value << std::round(p.tcp_rpy_deg.y() * 100.0) / 100.0;
        out << YAML::Key << "rz" << YAML::Value << std::round(p.tcp_rpy_deg.z() * 100.0) / 100.0;
        out << YAML::EndMap;
        out << YAML::Key << "normal_2d_proj" << YAML::Value << YAML::Flow << YAML::BeginSeq << p.n2d_u << p.n2d_v
            << YAML::EndSeq;
        out << YAML::Key << "spraying" << YAML::Value << (p.is_jump ? "off" : "on");
        out << YAML::Key << "is_jump" << YAML::Value << p.is_jump;
        out << YAML::Key << "leg_id" << YAML::Value << p.leg_id;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    out << YAML::EndSeq;
    out << YAML::EndMap;
    std::ofstream f(path);
    if (!f) throw VisionError("cannot write " + path);
    f << out.c_str();
}

}  // namespace visioncpp
