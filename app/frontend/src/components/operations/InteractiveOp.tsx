import React, { useState, useEffect, useRef, type MouseEvent } from 'react';
import { API_BASE, WS_BASE } from '../../config';
import {
  Play,
  Pause,
  X,
} from 'lucide-react';

import type {
  FileItem,
  Point,
  MaskData,
  WaypointItem,
  ManualPathItem,
  VerificationReport,
  SessionData,
  LiveNormalInfo,
  PathStateType,
  SimulationState,
  CanvasNotice,
  ExecutingAction,
} from './interactive/types';
import { STATE_THEMES, pathSourceOf } from './interactive/types';
import { computeNormalClientSide } from './interactive/normalComputation';
import { InteractiveCanvas } from './interactive/InteractiveCanvas';
import { TemplateTopBar } from './interactive/TemplateFileManager';
import { TemplateFileList } from './interactive/TemplateFileList';
import { FollowPanel } from './interactive/FollowPanel';
import { InteractiveActionColumn } from './interactive/InteractiveActionColumn';

interface InteractiveOpProps {
  externalActiveTemplate?: string | null;
  onTemplateChange?: (templateName: string | null) => void;
  onMeshUpdated?: () => void;
  onPathsUpdated?: () => void;
  onPathStateChange?: (state: PathStateType) => void;
  onSimulationJointsChange?: (joints: number[] | null) => void;
}

function projectBasePointToPixel(
  posBaseMm: [number, number, number],
  T_base_cam: number[],
  intrinsics: { fx: number; fy: number; cx: number; cy: number }
): [number, number] | null {
  if (!T_base_cam || T_base_cam.length < 16) return null;
  const R00 = T_base_cam[0], R01 = T_base_cam[1], R02 = T_base_cam[2], tx = T_base_cam[3];
  const R10 = T_base_cam[4], R11 = T_base_cam[5], R12 = T_base_cam[6], ty = T_base_cam[7];
  const R20 = T_base_cam[8], R21 = T_base_cam[9], R22 = T_base_cam[10], tz = T_base_cam[11];

  const dx = posBaseMm[0] - tx;
  const dy = posBaseMm[1] - ty;
  const dz = posBaseMm[2] - tz;

  const Xc = R00 * dx + R10 * dy + R20 * dz;
  const Yc = R01 * dx + R11 * dy + R21 * dz;
  const Zc = R02 * dx + R12 * dy + R22 * dz;

  if (Zc <= 10.0) return null;

  const u = (intrinsics.fx * Xc) / Zc + intrinsics.cx;
  const v = (intrinsics.fy * Yc) / Zc + intrinsics.cy;
  return [Math.round(u * 10) / 10, Math.round(v * 10) / 10];
}

const InteractiveOp: React.FC<InteractiveOpProps> = ({
  externalActiveTemplate,
  onTemplateChange,
  onMeshUpdated,
  onPathsUpdated,
  onPathStateChange,
  onSimulationJointsChange,
}) => {
  // ─── 1. Template & File State ──────────────────────────────────────────
  const [templates, setTemplates] = useState<string[]>([]);
  const [activeTemplate, setActiveTemplate] = useState<string | null>(externalActiveTemplate || null);
  const [files, setFiles] = useState<FileItem[]>([]);
  const [isLoadingTemplate, setIsLoadingTemplate] = useState<boolean>(false);

  // ─── 2. Image & Viewport State ──────────────────────────────────────────
  const [imageUrl, setImageUrl] = useState<string | null>(null);
  const [hasImage, setHasImage] = useState<boolean>(false);
  const [natSize, setNatSize] = useState<{ w: number; h: number } | null>(null);
  const [zoom, setZoom] = useState<number>(1);
  const [pan, setPan] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [isPanning, setIsPanning] = useState<boolean>(false);
  const [isSpacePressed, setIsSpacePressed] = useState<boolean>(false);

  // ─── 3. SAM Segmentation State ──────────────────────────────────────────
  const [segMode, setSegMode] = useState<boolean>(false);
  const [currentPoints, setCurrentPoints] = useState<Point[]>([]);
  const [currentPolygons, setCurrentPolygons] = useState<number[][][]>([]);
  const [committedMasks, setCommittedMasks] = useState<MaskData[]>([]);
  const [savedMasks, setSavedMasks] = useState<MaskData[]>([]);
  const [showMasksOverlay, setShowMasksOverlay] = useState<boolean>(true);
  const [isSamInitializing, setIsSamInitializing] = useState<boolean>(false);
  // wissight 检出的 box prompt：作为当前初稿的提示之一参与预测，可手动去掉
  const [currentBox, setCurrentBox] = useState<number[] | null>(null);
  const [isDetecting, setIsDetecting] = useState<boolean>(false);

  // ─── 4. Manual / Auto × Orig / POI path state ────────────────────────────
  const [activeState, setActiveState] = useState<PathStateType>('raw');
  const [manualPathMode, setManualPathMode] = useState<boolean>(false);
  const [manualPaths, setManualPaths] = useState<ManualPathItem[]>([]);
  const [rawPaths, setRawPaths] = useState<ManualPathItem[]>([]);
  const [autoPaths, setAutoPaths] = useState<ManualPathItem[]>([]);
  const [poiPaths, setPoiPaths] = useState<ManualPathItem[]>([]);
  const [autoPoiPaths, setAutoPoiPaths] = useState<ManualPathItem[]>([]);
  const [currentManualPoints, setCurrentManualPoints] = useState<WaypointItem[]>([]);
  const [selectedPathIdForEdit, setSelectedPathIdForEdit] = useState<number | null>(null);
  const [standoffDistMm, setStandoffDistMm] = useState<number>(150.0);
  const [showManualPathsOverlay, setShowManualPathsOverlay] = useState<boolean>(true);
  const [hoveredWaypoint, setHoveredWaypoint] = useState<WaypointItem | null>(null);
  const [highlightedPathId, setHighlightedPathId] = useState<number | null>(null);
  const [sessionData, setSessionData] = useState<SessionData | null>(null);

  // Workflow dependencies
  const hasMask = savedMasks.length > 0 || committedMasks.length > 0 || files.some((f) => f.name === 'scan.masks.yaml');
  const hasMesh = files.some((f) => f.name === 'scan.mesh.ply' || f.name === 'scan.mesh.stl' || f.name.includes('mesh'));
  const [mousePixel, setMousePixel] = useState<{ u: number; v: number } | null>(null);
  const [liveNormal, setLiveNormal] = useState<LiveNormalInfo | null>(null);

  // ─── 5. Verification & Report State ─────────────────────────────────────
  const [rawReport, setRawReport] = useState<VerificationReport | null>(null);
  const [autoReport, setAutoReport] = useState<VerificationReport | null>(null);
  const [poiReport, setPoiReport] = useState<VerificationReport | null>(null);
  const [autoPoiReport, setAutoPoiReport] = useState<VerificationReport | null>(null);
  const [isVerifying, setIsVerifying] = useState<boolean>(false);
  const [isOptimizing, setIsOptimizing] = useState<boolean>(false);
  const [bypassVerification, setBypassVerification] = useState<boolean>(false);

  // ─── 6. Action Execution State ──────────────────────────────────────────
  const [isCapturing, setIsCapturing] = useState<boolean>(false);
  const [isReconstructing, setIsReconstructing] = useState<boolean>(false);
  const [isAutoGenerating, setIsAutoGenerating] = useState<boolean>(false);
  const [isCleaningFiles, setIsCleaningFiles] = useState<boolean>(false);

  // ─── 7. 3D/2D Synchronized Simulation Engine State (Feature 7) ───────────
  const [simulationState, setSimulationState] = useState<SimulationState | null>(null);
  const simAnimFrameRef = useRef<number | null>(null);
  const simDataRef = useRef<{
    steps: Array<{
      q_deg: number[];
      tcp: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
      pixel: [number, number] | null;
      pathIdx: number;
    }>;
    stepIndex: number;
    speedMultiplier: number;
    isPlaying: boolean;
    stateType: PathStateType;
  }>({
    steps: [],
    stepIndex: 0,
    speedMultiplier: 1.0,
    isPlaying: false,
    stateType: 'raw',
  });

  // ─── 8. In-Canvas Animated HUD Notification State ───────────────────────
  const [canvasNotice, setCanvasNotice] = useState<CanvasNotice | null>(null);
  const noticeTimeoutRef = useRef<any>(null);

  const showNotice = (
    type: 'success' | 'warning' | 'error' | 'info',
    message: string,
    title?: string,
    duration = 3500
  ) => {
    if (noticeTimeoutRef.current) {
      clearTimeout(noticeTimeoutRef.current);
    }
    const newId = Date.now();
    setCanvasNotice({
      id: newId,
      type,
      title,
      message,
      duration,
    });
    if (duration > 0) {
      noticeTimeoutRef.current = setTimeout(() => {
        setCanvasNotice((curr: CanvasNotice | null) => (curr?.id === newId ? null : curr));
      }, duration);
    }
  };

  // ─── 9. Robot Real-Time State (via WebSocket) ──────────────────────────
  const [robotConnected, setRobotConnected] = useState<boolean>(false);
  const [robotStatus, setRobotStatus] = useState<number | null>(null);
  const [isExecutingPath, setIsExecutingPath] = useState<boolean>(false);
  const [executingAction, setExecutingAction] = useState<ExecutingAction | null>(null);
  const execActionTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const robotWsRef = useRef<WebSocket | null>(null);
  // Ref for per-WS-message projection of real TCP → pixel (avoids stale closure)
  const sessionDataRef = useRef<typeof sessionData>(null);
  useEffect(() => { sessionDataRef.current = sessionData; }, [sessionData]);

  // Sync external active template
  useEffect(() => {
    if (externalActiveTemplate && externalActiveTemplate !== activeTemplate) {
      handleSelectTemplate(externalActiveTemplate);
    }
  }, [externalActiveTemplate]);

  // WebSocket: monitor robot connection status + real execution progress
  useEffect(() => {
    let ws: WebSocket | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;

    const connect = () => {
      ws = new WebSocket(`${WS_BASE}/api/robot/ws`);
      robotWsRef.current = ws;

      ws.onopen = () => {};
      ws.onmessage = (e) => {
        try {
          const msg = JSON.parse(e.data) as { type: string; data: any };

          if (msg.type === 'robot_state') {
            const isConn = msg.data?.connected !== undefined ? Boolean(msg.data.connected) : true;
            setRobotConnected(isConn);
            if (typeof msg.data?.status === 'number') {
              setRobotStatus(msg.data.status);
            }

            // In real-exec mode: drive currentPixel + currentJoints from real TCP
            setSimulationState((prev) => {
              if (!prev?.isRealExec) return prev;
              const pose: number[] = msg.data.pose ?? [];
              if (pose.length < 3) return prev;

              // Project TCP XYZ to 2D pixel using calibration
              let pixel: [number, number] | null = null;
              const sd = sessionDataRef.current;
              if (sd?.T && sd.T.length >= 16) {
                pixel = projectBasePointToPixel(
                  [pose[0], pose[1], pose[2]],
                  sd.T,
                  { fx: sd.fx, fy: sd.fy, cx: sd.cx, cy: sd.cy }
                );
              }

              const joints: number[] = (msg.data.joint ?? []).map((v: number) =>
                typeof v === 'number' ? v : 0
              );
              if (onSimulationJointsChange) onSimulationJointsChange(joints);

              return {
                ...prev,
                currentPixel: pixel ?? prev.currentPixel,
                currentJoints: joints.length > 0 ? joints : prev.currentJoints,
                currentTcpPose: {
                  x: pose[0] ?? prev.currentTcpPose.x,
                  y: pose[1] ?? prev.currentTcpPose.y,
                  z: pose[2] ?? prev.currentTcpPose.z,
                  rx: pose[3] ?? prev.currentTcpPose.rx,
                  ry: pose[4] ?? prev.currentTcpPose.ry,
                  rz: pose[5] ?? prev.currentTcpPose.rz,
                },
              };
            });
          } else if (msg.type === 'exec_progress') {
            // Update progress bar and path index from real waypoint events
            const d = msg.data as {
              current_waypoint: number;
              total_waypoints: number;
              path_idx: number;
              total_paths: number;
              progress: number;
            };
            setSimulationState((prev) => {
              if (!prev?.isRealExec) return prev;
              const isDone = d.current_waypoint >= d.total_waypoints;
              return {
                ...prev,
                progress: d.progress,
                currentPathIndex: d.path_idx,
                currentStep: d.current_waypoint,
                totalSteps: d.total_waypoints,
                isPlaying: !isDone,
              };
            });
          } else if (msg.type === 'exec_status') {
            // Real-time execution action status broadcast from backend
            const d = msg.data as {
              action: string;
              stage: string;
              detail?: string;
              progress?: number;
            };
            if (execActionTimerRef.current) {
              clearTimeout(execActionTimerRef.current);
              execActionTimerRef.current = null;
            }
            const isDone = d.stage === 'done';
            const isError = d.stage === 'error';
            setExecutingAction({
              text: d.action,
              stage: d.stage,
              isDone,
              isError,
              progress: d.progress,
            });
            if (isDone || isError) {
              setIsExecutingPath(false);
              execActionTimerRef.current = setTimeout(() => {
                setExecutingAction(null);
              }, isDone ? 3000 : 5000);
            } else {
              setIsExecutingPath(true);
            }
          }
        } catch {}
      };
      ws.onclose = () => {
        // When WS closes, robot might be disconnected
        setRobotConnected(false);
        setRobotStatus(null);
        reconnectTimer = setTimeout(connect, 3000);
      };
    };

    connect();
    return () => {
      if (reconnectTimer) clearTimeout(reconnectTimer);
      if (execActionTimerRef.current) clearTimeout(execActionTimerRef.current);
      ws?.close();
    };
  }, []);


  // ─── 10. Follow（相机跟随驱动仿真臂）────────────────────────────────────
  // 只记一件事："跟随此刻有没有在驱动臂"。仿真臂只有一个写入者 —— 轨迹回放和跟随都往
  // onSimulationJointsChange 上推关节角，两边同时写等于每帧换一次目标，臂会抖成抽奖。
  // 所以互斥放在**入口**（谁在跑就挡住另一个），不在下游做优先级仲裁。
  const [followJoints, setFollowJoints] = useState<number[] | null>(null);

  const handleFollowJoints = (joints: number[] | null) => {
    setFollowJoints(joints);
    if (onSimulationJointsChange) onSimulationJointsChange(joints);
  };

  // Load template list on mount
  useEffect(() => {
    fetchTemplates();
  }, []);

  // Keyboard space key listener for CAD/Figma canvas panning
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.code === 'Space' && !isSpacePressed && (e.target as HTMLElement).tagName !== 'INPUT') {
        setIsSpacePressed(true);
      }
    };
    const handleKeyUp = (e: KeyboardEvent) => {
      if (e.code === 'Space') {
        setIsSpacePressed(false);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [isSpacePressed]);

  // ─── Templates API & Atomic Synchronization ─────────────────────────────
  const fetchTemplates = async (preferredTemplate?: string) => {
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates`);
      if (!res.ok) return;
      const data = await res.json();
      const list: string[] = data.templates || [];
      setTemplates(list);
      if (list.length > 0) {
        // Automatically select preferred template if valid, otherwise select the latest template (list[0])
        const target = (preferredTemplate && list.includes(preferredTemplate))
          ? preferredTemplate
          : list[0];
        handleSelectTemplate(target);
      } else {
        setActiveTemplate(null);
        if (onTemplateChange) onTemplateChange(null);
      }
    } catch (err) {
      console.error('Failed to fetch templates:', err);
    }
  };

  const handleSelectTemplate = async (templateName: string) => {
    stopSimulation();
    setActiveTemplate(templateName);
    if (onTemplateChange) onTemplateChange(templateName);

    // Reset local edit states
    setSegMode(false);
    setManualPathMode(false);
    setCurrentPoints([]);
    setCurrentPolygons([]);
    setCurrentBox(null);
    setCommittedMasks([]);
    setCurrentManualPoints([]);
    setSelectedPathIdForEdit(null);
    setHighlightedPathId(null);
    setHoveredWaypoint(null);

    // Fetch batch summary atomically
    setIsLoadingTemplate(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${templateName}/summary`);
      if (!res.ok) throw new Error(`Summary status ${res.status}`);
      const summary = await res.json();

      setFiles(summary.files || []);
      setHasImage(summary.has_image || false);
      setSavedMasks(summary.masks || []);

      const raw = summary.raw_paths || [];
      const auto = summary.auto_paths || [];
      const poi = summary.poi_paths || [];
      const autoPoi = summary.auto_poi_paths || [];
      setRawPaths(raw);
      setAutoPaths(auto);
      setPoiPaths(poi);
      setAutoPoiPaths(autoPoi);

      let initialSt: PathStateType = 'raw';
      if (poi.length > 0) initialSt = 'poi';
      else if (autoPoi.length > 0) initialSt = 'auto_poi';
      else if (auto.length > 0) initialSt = 'auto';

      setActiveState(initialSt);
      const chosenPaths = initialSt === 'poi' ? poi : (initialSt === 'auto_poi' ? autoPoi : (initialSt === 'auto' ? auto : raw));
      setManualPaths(chosenPaths);
      if (onPathStateChange) onPathStateChange(initialSt);

      if (summary.standoff_distance_mm) {
        setStandoffDistMm(summary.standoff_distance_mm);
      }

      setRawReport(summary.raw_report || null);
      setAutoReport(summary.auto_report || null);
      setPoiReport(summary.poi_report || null);
      setAutoPoiReport(summary.auto_poi_report || null);

      if (summary.has_image) {
        const imgName = summary.image_filename || 'scan.color.jpg';
        const newImgUrl = `${API_BASE}/templates/${templateName}/${imgName}?v=${Date.now()}`;
        // Image-First barrier: Preload image before dropping backdrop
        await new Promise<void>((resolve) => {
          const img = new Image();
          img.onload = () => {
            setImageUrl(newImgUrl);
            setNatSize({ w: img.naturalWidth, h: img.naturalHeight });
            resolve();
          };
          img.onerror = () => {
            setImageUrl(newImgUrl);
            resolve();
          };
          img.src = newImgUrl;
        });
      } else {
        setImageUrl(null);
        setNatSize(null);
      }

      // Load session depth data in background without blocking initial template view
      loadSessionData(templateName);
    } catch (err) {
      console.error('Failed to load template atomic summary:', err);
    } finally {
      setIsLoadingTemplate(false);
    }
  };

  const loadSessionData = async (templateName: string) => {
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${templateName}/session_data`);
      if (!res.ok) throw new Error('Session data fetch failed');
      const data = await res.json();
      const b64 = data.depth_flat_b64 as string;
      // Native fast asynchronous Base64 decoding (prevents UI freeze on 5MB payload)
      const fetchRes = await fetch(`data:application/octet-stream;base64,${b64}`);
      const arrayBuffer = await fetchRes.arrayBuffer();
      const depthF32 = new Float32Array(arrayBuffer);
      setSessionData({
        width: data.width,
        height: data.height,
        depth: depthF32,
        fx: data.intrinsics.fx,
        fy: data.intrinsics.fy,
        cx: data.intrinsics.cx,
        cy: data.intrinsics.cy,
        T: data.T_base_camera,
        calib_source: data.calib_source,
      });
    } catch (err) {
      console.warn('Failed to load session depth data:', err);
    }
  };

  const pathsForState = (st: PathStateType): ManualPathItem[] => {
    if (st === 'auto') return autoPaths;
    if (st === 'auto_poi') return autoPoiPaths;
    if (st === 'poi') return poiPaths;
    return rawPaths;
  };

  const activatePathStateSnapshot = (newState: PathStateType, explicitPaths?: ManualPathItem[]) => {
    const targetPaths = explicitPaths ?? pathsForState(newState);
    stopSimulation();
    setActiveState(newState);
    setManualPaths(targetPaths.map((path) => ({ ...path, points: [...(path.points || [])] })));
    setHoveredWaypoint(null);
    setHighlightedPathId(null);
    setSelectedPathIdForEdit(null);
    setCurrentManualPoints([]);
    if (onPathStateChange) onPathStateChange(newState);
    if (onPathsUpdated) onPathsUpdated();
  };

  // State Switcher (Manual/Auto × Orig/POI)
  const handleSelectActiveState = (newState: PathStateType) => {
    activatePathStateSnapshot(newState);
  };

  // ─── SAM Segmentation Handlers ──────────────────────────────────────────
  const handleToggleSegMode = async () => {
    if (segMode) {
      setSegMode(false);
      setCurrentPoints([]);
      setCurrentPolygons([]);
      setCurrentBox(null);
      return;
    }
    if (!activeTemplate || !hasImage) return;
    setSegMode(true);
    setCurrentPoints([]);
    setCurrentPolygons([]);
    setCurrentBox(null);
    setCommittedMasks(savedMasks.length > 0 ? [...savedMasks] : []);

    setIsSamInitializing(true);

    try {
      await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/sam/init`, {
        method: 'POST',
      });
    } catch (err) {
      console.warn('Failed to init SAM session:', err);
    } finally {
      setIsSamInitializing(false);
    }

    // 图像编码完成后先检一次；检不到就维持原来的纯手动点选，不阻断流程
    detectBox(false);
  };

  /**
   * 跑一次 wissight 检测，命中则拿面积最大的框：开精修时把它当 box prompt 去预测，
   * 关精修时直接用后端解出的实例 mask 当可编辑初稿。
   * silent=false 时只在“什么都没检到”弹提示（检到时图上的虚线框本身就是反馈）。
   */
  const detectBox = async (silent = true) => {
    if (!activeTemplate) return;
    setIsDetecting(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/detect`, {
        method: 'POST',
      });
      if (!res.ok) throw new Error(`Detect failed (${res.status})`);
      const data = await res.json();
      // 后端已按面积降序，[0] 即画面主体
      const box: number[] | undefined = data.detections?.[0]?.box;
      if (!box) {
        if (!silent) showNotice('info', 'No target detected. Please click to segment manually');
        return;
      }
      setCurrentBox(box);
      if (data.sam_refine === false && data.polygons?.length) {
        setCurrentPolygons(data.polygons);
      } else {
        predictMask(currentPoints, box);
      }
    } catch (err: any) {
      console.warn('Auto-detect error:', err);
      if (!silent) showNotice('error', `Auto detection failed: ${err.message}`);
    } finally {
      setIsDetecting(false);
    }
  };

  /** 去掉 box 提示，回到只用点提示的预测（点也为空则清空初稿）。 */
  const handleClearDetectedBox = () => {
    setCurrentBox(null);
    if (currentPoints.length > 0) predictMask(currentPoints, null);
    else setCurrentPolygons([]);
  };

  const handleSegImageClick = async (e: MouseEvent<SVGSVGElement>) => {
    if (!segMode || !natSize || !activeTemplate || isSpacePressed || isSamInitializing) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const x = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const y = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    const newPoints = [...currentPoints, { x, y, label: 1 }];
    setCurrentPoints(newPoints);
    predictMask(newPoints);
  };

  const handleSegContextMenu = (e: MouseEvent<SVGSVGElement>) => {
    if (!segMode || !natSize || !activeTemplate) return;
    e.preventDefault();
    const rect = e.currentTarget.getBoundingClientRect();
    const x = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const y = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    const newPoints = [...currentPoints, { x, y, label: 0 }];
    setCurrentPoints(newPoints);
    predictMask(newPoints);
  };

  const predictMask = async (pts: Point[], box: number[] | null = currentBox) => {
    if (!activeTemplate || (pts.length === 0 && !box)) {
      setCurrentPolygons([]);
      return;
    }
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/sam/predict`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          points: pts.map((p) => [p.x, p.y]),
          labels: pts.map((p) => p.label),
          box,
        }),
      });
      if (!res.ok) throw new Error('Predict failed');
      const data = await res.json();
      setCurrentPolygons(data.polygons || []);
    } catch (err) {
      console.error('SAM predict error:', err);
    }
  };

  const handleCommitCurrentSegMask = () => {
    if (currentPolygons.length === 0) return;
    const newMask: MaskData = {
      id: committedMasks.length + 1,
      points: [...currentPoints],
      labels: currentPoints.map((p) => p.label),
      box: currentBox || undefined,
      polygons: currentPolygons,
    };
    setCommittedMasks([...committedMasks, newMask]);
    setCurrentPoints([]);
    setCurrentPolygons([]);
    setCurrentBox(null);
  };

  const handleSaveAllSegMasks = async () => {
    if (!activeTemplate) return;
    try {
      const masksToSave: MaskData[] = [...committedMasks];
      if (currentPolygons.length > 0 && (currentPoints.length > 0 || currentBox)) {
        masksToSave.push({
          id: masksToSave.length + 1,
          points: [...currentPoints],
          labels: currentPoints.map((p) => p.label),
          box: currentBox || undefined,
          polygons: currentPolygons,
        });
      }

      if (masksToSave.length === 0) {
        showNotice('warning', 'No masks to save');
        return;
      }

      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/sam/save`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ committed_masks: masksToSave }),
      });
      if (!res.ok) throw new Error('Failed to save masks');
      setCommittedMasks(masksToSave);
      setCurrentPoints([]);
      setCurrentPolygons([]);
      setCurrentBox(null);
      setSavedMasks(masksToSave);
      setSegMode(false);
      await fetchTemplateFiles(activeTemplate);

      try {
        const sumRes = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/summary`);
        if (sumRes.ok) {
          const sumData = await sumRes.json();
          if (sumData.masks) setSavedMasks(sumData.masks);
        }
      } catch (e) {
        console.warn('Failed to refresh template summary after save masks:', e);
      }

      showNotice('success', `Saved ${masksToSave.length} mask(s)`);
    } catch (err: any) {
      showNotice('error', `Save failed: ${err.message}`);
    }
  };

  const fetchTemplateFiles = async (templateName: string) => {
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${templateName}/files`);
      if (res.ok) {
        const data = await res.json();
        setFiles(data.files || []);
      }
    } catch (err) {
      console.error('Failed to refresh files:', err);
    }
  };

  // ─── Manual TCP Path Handlers ───────────────────────────────────────────
  const handleManualMouseMove = (e: MouseEvent<SVGSVGElement>) => {
    if (!manualPathMode || !natSize || isSpacePressed) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);
    setMousePixel({ u, v });

    if (sessionData) {
      const res = computeNormalClientSide(sessionData, u, v, standoffDistMm);
      if (res) {
        setLiveNormal({
          dx: res.proj_dx,
          dy: res.proj_dy,
          surfPointBase: res.surf_base,
          normalBase: res.normal_base,
          tcpPose: {
            x: res.tcp_base[0],
            y: res.tcp_base[1],
            z: res.tcp_base[2],
            rx: res.euler_deg[0],
            ry: res.euler_deg[1],
            rz: res.euler_deg[2],
          },
        });
      }
    }
  };

  const handleManualImageClick = (e: MouseEvent<SVGSVGElement>) => {
    if (!manualPathMode || !natSize || isSpacePressed || !liveNormal?.surfPointBase || !liveNormal?.tcpPose) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    const newWaypoint: WaypointItem = {
      index: currentManualPoints.length + 1,
      pixel: [u, v],
      surface_point_cam_mm: [0, 0, 0],
      surface_point_base_mm: liveNormal.surfPointBase,
      surface_normal_base: liveNormal.normalBase || [0, 0, 1],
      surface_normal_cam: [0, 0, 1],
      standoff_distance_mm: standoffDistMm,
      tcp_pose_base: liveNormal.tcpPose,
      normal_2d_proj: [liveNormal.dx, liveNormal.dy],
    };

    setCurrentManualPoints([...currentManualPoints, newWaypoint]);
  };

  const handleCommitManualPath = () => {
    if (currentManualPoints.length === 0) return;
    if (selectedPathIdForEdit) {
      setManualPaths(
        manualPaths.map((p) =>
          p.path_id === selectedPathIdForEdit
            ? { ...p, points: currentManualPoints }
            : p
        )
      );
      setSelectedPathIdForEdit(null);
    } else {
      const newPathId = manualPaths.length > 0 ? Math.max(...manualPaths.map((p) => p.path_id)) + 1 : 1;
      const newPath: ManualPathItem = {
        path_id: newPathId,
        name: `Path ${newPathId}`,
        points: currentManualPoints,
      };
      setManualPaths([...manualPaths, newPath]);
    }
    setCurrentManualPoints([]);
  };

  const handleSaveManualPaths = async () => {
    if (!activeTemplate) return;
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/manual_paths?state_type=raw`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          paths: manualPaths,
        }),
      });
      if (!res.ok) throw new Error('Failed to save manual paths');
      setRawPaths(manualPaths);
      activatePathStateSnapshot('raw', manualPaths);
      setManualPathMode(false);
      if (onPathsUpdated) onPathsUpdated();
      await fetchTemplateFiles(activeTemplate);
      showNotice('success', `Saved ${manualPaths.length} path(s)`);
    } catch (err: any) {
      showNotice('error', `Save failed: ${err.message}`);
    }
  };

  // ─── Verification & POI Optimization Handlers ────────────────────────────
  const handleVerifyPath = async (targetState?: PathStateType) => {
    if (!activeTemplate) return;
    const st = targetState || activeState;
    setIsVerifying(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/verify_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          state_type: st,
        }),
      });
      if (!res.ok) {
        const errBody = await res.json().catch(() => null);
        throw new Error(errBody?.detail || 'Verification failed');
      }
      const data = await res.json();
      if (st === 'poi') setPoiReport(data);
      else if (st === 'auto_poi') setAutoPoiReport(data);
      else if (st === 'auto') setAutoReport(data);
      else setRawReport(data);

      await fetchTemplateFiles(activeTemplate);
      showNotice('success', `Verified (${data.summary?.status || data.status || 'PASS'})`);
    } catch (err: any) {
      showNotice('error', err.message);
    } finally {
      setIsVerifying(false);
    }
  };

  const handleOptimizePath = async (targetState?: PathStateType) => {
    if (!activeTemplate) return;
    const st = targetState || activeState;
    const source = pathSourceOf(st) === 'auto' ? 'auto' : 'raw';
    setIsOptimizing(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/optimize_paths?mode=poi`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          mode: 'poi',
          source,
        }),
      });
      if (!res.ok) {
        const errBody = await res.json().catch(() => null);
        throw new Error(errBody?.detail || 'POI Optimization failed');
      }
      const data = await res.json();
      const rep = data.verification_report || data;
      const paths = (data.optimized_paths || rep.optimized_paths || []) as ManualPathItem[];

      if (source === 'auto') {
        setAutoPoiPaths(paths);
        setAutoPoiReport(rep);
        activatePathStateSnapshot('auto_poi', paths);
      } else {
        setPoiPaths(paths);
        setPoiReport(rep);
        activatePathStateSnapshot('poi', paths);
      }

      if (onPathsUpdated) onPathsUpdated();
      await fetchTemplateFiles(activeTemplate);

      const isPass = rep.status === 'PASS' || rep.summary?.status === 'PASS';
      if (isPass) {
        showNotice('success', `POI optimized (${source === 'auto' ? 'AUTO-POI' : 'POI'} PASS)`);
      } else {
        const issueCount = rep.summary?.total_issues ?? rep.issues?.length ?? 1;
        showNotice('error', `Optimization FAILED (${issueCount} kinematic issues). Robot execution blocked.`);
      }
    } catch (err: any) {
      showNotice('error', `Optimization failed: ${err.message}`);
      if (activeTemplate) await fetchTemplateFiles(activeTemplate);
    } finally {
      setIsOptimizing(false);
    }
  };

  // ─── 3D/2D Synchronized Simulation Engine (Feature 7) ────────────────────
  const startSimulation = (
    stateType: PathStateType = activeState,
    targetPathId?: number | null,
    overrideReport?: VerificationReport | null
  ) => {
    if (followJoints) {
      showNotice('warning', 'Please stop Follow first');
      return;
    }
    stopSimulation();
    const rep = overrideReport
      || (stateType === 'poi' ? poiReport
        : (stateType === 'auto_poi' ? autoPoiReport
          : (stateType === 'auto' ? autoReport : rawReport)));

    // Build dense playback steps
    const simSteps: Array<{
      q_deg: number[];
      tcp: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
      pixel: [number, number] | null;
      pathIdx: number;
    }> = [];

    if (rep?.path_reports && rep.path_reports.length > 0) {
      // Filter target path reports if a specific path is requested
      const targetReports = (targetPathId !== undefined && targetPathId !== null)
        ? rep.path_reports.filter((pr: any, idx: number) => (pr.path_id === targetPathId || idx + 1 === targetPathId))
        : rep.path_reports;

      targetReports.forEach((pr, pIdx) => {
        const realPIdx = rep.path_reports ? rep.path_reports.indexOf(pr) : pIdx;
        const tq = pr.trajectory_q || [];
        const tt = pr.trajectory_tcp || [];
        const totalPSteps = Math.min(tq.length, tt.length);

        for (let s = 0; s < totalPSteps; s++) {
          const q_rad = tq[s];
          const q_deg = q_rad.map((r) => (r * 180.0) / Math.PI);
          const tcpArr = tt[s];
          const tcpPose = {
            x: tcpArr[0],
            y: tcpArr[1],
            z: tcpArr[2],
            rx: tcpArr[3],
            ry: tcpArr[4],
            rz: tcpArr[5],
          };

          // Project base 3D point to 2D pixel
          let pixelProj: [number, number] | null = null;
          if (sessionData) {
            pixelProj = projectBasePointToPixel(
              [tcpArr[0], tcpArr[1], tcpArr[2]],
              sessionData.T,
              { fx: sessionData.fx, fy: sessionData.fy, cx: sessionData.cx, cy: sessionData.cy }
            );
          }

          simSteps.push({
            q_deg,
            tcp: tcpPose,
            pixel: pixelProj,
            pathIdx: realPIdx >= 0 ? realPIdx : pIdx,
          });
        }
      });
    } else if (bypassVerification) {
      // Direct waypoint simulation when kinematics verification is bypassed
      const srcPaths = pathsForState(stateType);
      const filteredPaths = (targetPathId !== undefined && targetPathId !== null)
        ? srcPaths.filter((p) => p.path_id === targetPathId)
        : srcPaths;

      filteredPaths.forEach((path, pIdx) => {
        const points = path.points || [];
        for (let i = 0; i < points.length; i++) {
          const pt = points[i];
          const nextPt = i < points.length - 1 ? points[i + 1] : null;

          // Dense interpolation between consecutive waypoints for smooth visualization (~10 sub-steps)
          const subSteps = nextPt ? 10 : 1;
          for (let s = 0; s < subSteps; s++) {
            const alpha = s / subSteps;
            const interpTcp = nextPt ? {
              x: pt.tcp_pose_base.x + (nextPt.tcp_pose_base.x - pt.tcp_pose_base.x) * alpha,
              y: pt.tcp_pose_base.y + (nextPt.tcp_pose_base.y - pt.tcp_pose_base.y) * alpha,
              z: pt.tcp_pose_base.z + (nextPt.tcp_pose_base.z - pt.tcp_pose_base.z) * alpha,
              rx: pt.tcp_pose_base.rx + (nextPt.tcp_pose_base.rx - pt.tcp_pose_base.rx) * alpha,
              ry: pt.tcp_pose_base.ry + (nextPt.tcp_pose_base.ry - pt.tcp_pose_base.ry) * alpha,
              rz: pt.tcp_pose_base.rz + (nextPt.tcp_pose_base.rz - pt.tcp_pose_base.rz) * alpha,
            } : pt.tcp_pose_base;

            let pixelProj: [number, number] | null = null;
            if (sessionData) {
              pixelProj = projectBasePointToPixel(
                [interpTcp.x, interpTcp.y, interpTcp.z],
                sessionData.T,
                { fx: sessionData.fx, fy: sessionData.fy, cx: sessionData.cx, cy: sessionData.cy }
              );
            } else if (pt.pixel) {
              pixelProj = pt.pixel;
            }

            simSteps.push({
              q_deg: [0, 0, 0, 0, 0, 0],
              tcp: interpTcp,
              pixel: pixelProj,
              pathIdx: path.path_id !== undefined ? path.path_id : pIdx,
            });
          }
        }
      });
    } else {
      showNotice('info', `No trajectory for ${stateType.toUpperCase()}. Verify first.`);
      return;
    }

    if (simSteps.length === 0) {
      showNotice('info', `No steps for ${stateType.toUpperCase()}`);
      return;
    }

    simDataRef.current = {
      steps: simSteps,
      stepIndex: 0,
      speedMultiplier: simulationState?.speed || 1.0,
      isPlaying: true,
      stateType: stateType,
    };

    setSimulationState({
      isPlaying: true,
      progress: 0,
      speed: simDataRef.current.speedMultiplier,
      currentPathIndex: 0,
      currentStep: 0,
      totalSteps: simSteps.length,
      currentJoints: simSteps[0].q_deg,
      currentTcpPose: simSteps[0].tcp,
      currentPixel: simSteps[0].pixel,
      activeState: stateType,
    });

    // Start 60 FPS animation loop
    let lastTime = performance.now();
    const animate = (time: number) => {
      const dt = (time - lastTime) / 1000.0;
      lastTime = time;

      const sim = simDataRef.current;
      if (!sim.isPlaying || sim.steps.length === 0) return;

      // Advance step based on speed multiplier (nominal MoveL step rate ~ 60 steps/sec)
      const stepIncrement = Math.max(1, Math.round(60 * dt * sim.speedMultiplier));
      sim.stepIndex += stepIncrement;

      if (sim.stepIndex >= sim.steps.length) {
        sim.stepIndex = sim.steps.length - 1;
        sim.isPlaying = false;
      }

      const curr = sim.steps[sim.stepIndex];
      const prog = sim.stepIndex / (sim.steps.length - 1);

      setSimulationState({
        isPlaying: sim.isPlaying,
        progress: prog,
        speed: sim.speedMultiplier,
        currentPathIndex: curr.pathIdx,
        currentStep: sim.stepIndex,
        totalSteps: sim.steps.length,
        currentJoints: curr.q_deg,
        currentTcpPose: curr.tcp,
        currentPixel: curr.pixel,
        activeState: sim.stateType,
      });

      if (onSimulationJointsChange) {
        onSimulationJointsChange(curr.q_deg);
      }

      simAnimFrameRef.current = requestAnimationFrame(animate);
    };

    simAnimFrameRef.current = requestAnimationFrame(animate);
  };

  const pauseSimulation = () => {
    if (simAnimFrameRef.current) cancelAnimationFrame(simAnimFrameRef.current);
    simDataRef.current.isPlaying = false;
    setSimulationState((prev) => (prev ? { ...prev, isPlaying: false } : null));
  };

  const resumeSimulation = () => {
    if (!simulationState || simDataRef.current.steps.length === 0) return;
    simDataRef.current.isPlaying = true;
    setSimulationState((prev) => (prev ? { ...prev, isPlaying: true } : null));

    let lastTime = performance.now();
    const animate = (time: number) => {
      const dt = (time - lastTime) / 1000.0;
      lastTime = time;

      const sim = simDataRef.current;
      if (!sim.isPlaying || sim.steps.length === 0) return;

      const stepIncrement = Math.max(1, Math.round(60 * dt * sim.speedMultiplier));
      sim.stepIndex += stepIncrement;

      if (sim.stepIndex >= sim.steps.length) {
        sim.stepIndex = sim.steps.length - 1;
        sim.isPlaying = false;
      }

      const curr = sim.steps[sim.stepIndex];
      const prog = sim.stepIndex / (sim.steps.length - 1);

      setSimulationState({
        isPlaying: sim.isPlaying,
        progress: prog,
        speed: sim.speedMultiplier,
        currentPathIndex: curr.pathIdx,
        currentStep: sim.stepIndex,
        totalSteps: sim.steps.length,
        currentJoints: curr.q_deg,
        currentTcpPose: curr.tcp,
        currentPixel: curr.pixel,
        activeState: sim.stateType,
      });

      if (onSimulationJointsChange) {
        onSimulationJointsChange(curr.q_deg);
      }

      simAnimFrameRef.current = requestAnimationFrame(animate);
    };

    simAnimFrameRef.current = requestAnimationFrame(animate);
  };

  const seekSimulation = (targetProgress: number) => {
    const sim = simDataRef.current;
    if (sim.steps.length === 0) return;
    const targetIdx = Math.min(
      Math.max(0, Math.floor(targetProgress * (sim.steps.length - 1))),
      sim.steps.length - 1
    );
    sim.stepIndex = targetIdx;
    const curr = sim.steps[targetIdx];

    setSimulationState((prev) =>
      prev
        ? {
          ...prev,
          progress: targetProgress,
          currentPathIndex: curr.pathIdx,
          currentStep: targetIdx,
          currentJoints: curr.q_deg,
          currentTcpPose: curr.tcp,
          currentPixel: curr.pixel,
        }
        : null
    );

    if (onSimulationJointsChange) {
      onSimulationJointsChange(curr.q_deg);
    }
  };

  const stopSimulation = () => {
    if (simAnimFrameRef.current) cancelAnimationFrame(simAnimFrameRef.current);
    simDataRef.current.isPlaying = false;
    simDataRef.current.steps = [];
    setSimulationState(null);
    if (onSimulationJointsChange) {
      onSimulationJointsChange(null);
    }
  };

  const setSimulationSpeed = (speed: number) => {
    simDataRef.current.speedMultiplier = speed;
    setSimulationState((prev) => (prev ? { ...prev, speed } : null));
  };

  // ─── Direct Robot Path Execution with 2D Live Progress ────────────────────
  const handleDirectExecutePath = async (fileName: string, stateType?: PathStateType, pathId?: number | null) => {
    if (!activeTemplate || isExecutingPath || robotStatus === 1) return;
    const targetState = stateType || activeState;
    handleSelectActiveState(targetState);

    // Safety validation guard: Physical robot execution requires verified PASS unless bypass is enabled
    if (!bypassVerification) {
      const rep = targetState === 'poi' ? poiReport : (targetState === 'auto_poi' ? autoPoiReport : (targetState === 'auto' ? autoReport : rawReport));
      if (!rep || (rep.summary?.status !== 'PASS' && rep.status !== 'PASS')) {
        showNotice('warning', `Unverified: Verify ${targetState.toUpperCase()} first`);
        return;
      }
    }

    // Lock execution button immediately and initiate action HUD
    setIsExecutingPath(true);
    if (execActionTimerRef.current) {
      clearTimeout(execActionTimerRef.current);
      execActionTimerRef.current = null;
    }
    setExecutingAction({
      text: 'Initializing spray trajectory...',
      stage: 'preparing',
      progress: 0,
    });

    // Count total waypoints in the target paths for progress state initialisation
    const srcPaths = pathsForState(targetState);
    const filteredPaths = (pathId !== null && pathId !== undefined)
      ? srcPaths.filter((p) => p.path_id === pathId)
      : srcPaths;
    const totalWaypoints = filteredPaths.reduce((sum, p) => sum + (p.points?.length ?? 0), 0);

    // 1. Enter real-exec mode: show trajectory overlay, drive dot from real WS TCP data
    stopSimulation();
    setSimulationState({
      isPlaying: true,
      isRealExec: true,
      progress: 0,
      speed: 1.0,
      currentPathIndex: 0,
      currentStep: 0,
      totalSteps: Math.max(totalWaypoints, 1),
      currentJoints: [0, 0, 0, 0, 0, 0],
      currentTcpPose: { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 },
      currentPixel: null,
      activeState: targetState,
    });

    // 2. Dispatch real robot execution asynchronously
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/execute_yaml_path`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          file_name: fileName,
          path_id: pathId,
          skip_verification: bypassVerification,
        }),
      });
      if (!res.ok) {
        const err = await res.json().catch(() => ({ detail: 'Execution failed' }));
        console.error('Robot path execution error:', err.detail || 'Execution failed');
        setExecutingAction({
          text: `Execution failed: ${err.detail || 'Execution error'}`,
          stage: 'error',
          isError: true,
        });
        setIsExecutingPath(false);
        if (execActionTimerRef.current) clearTimeout(execActionTimerRef.current);
        execActionTimerRef.current = setTimeout(() => {
          setExecutingAction(null);
        }, 5000);
      }
    } catch (err: any) {
      console.error('Failed to execute path on robot:', err);
      setExecutingAction({
        text: `Execution failed: ${err.message || 'Network connection error'}`,
        stage: 'error',
        isError: true,
      });
      setIsExecutingPath(false);
      if (execActionTimerRef.current) clearTimeout(execActionTimerRef.current);
      execActionTimerRef.current = setTimeout(() => {
        setExecutingAction(null);
      }, 5000);
    } finally {
      // Mark done when HTTP completes (WS exec_progress should have already done this)
      setSimulationState((prev) => prev?.isRealExec ? { ...prev, isPlaying: false } : prev);
    }
  };


  // ─── Actions: Capture & Reconstruct ─────────────────────────────────────
  const handleTriggerCapture = async () => {
    if (!activeTemplate) return;
    setIsCapturing(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/capture`, {
        method: 'POST',
      });
      if (!res.ok) throw new Error('Capture failed');
      await handleSelectTemplate(activeTemplate);
      showNotice('success', 'Camera captured');
    } catch (err: any) {
      showNotice('error', err.message);
    } finally {
      setIsCapturing(false);
    }
  };

  const handleTriggerReconstruct = async () => {
    if (!activeTemplate) return;
    if (!hasMask) {
      showNotice('warning', 'Segment mask required to reconstruct 3D');
      return;
    }
    const startTime = performance.now();
    console.log(`[3D Reconstruction] Clicked reconstruct button for template '${activeTemplate}' at ${new Date().toISOString()}`);
    setIsReconstructing(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/reconstruct`, {
        method: 'POST',
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) {
        throw new Error(typeof data.detail === 'string' ? data.detail : 'Reconstruction failed');
      }
      const elapsedSec = ((performance.now() - startTime) / 1000).toFixed(2);
      console.log(`[3D Reconstruction] Completed successfully in ${elapsedSec}s for template '${activeTemplate}'. Mesh:`, {
        vertices: data.vertices,
        faces: data.faces,
        is_watertight: data.is_watertight,
        backend_elapsed_seconds: data.elapsed_seconds,
        timings: data.timings,
      });

      // Refresh template files to update hasMesh without full-screen flash or RGB image re-download
      await fetchTemplateFiles(activeTemplate);
      if (onMeshUpdated) onMeshUpdated();
      showNotice('success', `3D mesh ready (${elapsedSec}s)`);
    } catch (err: any) {
      const elapsedSec = ((performance.now() - startTime) / 1000).toFixed(2);
      console.error(`[3D Reconstruction] Failed after ${elapsedSec}s:`, err);
      showNotice('error', err.message);
    } finally {
      setIsReconstructing(false);
    }
  };

  const runAutoPathGeneration = async () => {
    if (!activeTemplate) return;
    if (!hasMesh) {
      showNotice('warning', '3D mesh required to generate spray paths');
      return;
    }
    if (!hasMask) {
      showNotice('warning', 'Segment mask required to generate spray paths');
      return;
    }
    setIsAutoGenerating(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/auto_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({}),
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) {
        throw new Error(typeof data.detail === 'string' ? data.detail : 'Auto path generation failed');
      }
      await handleSelectTemplate(activeTemplate);
      const generated = (data.paths || []) as ManualPathItem[];
      setAutoPaths(generated);
      activatePathStateSnapshot('auto', generated);
      if (onPathsUpdated) onPathsUpdated();
      showNotice('success', `Generated ${data.point_count ?? 0} waypoints`);
    } catch (err: any) {
      showNotice('error', err.message);
    } finally {
      setIsAutoGenerating(false);
    }
  };

  const handleDeleteFile = async (fileName: string) => {
    if (!activeTemplate) return;
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/files/${fileName}`, {
        method: 'DELETE',
      });
      if (!res.ok) throw new Error('Failed to delete file');
      if (fileName.includes('mesh') || fileName.endsWith('.ply') || fileName.endsWith('.stl')) {
        if (onMeshUpdated) onMeshUpdated();
      }
      if (fileName.includes('path')) {
        if (onPathsUpdated) onPathsUpdated();
      }
      await handleSelectTemplate(activeTemplate);
      showNotice('success', `Deleted ${fileName}`);
    } catch (err: any) {
      showNotice('error', err.message);
    }
  };

  const handleCleanTemplateFiles = async () => {
    if (!activeTemplate) return;
    setIsCleaningFiles(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/clean`, {
        method: 'POST',
      });
      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        throw new Error(err.detail || 'Failed to clean template files');
      }
      const data = await res.json();
      if (onMeshUpdated) onMeshUpdated();
      if (onPathsUpdated) onPathsUpdated();
      await handleSelectTemplate(activeTemplate);
      const deletedCount = data.deleted ? data.deleted.length : 0;
      showNotice('success', `Cleaned ${deletedCount} files`);
    } catch (err: any) {
      showNotice('error', err.message);
    } finally {
      setIsCleaningFiles(false);
    }
  };

  const handleTriggerAutoPath = () => {
    if (!activeTemplate) return;
    void runAutoPathGeneration();
  };


  return (
    <div className="flex-1 flex flex-col h-full w-full bg-slate-950 overflow-hidden relative font-sans select-none">
      {/* Top Horizontal Template Switcher */}
      <TemplateTopBar
        templates={templates}
        activeTemplate={activeTemplate}
        onSelectTemplate={handleSelectTemplate}
        onCreateTemplate={async (name) => {
          if (!name.trim()) return;
          try {
            const res = await fetch(`${API_BASE}/api/interactive/templates`, {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({ name: name.trim() }),
            });
            if (!res.ok) throw new Error('Failed to create template');
            await fetchTemplates(name.trim());
            showNotice('success', `Created ${name.trim()}`);
          } catch (err: any) {
            showNotice('error', err.message);
          }
        }}
        onDeleteTemplate={async (t) => {
          try {
            const res = await fetch(`${API_BASE}/api/interactive/templates/${t}`, {
              method: 'DELETE',
            });
            if (!res.ok) throw new Error('Failed to delete template');
            setActiveTemplate(null);
            await fetchTemplates();
            showNotice('success', `Deleted ${t}`);
          } catch (err: any) {
            showNotice('error', err.message);
          }
        }}
      />

      {/* Main Operational 3-Column Area */}
      <div className="flex-1 flex overflow-hidden relative min-h-0">
        {/* Column 1: 2D Image Viewport & Overlays */}
        <InteractiveCanvas
          imageUrl={imageUrl}
          isLoadingTemplate={isLoadingTemplate}
          isCapturing={isCapturing}
          isReconstructing={isReconstructing}
          isAutoGenerating={isAutoGenerating}
          isVerifying={isVerifying}
          isOptimizing={isOptimizing}
          executingAction={executingAction}
          canvasNotice={canvasNotice}
          onDismissNotice={() => setCanvasNotice(null)}
          segMode={segMode}
          manualPathMode={manualPathMode}
          showMasksOverlay={showMasksOverlay}
          showManualPathsOverlay={showManualPathsOverlay}
          savedMasks={savedMasks}
          committedMasks={committedMasks}
          currentPolygons={currentPolygons}
          currentPoints={currentPoints}
          currentBox={currentBox}
          isDetecting={isDetecting}
          manualPaths={manualPaths}
          currentManualPoints={currentManualPoints}
          selectedPathIdForEdit={selectedPathIdForEdit}
          highlightedPathId={highlightedPathId}
          hoveredWaypoint={hoveredWaypoint}
          mousePixel={mousePixel}
          liveNormal={liveNormal}
          natSize={natSize}
          verificationReport={
            activeState === 'poi' ? poiReport
              : (activeState === 'auto_poi' ? autoPoiReport
                : (activeState === 'auto' ? autoReport : rawReport))
          }
          activeState={activeState}
          simulationState={simulationState}
          sessionData={sessionData}
          onSelectActiveState={handleSelectActiveState}
          zoom={zoom}
          pan={pan}
          isPanning={isPanning}
          isSpacePressed={isSpacePressed}
          setZoom={setZoom}
          setPan={setPan}
          setIsPanning={setIsPanning}
          setNatSize={setNatSize}
          setHighlightedPathId={setHighlightedPathId}
          setHoveredWaypoint={setHoveredWaypoint}
          onSelectPathForEdit={(pathId) => {
            const p = manualPaths.find((it) => it.path_id === pathId);
            if (p) {
              setSelectedPathIdForEdit(pathId);
              setCurrentManualPoints([...p.points]);
              setManualPathMode(true);
            }
          }}
          onManualMouseMove={handleManualMouseMove}
          onManualImageClick={handleManualImageClick}
          onDeleteSingleWaypoint={(idx) => {
            const updated = currentManualPoints.filter((_, i) => i !== idx);
            setCurrentManualPoints(updated.map((wp, i) => ({ ...wp, index: i + 1 })));
          }}
          onSegImageClick={handleSegImageClick}
          onSegContextMenu={handleSegContextMenu}
          onToggleMasksOverlay={() => setShowMasksOverlay(!showMasksOverlay)}
          onToggleManualPathsOverlay={() => setShowManualPathsOverlay(!showManualPathsOverlay)}
          onUndoSegPoint={() => {
            if (currentPoints.length === 0) return;
            const updated = currentPoints.slice(0, -1);
            setCurrentPoints(updated);
            // 还带着检测框时回到 box 初稿，而不是直接清空
            if (updated.length > 0 || currentBox) predictMask(updated, currentBox);
            else setCurrentPolygons([]);
          }}
          onClearCurrentSegPoints={() => {
            setCurrentPoints([]);
            predictMask([], currentBox);
          }}
          onDetectBox={() => detectBox(false)}
          onClearDetectedBox={handleClearDetectedBox}
          onCommitCurrentSegMask={handleCommitCurrentSegMask}
          onSaveAllSegMasks={handleSaveAllSegMasks}
          onClearAllMasks={() => {
            setCommittedMasks([]);
            setCurrentPoints([]);
            setCurrentPolygons([]);
            setCurrentBox(null);
          }}
          onExitSegMode={() => {
            setSegMode(false);
            setCurrentPoints([]);
            setCurrentPolygons([]);
            setCurrentBox(null);
          }}
          onUndoManualPoint={() => {
            if (currentManualPoints.length === 0) return;
            setCurrentManualPoints(currentManualPoints.slice(0, -1));
          }}
          onClearCurrentManualPoints={() => setCurrentManualPoints([])}
          onCommitManualPath={handleCommitManualPath}
          onSaveManualPaths={handleSaveManualPaths}
          onDeleteCurrentPath={() => {
            if (selectedPathIdForEdit) {
              setManualPaths(manualPaths.filter((p) => p.path_id !== selectedPathIdForEdit));
              setSelectedPathIdForEdit(null);
              setCurrentManualPoints([]);
            }
          }}
          onExitManualPathMode={() => {
            setManualPathMode(false);
            setCurrentManualPoints([]);
            setSelectedPathIdForEdit(null);
          }}
        />

        {/* 3D/2D Synchronized Simulation Floating Playback Control Bar (Feature 7) */}
        {simulationState && (
          <div className="absolute top-4 left-1/2 -translate-x-1/2 z-40 bg-slate-950/90 backdrop-blur-md border border-sky-500/40 rounded-full px-4 py-1.5 shadow-2xl flex items-center gap-3 text-slate-200 text-xs select-none">
            {/* Play/Pause Button */}
            <button
              onClick={simulationState.isPlaying ? pauseSimulation : resumeSimulation}
              className="p-1.5 rounded-full bg-sky-600 hover:bg-sky-500 text-white shadow transition-all"
              title={simulationState.isPlaying ? 'Pause Simulation' : 'Resume Simulation'}
            >
              {simulationState.isPlaying ? <Pause size={12} /> : <Play size={12} className="fill-white" />}
            </button>

            {/* State Badge */}
            <span
              className={`text-[9px] font-bold font-mono px-1.5 py-0.5 rounded border ${STATE_THEMES[simulationState.activeState].bg
                } ${STATE_THEMES[simulationState.activeState].text} ${STATE_THEMES[simulationState.activeState].border}`}
            >
              {simulationState.activeState.toUpperCase()} SIM
            </span>

            {/* Step & Progress Scrubber */}
            <div className="flex items-center gap-2">
              <input
                type="range"
                min={0}
                max={1}
                step={0.002}
                value={simulationState.progress}
                onChange={(e) => seekSimulation(parseFloat(e.target.value))}
                className="w-32 accent-sky-400 h-1.5 bg-slate-800 rounded cursor-pointer"
              />
              <span className="text-[10px] font-mono text-slate-400 w-16 text-right">
                {Math.round(simulationState.progress * 100)}% ({simulationState.currentStep}/{simulationState.totalSteps})
              </span>
            </div>

            {/* Speed Multiplier Options */}
            <div className="flex items-center gap-1 bg-slate-900 rounded p-0.5 border border-slate-800 text-[9.5px] font-mono">
              {[0.5, 1.0, 2.0, 5.0].map((spd) => (
                <button
                  key={spd}
                  onClick={() => setSimulationSpeed(spd)}
                  className={`px-1.5 py-0.5 rounded transition-all ${simulationState.speed === spd
                      ? 'bg-sky-600 text-white font-bold'
                      : 'text-slate-400 hover:text-slate-200'
                    }`}
                >
                  {spd}x
                </button>
              ))}
            </div>

            {/* Close / Stop Button */}
            <button
              onClick={stopSimulation}
              className="p-1 text-slate-400 hover:text-rose-400 transition-colors ml-1"
              title="Stop Simulation"
            >
              <X size={14} />
            </button>
          </div>
        )}

        {/* Right Side Panel (w-[320px]): Top Action Toolbar + File List */}
        <div className="w-[320px] shrink-0 border-l border-slate-800 bg-slate-950/40 flex flex-col h-full min-h-0 overflow-hidden">
          {/* Top Horizontal Action Toolbar */}
          <InteractiveActionColumn
            hasImage={hasImage}
            hasMask={hasMask}
            hasMesh={hasMesh}
            activeTemplate={activeTemplate}
            isCapturing={isCapturing}
            isReconstructing={isReconstructing}
            isAutoGenerating={isAutoGenerating}
            segMode={segMode}
            manualPathMode={manualPathMode}
            onTriggerCapture={handleTriggerCapture}
            onToggleSegMode={handleToggleSegMode}
            onToggleManualPathMode={() => {
              if (manualPathMode) {
                setCurrentManualPoints([]);
                setSelectedPathIdForEdit(null);
              } else {
                if (activeTemplate && !sessionData) {
                  loadSessionData(activeTemplate);
                }
              }
              setManualPathMode(!manualPathMode);
            }}
            onTriggerReconstruct={handleTriggerReconstruct}
            onTriggerAutoPath={handleTriggerAutoPath}
          />

          {/* Panel Content (File List) */}
          <div className="flex-1 min-h-0 overflow-hidden flex flex-col">
            <TemplateFileList
              files={files}
              activeState={activeState}
              templateName={activeTemplate}
              onCleanGeneratedFiles={handleCleanTemplateFiles}
              isCleaning={isCleaningFiles}
              onDeleteFile={handleDeleteFile}
              rawPaths={rawPaths}
              autoPaths={autoPaths}
              autoPoiPaths={autoPoiPaths}
              poiPaths={poiPaths}
              rawReport={rawReport}
              autoReport={autoReport}
              poiReport={poiReport}
              autoPoiReport={autoPoiReport}
              robotConnected={robotConnected}
              isVerifying={isVerifying}
              isOptimizing={isOptimizing}
              isExecuting={isExecutingPath}
              isRobotMoving={robotStatus === 1}
              onVerifyPath={handleVerifyPath}
              onOptimizePath={handleOptimizePath}
              onSimulatePath={(st, pId) => startSimulation(st, pId)}
              onExecutePath={(fileName, st, pId) => handleDirectExecutePath(fileName, st, pId)}
              bypassVerification={bypassVerification}
              onToggleBypassVerification={setBypassVerification}
              onSelectFile={(fileName) => {
                if (fileName.includes('auto.poi')) {
                  handleSelectActiveState('auto_poi');
                } else if (fileName.includes('manual.poi') || fileName.includes('poi.path')) {
                  handleSelectActiveState('poi');
                } else if (fileName.includes('auto.path')) {
                  handleSelectActiveState('auto');
                } else if (fileName.includes('manual.path') || fileName.includes('raw.path') || fileName.includes('manual_paths')) {
                  handleSelectActiveState('raw');
                }
              }}
            />
          </div>

          {/* Follow 行：贴在文件列表底下。放在 flex-1 容器**外面** —— 文件多了要能滚，
              而这排按钮是常驻控制，不该被列表挤下去。 */}
          <FollowPanel
            isReplaying={!!simulationState}
            onFollowJoints={handleFollowJoints}
          />
        </div>
      </div>

    </div>
  );
};

export default InteractiveOp;
export * from './interactive/types';
