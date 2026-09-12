cmake_minimum_required(VERSION 3.16)
project(aisprayer_motion LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/platform.cmake")

if(AIS_ARCH_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AIS_ARCH_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2 REQUIRED)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)

add_library(motion_core STATIC
  src/kinematics.cpp
  src/segment.cpp
  src/verifier.cpp
  src/optimizer.cpp
)
target_include_directories(motion_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(motion_core PUBLIC Eigen3::Eigen)
# 禁止改变浮点语义的优化（黄金回归要求）
target_compile_options(motion_core PRIVATE -fno-fast-math)

add_library(motion_io STATIC
  src/io_urdf.cpp
  src/io_path.cpp
  src/io_json.cpp
)
target_include_directories(motion_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(motion_io PUBLIC motion_core yaml-cpp tinyxml2::tinyxml2)

add_executable(motion_cli src/cli_main.cpp src/cli_report.cpp)
target_link_libraries(motion_cli PRIVATE motion_io)
target_include_directories(motion_cli PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_library(motion_c SHARED src/c_api.cpp)
target_link_libraries(motion_c PRIVATE motion_core)
set_target_properties(motion_c PROPERTIES OUTPUT_NAME motion_c)

enable_testing()
add_executable(test_kinematics tests/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE motion_core)
add_test(NAME test_kinematics COMMAND test_kinematics)

add_executable(test_continuity tests/test_continuity.cpp)
target_link_libraries(test_continuity PRIVATE motion_core)
add_test(NAME test_continuity COMMAND test_continuity)

add_executable(test_golden_verify tests/test_golden_verify.cpp)
target_link_libraries(test_golden_verify PRIVATE motion_io)
target_compile_definitions(test_golden_verify PRIVATE REPO_ROOT="${REPO_ROOT}")
add_test(NAME test_golden_verify COMMAND test_golden_verify)

add_executable(test_golden_optimize tests/test_golden_optimize.cpp)
target_link_libraries(test_golden_optimize PRIVATE motion_io)
target_compile_definitions(test_golden_optimize PRIVATE REPO_ROOT="${REPO_ROOT}")
add_test(NAME test_golden_optimize COMMAND test_golden_optimize)

# 容差阶梯择优（大容差不得劣于小容差）的验收测试，跑真实工件数据，约 10 s。
add_executable(test_tol_ladder tests/test_tol_ladder.cpp)
target_link_libraries(test_tol_ladder PRIVATE motion_io)
target_compile_definitions(test_tol_ladder PRIVATE REPO_ROOT="${REPO_ROOT}")
add_test(NAME test_tol_ladder COMMAND test_tol_ladder)

# 姿态契约：原姿态优先、零包络严格，以及无解点只能在包络内修复。
add_executable(test_orientation_contract tests/test_orientation_contract.cpp)
target_link_libraries(test_orientation_contract PRIVATE motion_io)
target_compile_definitions(test_orientation_contract PRIVATE REPO_ROOT="${REPO_ROOT}")
add_test(NAME test_orientation_contract COMMAND test_orientation_contract)
