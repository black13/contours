cmake_minimum_required(VERSION 3.16)
project(Contours VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(FREESTYLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/freestyle)

# ===================================================================
# OpenSubdiv (tess_RifFilter backend)
# ===================================================================

# ---- OpenSubdiv: hbr (header-only) ----
add_library(hbr INTERFACE)
target_include_directories(hbr INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/tess_RifFilter/opensubdiv
)

# ---- OpenSubdiv: osdutil (static lib, minimal) ----
set(OSD_DIR tess_RifFilter/opensubdiv/osd)
set(OSDUTIL_DIR tess_RifFilter/opensubdiv/osdutil)
set(OSDUTIL_SOURCES
    ${OSDUTIL_DIR}/adaptiveEvaluator.cpp ${OSDUTIL_DIR}/topology.cpp
    ${OSDUTIL_DIR}/refiner.cpp           ${OSDUTIL_DIR}/mesh.cpp
    ${OSD_DIR}/cpuComputeContext.cpp     ${OSD_DIR}/cpuComputeController.cpp
    ${OSD_DIR}/cpuEvalLimitContext.cpp   ${OSD_DIR}/cpuEvalLimitController.cpp
    ${OSD_DIR}/cpuEvalLimitKernel.cpp    ${OSD_DIR}/cpuKernel.cpp
    ${OSD_DIR}/cpuVertexBuffer.cpp       ${OSD_DIR}/evalLimitContext.cpp
    ${OSD_DIR}/error.cpp
)
add_library(osdutil STATIC ${OSDUTIL_SOURCES})
target_include_directories(osdutil PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/tess_RifFilter/opensubdiv
)
target_link_libraries(osdutil PUBLIC hbr)

# ---- tess_RifFilter core (no RenderMan) ----
add_library(tess_RifFilter STATIC
    tess_RifFilter/refineContour.cpp tess_RifFilter/subdiv.cpp
    tess_RifFilter/vdtess.cpp        tess_RifFilter/cameraModel.cpp
    tess_RifFilter/refineContour.h   tess_RifFilter/refineContourFunctions.h
    tess_RifFilter/subdiv.h          tess_RifFilter/cameraModel.h
    tess_RifFilter/VecMat.h          tess_RifFilter/chart.h
    tess_RifFilter/chartFunctions.h  tess_RifFilter/meshFunctions.h
    tess_RifFilter/paramPoint.h      tess_RifFilter/paramPointFunctions.h
)
target_include_directories(tess_RifFilter PUBLIC tess_RifFilter)
target_link_libraries(tess_RifFilter PUBLIC osdutil)

# ===================================================================
# Freestyle NPR libraries (pure C++11, no Qt/Python/OpenGL/SWIG)
# ===================================================================

# --- 1. geometry (pure math, no external deps) ---
add_library(fs_geometry STATIC
    ${FREESTYLE_DIR}/geometry/Bezier.cpp
    ${FREESTYLE_DIR}/geometry/FastGrid.cpp
    ${FREESTYLE_DIR}/geometry/FitCurve.cpp
    ${FREESTYLE_DIR}/geometry/GeomCleaner.cpp
    ${FREESTYLE_DIR}/geometry/GeomUtils.cpp
    ${FREESTYLE_DIR}/geometry/Grid.cpp
    ${FREESTYLE_DIR}/geometry/HashGrid.cpp
    ${FREESTYLE_DIR}/geometry/Noise.cpp
    ${FREESTYLE_DIR}/geometry/matrix_util.cpp
    ${FREESTYLE_DIR}/geometry/normal_cycle.cpp
)
target_include_directories(fs_geometry PUBLIC ${FREESTYLE_DIR})

# --- 2. image (image pyramid, no external deps) ---
add_library(fs_image STATIC
    ${FREESTYLE_DIR}/image/GaussianFilter.cpp
    ${FREESTYLE_DIR}/image/ImagePyramid.cpp
)
target_include_directories(fs_image PUBLIC ${FREESTYLE_DIR})
target_link_libraries(fs_image PUBLIC fs_geometry)

# --- 3. scene_graph (PLY loading + scene representation) ---
add_library(fs_scene_graph STATIC
    ${FREESTYLE_DIR}/scene_graph/IndexedFaceSet.cpp
    ${FREESTYLE_DIR}/scene_graph/LineRep.cpp
    ${FREESTYLE_DIR}/scene_graph/NodeCamera.cpp
    ${FREESTYLE_DIR}/scene_graph/NodeDrawingStyle.cpp
    ${FREESTYLE_DIR}/scene_graph/NodeGroup.cpp
    ${FREESTYLE_DIR}/scene_graph/NodeLight.cpp
    ${FREESTYLE_DIR}/scene_graph/NodeShape.cpp
    ${FREESTYLE_DIR}/scene_graph/NodeTransform.cpp
    ${FREESTYLE_DIR}/scene_graph/OrientedLineRep.cpp
    ${FREESTYLE_DIR}/scene_graph/PLYFileLoader.cpp
    ${FREESTYLE_DIR}/scene_graph/Rep.cpp
    ${FREESTYLE_DIR}/scene_graph/ScenePrettyPrinter.cpp
    ${FREESTYLE_DIR}/scene_graph/SceneVisitor.cpp
    ${FREESTYLE_DIR}/scene_graph/TriangleRep.cpp
    ${FREESTYLE_DIR}/scene_graph/VertexRep.cpp
)
target_include_directories(fs_scene_graph PUBLIC ${FREESTYLE_DIR})
target_link_libraries(fs_scene_graph PUBLIC fs_geometry)

# --- 4. system (base utilities, NO Python, NO Qt) ---
add_library(fs_system STATIC
    ${FREESTYLE_DIR}/system/BaseObject.cpp
    ${FREESTYLE_DIR}/system/Exception.cpp
    ${FREESTYLE_DIR}/system/PseudoNoise.cpp
    ${FREESTYLE_DIR}/system/RandGen.cpp
    ${FREESTYLE_DIR}/system/StringUtils.cpp
    ${FREESTYLE_DIR}/system/TimeStamp.cpp
)
target_include_directories(fs_system PUBLIC ${FREESTYLE_DIR})
# Note: PythonInterpreter.cpp excluded (links Python.h)
#       StringUtils uses QFileInfo → replaced with plain std::string path ops

# --- 5. winged_edge (feature-edge detection) ---
add_library(fs_winged_edge STATIC
    ${FREESTYLE_DIR}/winged_edge/Curvature.cpp
    ${FREESTYLE_DIR}/winged_edge/WEdge.cpp
    ${FREESTYLE_DIR}/winged_edge/WFillGrid.cpp
    ${FREESTYLE_DIR}/winged_edge/WXEdge.cpp
    ${FREESTYLE_DIR}/winged_edge/WXEdgeBuilder.cpp
    ${FREESTYLE_DIR}/winged_edge/WingedEdgeBuilder.cpp
)
target_include_directories(fs_winged_edge PUBLIC ${FREESTYLE_DIR})
target_link_libraries(fs_winged_edge PUBLIC fs_geometry fs_scene_graph)

# --- 6. view_map (ViewMap builder, feature extraction) ---
#     SteerableViewMap uses QImage → excluded
#     ViewMap debug drawing uses OpenGL → ifdef'd with FREESTYLE_NO_OPENGL
add_library(fs_view_map STATIC
    ${FREESTYLE_DIR}/view_map/FEdgeXDetector.cpp
    ${FREESTYLE_DIR}/view_map/Functions0D.cpp
    ${FREESTYLE_DIR}/view_map/Functions1D.cpp
    ${FREESTYLE_DIR}/view_map/Interface1D.cpp
    ${FREESTYLE_DIR}/view_map/PunchOut.cpp
    ${FREESTYLE_DIR}/view_map/Silhouette.cpp
    ${FREESTYLE_DIR}/view_map/SilhouetteGeomEngine.cpp
    ${FREESTYLE_DIR}/view_map/ViewEdgeXBuilder.cpp
    ${FREESTYLE_DIR}/view_map/ViewMap.cpp
    ${FREESTYLE_DIR}/view_map/ViewMapBuilder.cpp
    ${FREESTYLE_DIR}/view_map/ViewMapIO.cpp
    ${FREESTYLE_DIR}/view_map/ViewMapTesselator.cpp
    ${FREESTYLE_DIR}/view_map/grid2d.cpp
    ${FREESTYLE_DIR}/view_map/predicates.c
    ${FREESTYLE_DIR}/view_map/triangle_triangle_intersection.c
)
target_include_directories(fs_view_map PUBLIC ${FREESTYLE_DIR})
target_link_libraries(fs_view_map PUBLIC fs_scene_graph fs_winged_edge fs_image fs_system)
target_compile_definitions(fs_view_map PUBLIC FREESTYLE_NO_OPENGL FREESTYLE_NO_STEERABLE)

# --- 7. stroke (chaining, PS/EPS output) ---
add_library(fs_stroke STATIC
    ${FREESTYLE_DIR}/stroke/Chain.cpp
    ${FREESTYLE_DIR}/stroke/ChainingIterators.cpp
    ${FREESTYLE_DIR}/stroke/Operators.cpp
    ${FREESTYLE_DIR}/stroke/Stroke.cpp
    ${FREESTYLE_DIR}/stroke/StrokeRep.cpp
    ${FREESTYLE_DIR}/stroke/StrokeRenderer.cpp
    ${FREESTYLE_DIR}/stroke/StrokeTesselator.cpp
    ${FREESTYLE_DIR}/stroke/StrokeLayer.cpp
    ${FREESTYLE_DIR}/stroke/StrokeIO.cpp
    ${FREESTYLE_DIR}/stroke/Canvas.cpp
    ${FREESTYLE_DIR}/stroke/Curve.cpp
    ${FREESTYLE_DIR}/stroke/ContextFunctions.cpp
    ${FREESTYLE_DIR}/stroke/BasicStrokeShaders.cpp
    ${FREESTYLE_DIR}/stroke/PSStrokeRenderer.cpp
    ${FREESTYLE_DIR}/stroke/TextStrokeRenderer.cpp
)
target_include_directories(fs_stroke PUBLIC ${FREESTYLE_DIR})
target_link_libraries(fs_stroke PUBLIC fs_geometry fs_view_map)
target_compile_definitions(fs_stroke PUBLIC FREESTYLE_NO_OPENGL FREESTYLE_NO_QT)

# ===================================================================
# Standalone CLI test driver
# ===================================================================
add_executable(contours_cli contours_cli.cpp)
target_include_directories(contours_cli PRIVATE tess_RifFilter)
target_link_libraries(contours_cli PRIVATE tess_RifFilter osdutil)

# ---- Freestyle NPR renderer (PLY → EPS) ----
add_executable(freestyle_render freestyle_render.cpp)
target_link_libraries(freestyle_render PRIVATE fs_stroke)

# ---- Monte Carlo engraving renderer (PLY → SVG) ----
add_executable(engraving_render engraving_render.cpp)
target_link_libraries(engraving_render PRIVATE fs_winged_edge fs_view_map fs_scene_graph fs_geometry)

message(STATUS "Contours build configured")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenSubdiv: hbr, osdutil, tess_RifFilter")
message(STATUS "  Freestyle:  fs_geometry fs_image fs_scene_graph fs_system")
message(STATUS "              fs_winged_edge fs_view_map fs_stroke")
message(STATUS "  CLI: contours_cli, freestyle_render, engraving_render")
