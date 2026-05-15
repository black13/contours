cmake_minimum_required(VERSION 3.16)
project(Contours VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- OpenSubdiv: hbr (header-only) ----
add_library(hbr INTERFACE)
target_include_directories(hbr INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/tess_RifFilter/opensubdiv
)

# ---- OpenSubdiv: osdutil (static lib, minimal) ----
set(OSDUTIL_DIR tess_RifFilter/opensubdiv/osdutil)
set(OSDUTIL_SOURCES
    ${OSDUTIL_DIR}/adaptiveEvaluator.cpp
    ${OSDUTIL_DIR}/topology.cpp
    ${OSDUTIL_DIR}/refiner.cpp
    ${OSDUTIL_DIR}/mesh.cpp
)
add_library(osdutil STATIC ${OSDUTIL_SOURCES})
target_include_directories(osdutil PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/tess_RifFilter/opensubdiv
)
target_link_libraries(osdutil PUBLIC hbr)

# ---- tess_RifFilter core (no RenderMan) ----
set(TESS_SOURCES
    tess_RifFilter/refineContour.cpp
    tess_RifFilter/subdiv.cpp
    tess_RifFilter/vdtess.cpp
    tess_RifFilter/cameraModel.cpp
)
set(TESS_HEADERS
    tess_RifFilter/refineContour.h
    tess_RifFilter/refineContourFunctions.h
    tess_RifFilter/subdiv.h
    tess_RifFilter/cameraModel.h
    tess_RifFilter/VecMat.h
    tess_RifFilter/chart.h
    tess_RifFilter/chartFunctions.h
    tess_RifFilter/meshFunctions.h
    tess_RifFilter/paramPoint.h
    tess_RifFilter/paramPointFunctions.h
)
add_library(tess_RifFilter STATIC ${TESS_SOURCES} ${TESS_HEADERS})
target_include_directories(tess_RifFilter PUBLIC tess_RifFilter)
target_link_libraries(tess_RifFilter PUBLIC osdutil)

message(STATUS "Contours build configured")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Targets: hbr (header-only), osdutil, tess_RifFilter")
