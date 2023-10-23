cmake_minimum_required(VERSION 3.5)
project(JLURoboVision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Platform specific Compiler Feature selection
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # -Werror=return-type: Must return a value in non-void functions, otherwise error out.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED \
                         -Werror=return-type \
                        ")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    message("MSVC DETECTED. FORCE UTF-8 USAGE.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8")
else()
    message("Unknown Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
endif()

if(UNIX)
    find_package(OpenCV)
    find_package(OpenCV REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(Sophus REQUIRED)
    find_package(yaml-cpp REQUIRED)

    set(ADDITIONAL_LIBS
        -lpthread
        -lm
        -lgxiapi
        -lyaml-cpp
    )
elseif(WIN32)
    include("windows_libs.cmake")
    include(${WIN_OPENCV_CONFIG})#这些是在Windows的环境变量里面定义的
    include(${WIN_EIGEN3_CONFIG})
    include(${EIGEN3_USE_FILE})
    include(${WIN_CERES_CONFIG})
    link_directories(${WIN_GALAXY_LIB_DIR})
    

    set(ADDITIONAL_LIBS
        GxIAPI.lib
        DxImageProc.lib
    )
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(.)
include_directories(AngleSolver)
include_directories(Armor)
include_directories(Can)
include_directories(Debug)
include_directories(General)
include_directories(GxCamera)
include_directories(GxCamera/GxSDK)
include_directories(Main)
include_directories(Serial)
include_directories(Wind)

# Debug infrastructure
add_subdirectory(Debug)
include_directories(Debug/imgui)

# Misc platform specific source files
if(UNIX)
    set(MISC_PLATFORM_SOURCE
        Serial/Serial.cpp)
    include_directories(${SDL2_INCLUDE_DIRS})
elseif(WIN32)
    set(MISC_PLATFORM_SOURCE
        Serial/Serial_WinStub.cpp
            Armor/predictor/kalman_filter.cpp)
endif()

add_executable(JLURoboVision
        ${MISC_PLATFORM_SOURCE}
        AngleSolver/AngleSolver.cpp
        AngleSolver/AngleSolver.h
        Armor/Armor.h
        Armor/detector/ArmorBox.cpp
        Armor/detector/ArmorDetector.cpp
        Armor/detector/ArmorNumClassifier.cpp
        Armor/detector/findLights.cpp
        Armor/detector/LightBar.cpp
        Armor/detector/matchArmors.cpp
        General/General.h
        General/General.cpp
        GxCamera/GxSDK/DxImageProc.h
        GxCamera/GxSDK/GxIAPI.h
        GxCamera/GxCamera.cpp
        GxCamera/GxCamera.h
        Main/ArmorDetecting.cpp
        Main/ImageUpdating.cpp
        Main/main.cpp
        Serial/Serial.h
        Armor/predictor/AdaptiveEKF.h
        General/Platform.h
        Thirdparty/angles.h
        Armor/predictor/TrackState.cpp
        Armor/detector/ArmorDebug.cpp
        Armor/predictor/PredictPitch.cpp
        Armor/predictor/PredictPitch.h
        Armor/predictor/kalman_filter.cpp
        Armor/predictor/kalman_filter.hpp
        Armor/predictor/AntiScopperil.cpp
        Armor/predictor/AntiScopperil.h
        )


target_link_libraries(${PROJECT_NAME}#链接库
        ${OpenCV_LIBS}
        ${Eigen3_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Sophus_LIBRARIES}
        ImguiDbgkit
        ${ADDITIONAL_LIBS}
)
get_target_property(PRJ_INCLUDES JLURoboVision INCLUDE_DIRECTORIES)
message("Detected includes: ${PRJ_INCLUDES}")
