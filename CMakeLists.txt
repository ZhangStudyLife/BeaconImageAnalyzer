cmake_minimum_required(VERSION 3.20)

project(BeaconImageAnalyzer LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Network Widgets)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

add_library(beacon_algorithm STATIC
    algorithm/beacon_image.c
)

target_include_directories(beacon_algorithm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
)

set(BEACON_APP_PLATFORM_SOURCES)
if (WIN32)
    list(APPEND BEACON_APP_PLATFORM_SOURCES
        app/app.rc
    )
endif()

add_executable(BeaconImageAnalyzer
    resources.qrc
    app/main.cpp
    app/MainWindow.h
    app/MainWindow.cpp
    app/AiInstanceDialog.h
    app/AiInstanceDialog.cpp
    app/LogReplayWindow.h
    app/LogReplayWindow.cpp
    app/TcpImageWindow.h
    app/TcpImageWindow.cpp
    app/VideoWidget.h
    app/VideoWidget.cpp
    app/AnnotationPanel.h
    app/AnnotationPanel.cpp
    core/VideoReader.h
    core/VideoReader.cpp
    core/AlgorithmRunner.h
    core/AlgorithmRunner.cpp
    core/CarPlanRunner.h
    core/CarPlanRunner.cpp
    core/BeaconResultUtils.h
    core/JustFloatLog.h
    core/JustFloatLog.cpp
    core/FrameRenderer.h
    core/FrameRenderer.cpp
    core/SeekfreeImageFrameParser.h
    core/SeekfreeImageFrameParser.cpp
    core/VideoExporter.h
    core/VideoExporter.cpp
    annotation/AnnotationModel.h
    annotation/AnnotationModel.cpp
    annotation/AnnotationJson.h
    annotation/AnnotationJson.cpp
    ${BEACON_APP_PLATFORM_SOURCES}
)

target_include_directories(BeaconImageAnalyzer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/annotation
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(BeaconImageAnalyzer PRIVATE
    Qt6::Network
    Qt6::Widgets
    beacon_algorithm
    ${OpenCV_LIBS}
)

target_compile_definitions(BeaconImageAnalyzer PRIVATE
    BEACON_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

if (WIN32)
    target_compile_definitions(BeaconImageAnalyzer PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
endif()
