cmake_minimum_required(VERSION 3.20)

project(BeaconImageAnalyzer LANGUAGES C CXX)

include(CTest)

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
    app/LogWaveformWindow.h
    app/LogWaveformWindow.cpp
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
    core/JustFloatCsvRecorder.h
    core/JustFloatCsvRecorder.cpp
    core/WaveformHistoryStore.h
    core/WaveformHistoryStore.cpp
    core/WaveformViewport.h
    core/WaveformViewport.cpp
    core/FrameRenderer.h
    core/FrameRenderer.cpp
    core/BimgImageFrameParser.h
    core/BimgImageFrameParser.cpp
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

if (BUILD_TESTING)
    find_package(Qt6 REQUIRED COMPONENTS Test)

    add_executable(BeaconImageAnalyzerTests
        tests/JustFloatLogTests.cpp
        core/JustFloatLog.h
        core/JustFloatLog.cpp
        core/JustFloatCsvRecorder.h
        core/JustFloatCsvRecorder.cpp
        core/WaveformHistoryStore.h
        core/WaveformHistoryStore.cpp
        core/WaveformViewport.h
        core/WaveformViewport.cpp
    )

    target_include_directories(BeaconImageAnalyzerTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
    )

    target_link_libraries(BeaconImageAnalyzerTests PRIVATE
        Qt6::Core
        Qt6::Test
    )

    add_test(NAME BeaconImageAnalyzerTests COMMAND BeaconImageAnalyzerTests)

    add_executable(BimgImageFrameParserTests
        tests/BimgImageFrameParserTests.cpp
        core/BimgImageFrameParser.h
        core/BimgImageFrameParser.cpp
    )

    target_include_directories(BimgImageFrameParserTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
    )

    target_link_libraries(BimgImageFrameParserTests PRIVATE
        Qt6::Gui
        Qt6::Test
    )

    add_test(NAME BimgImageFrameParserTests COMMAND BimgImageFrameParserTests)
endif()
