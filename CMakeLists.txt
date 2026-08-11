cmake_minimum_required(VERSION 3.20)

project(BeaconImageAnalyzer LANGUAGES C CXX)

include(CTest)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Concurrent Network Widgets)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

add_library(beacon_algorithm STATIC
    algorithm/beacon_image.c
)

target_include_directories(beacon_algorithm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
)

set(TWO_BL3_IMAGE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../HDUASC-SmartCar-21st-FlyOverMinefield/CYT2BL3_Image/project/code/Image"
    CACHE PATH "2BL3 image algorithm source directory")
if (EXISTS "${TWO_BL3_IMAGE_DIR}/image.c" AND EXISTS "${TWO_BL3_IMAGE_DIR}/image_params.c")
    add_library(two_bl3_diagnostic SHARED
        "${TWO_BL3_IMAGE_DIR}/image.c"
        "${TWO_BL3_IMAGE_DIR}/image_params.c"
        "${TWO_BL3_IMAGE_DIR}/image_horizon.c"
        algorithm/2bl3_host/two_bl3_desktop_adapter.c
    )
    target_include_directories(two_bl3_diagnostic PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/2bl3_host
        "${TWO_BL3_IMAGE_DIR}/.."
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    )
    target_link_libraries(two_bl3_diagnostic PRIVATE m)
    set_target_properties(two_bl3_diagnostic PROPERTIES
        PREFIX ""
        OUTPUT_NAME "two_bl3_diagnostic"
    )
endif()

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
    app/ImageLogReviewWindow.h
    app/ImageLogReviewWindow.cpp
    app/LogWaveformWindow.h
    app/LogWaveformWindow.cpp
    app/HorizonCalibrationWindow.h
    app/HorizonCalibrationWindow.cpp
    app/BeaconLabelWindow.h
    app/BeaconLabelWindow.cpp
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
    core/BeaconParameterDiagnostic.h
    core/BeaconParameterDiagnostic.cpp
    core/TwoBl3ParameterCatalog.h
    core/TwoBl3ParameterCatalog.cpp
    core/CarPlanRunner.h
    core/CarPlanRunner.cpp
    core/BeaconResultUtils.h
    core/JustFloatLog.h
    core/JustFloatLog.cpp
    core/JustFloatCsvRecorder.h
    core/JustFloatCsvRecorder.cpp
    core/WaveformHistoryStore.h
    core/WaveformHistoryStore.cpp
    core/VehicleMotionUtils.h
    core/WaveformViewport.h
    core/WaveformViewport.cpp
    core/FrameRenderer.h
    core/FrameRenderer.cpp
    core/BimgImageFrameParser.h
    core/BimgImageFrameParser.cpp
    core/ImageFrameSidecar.h
    core/ImageFrameSidecar.cpp
    core/ImageLogAligner.h
    core/ImageLogAligner.cpp
    core/SingleLampLogDiagnostics.h
    core/SingleLampLogDiagnostics.cpp
    core/HorizonLineGeometry.h
    core/HorizonLineGeometry.cpp
    core/HorizonCalibration.h
    core/HorizonCalibration.cpp
    core/HorizonCalibrationRecorder.h
    core/HorizonCalibrationRecorder.cpp
    core/DownGroundRangeCalibration.h
    core/DownGroundRangeCalibration.cpp
    core/BeaconLabelSession.h
    core/BeaconLabelSession.cpp
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
    Qt6::Concurrent
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
        core/VehicleMotionUtils.h
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

    add_executable(ImageFrameSidecarTests
        tests/ImageFrameSidecarTests.cpp
        core/ImageFrameSidecar.h
        core/ImageFrameSidecar.cpp
    )

    target_include_directories(ImageFrameSidecarTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
    )

    target_link_libraries(ImageFrameSidecarTests PRIVATE
        Qt6::Core
        Qt6::Test
    )

    add_test(NAME ImageFrameSidecarTests COMMAND ImageFrameSidecarTests)

    add_executable(ImageLogAlignerTests
        tests/ImageLogAlignerTests.cpp
        core/ImageLogAligner.h
        core/ImageLogAligner.cpp
        core/ImageFrameSidecar.h
        core/JustFloatLog.h
        core/JustFloatLog.cpp
    )

    target_include_directories(ImageLogAlignerTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
    )

    target_link_libraries(ImageLogAlignerTests PRIVATE
        Qt6::Core
        Qt6::Test
    )

    add_test(NAME ImageLogAlignerTests COMMAND ImageLogAlignerTests)

    add_executable(HorizonCalibrationTests
        tests/HorizonCalibrationTests.cpp
        core/HorizonLineGeometry.h
        core/HorizonLineGeometry.cpp
        core/HorizonCalibration.h
        core/HorizonCalibration.cpp
        core/HorizonCalibrationRecorder.h
        core/HorizonCalibrationRecorder.cpp
        core/ImageFrameSidecar.h
        core/ImageFrameSidecar.cpp
        core/DownGroundRangeCalibration.h
        core/DownGroundRangeCalibration.cpp
        core/VideoReader.h
        core/VideoReader.cpp
    )

    target_include_directories(HorizonCalibrationTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(HorizonCalibrationTests PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Test
        ${OpenCV_LIBS}
    )

    add_test(NAME HorizonCalibrationTests COMMAND HorizonCalibrationTests)

    add_executable(BeaconLabelSessionTests
        tests/BeaconLabelSessionTests.cpp
        core/BeaconLabelSession.h
        core/BeaconLabelSession.cpp
    )

    target_include_directories(BeaconLabelSessionTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
    )

    target_link_libraries(BeaconLabelSessionTests PRIVATE
        Qt6::Core
        Qt6::Test
    )

    add_test(NAME BeaconLabelSessionTests COMMAND BeaconLabelSessionTests)

    add_executable(TwoBl3AlgorithmRunnerTests
        tests/TwoBl3AlgorithmRunnerTests.cpp
        core/AlgorithmRunner.h
        core/AlgorithmRunner.cpp
        core/BeaconParameterDiagnostic.h
        core/BeaconParameterDiagnostic.cpp
        core/TwoBl3ParameterCatalog.h
        core/TwoBl3ParameterCatalog.cpp
        core/FrameRenderer.h
        core/FrameRenderer.cpp
        annotation/AnnotationModel.h
        annotation/AnnotationModel.cpp
    )

    target_include_directories(TwoBl3AlgorithmRunnerTests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/annotation
    )

    target_compile_definitions(TwoBl3AlgorithmRunnerTests PRIVATE
        BEACON_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
        BEACON_2BL3_TEST_IMAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../HDUASC-SmartCar-21st-FlyOverMinefield/CYT2BL3_Image/project/code/Image"
    )

    target_link_libraries(TwoBl3AlgorithmRunnerTests PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Test
        beacon_algorithm
    )

    add_test(NAME TwoBl3AlgorithmRunnerTests COMMAND TwoBl3AlgorithmRunnerTests)

    set(DOWN_DRAFT_IMAGE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/instances_down/0801_下摄原始灰度信标草稿/algorithm/Image")
    set(DOWN_DRAFT_HOST_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/instances_down/0801_下摄原始灰度信标草稿/algorithm/host")
    if (EXISTS "${DOWN_DRAFT_IMAGE_DIR}/image_down_horizon.c")
        configure_file(
            "${DOWN_DRAFT_IMAGE_DIR}/image_down_horizon.c"
            "${CMAKE_CURRENT_BINARY_DIR}/down_horizon_implicit_source.c"
            COPYONLY
        )
        add_executable(DownHorizonImplicitTests
            tests/DownHorizonImplicitTests.cpp
            "${CMAKE_CURRENT_BINARY_DIR}/down_horizon_implicit_source.c"
        )

        target_include_directories(DownHorizonImplicitTests PRIVATE
            "${DOWN_DRAFT_IMAGE_DIR}"
            "${DOWN_DRAFT_HOST_DIR}"
        )

        target_compile_options(DownHorizonImplicitTests PRIVATE
            -Wall
            -Wextra
            -Werror
        )

        target_compile_definitions(DownHorizonImplicitTests PRIVATE
            IMAGE_DOWN_HORIZON_TEST_HOOKS=1
        )

        target_link_libraries(DownHorizonImplicitTests PRIVATE
            Qt6::Core
            Qt6::Test
            m
        )

        add_test(NAME DownHorizonImplicitTests COMMAND DownHorizonImplicitTests)
    endif()
endif()
