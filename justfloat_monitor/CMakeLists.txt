cmake_minimum_required(VERSION 3.20)

project(JustFloatMonitor LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Network Widgets)

add_executable(JustFloatMonitor WIN32
    src/main.cpp
    src/MainWindow.h
    src/MainWindow.cpp
    src/CameraView.h
    src/CameraView.cpp
    src/TelemetryProtocol.h
    src/TelemetryProtocol.cpp
)

target_link_libraries(JustFloatMonitor PRIVATE
    Qt6::Network
    Qt6::Widgets
)

if (BUILD_TESTING)
    find_package(Qt6 REQUIRED COMPONENTS Test)

    add_executable(JustFloatMonitorTests
        tests/TelemetryProtocolTests.cpp
        src/TelemetryProtocol.h
        src/TelemetryProtocol.cpp
    )

    target_include_directories(JustFloatMonitorTests PRIVATE src)
    target_link_libraries(JustFloatMonitorTests PRIVATE Qt6::Core Qt6::Test)
    add_test(NAME JustFloatMonitorTests COMMAND JustFloatMonitorTests)
endif()
