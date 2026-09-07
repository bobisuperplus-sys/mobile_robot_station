cmake_minimum_required(VERSION 3.16)

project(mobile_console VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_PREFIX_PATH AND EXISTS "/home/yellowtown/Qt/6.11.2/gcc_64")
    list(APPEND CMAKE_PREFIX_PATH "/home/yellowtown/Qt/6.11.2/gcc_64")
endif()

find_package(Qt6 REQUIRED COMPONENTS Quick Network)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

qt_standard_project_setup(REQUIRES 6.10)

# Content of downloaded design from Figma to Qt or similar
if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/importedcontent/CMakeLists.txt")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/importedcontent)
endif()

qt_add_executable(appmobile_console
    main.cpp
    gst_video_receiver.h
    gst_video_receiver.cpp
    msh_client.h
    msh_client.cpp
)

set_source_files_properties(Theme.qml
    PROPERTIES
        QT_QML_SINGLETON_TYPE TRUE
)

qt_add_qml_module(appmobile_console
    URI mobile_console
    VERSION 1.0
    QML_FILES
        Main.qml
        Theme.qml
        components/CardPanel.qml
        components/StatusBadge.qml
        components/ActionButton.qml
        components/SliderBar.qml
        components/SplitHandleBar.qml
        views/TopHeader.qml
        views/CameraPanel.qml
        views/SpatialDemView.qml
        views/TelemetryPanel.qml
        views/WaypointPanel.qml
        views/MotionPanel.qml
        views/ConsoleLogFooter.qml
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appmobile_console PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appmobile_console
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appmobile_console
    PRIVATE Qt6::Quick Qt6::Network PkgConfig::GST
)

install(TARGETS appmobile_console
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
