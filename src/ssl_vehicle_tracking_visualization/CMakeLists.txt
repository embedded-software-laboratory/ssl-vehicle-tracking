cmake_minimum_required(VERSION 3.5)

project(ssl_vehicle_tracking_visualization)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 14)

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ssl_vehicle_tracking_msgs REQUIRED)

set(rviz_default_plugins_headers_to_moc
    include/ssl_vehicle_tracking_visualization/displays/object/ObjectDisplay.hpp
    include/ssl_vehicle_tracking_visualization/displays/wheel/WheelDisplay.hpp
    include/ssl_vehicle_tracking_visualization/displays/track/TrackDisplay.hpp
    )

foreach (header "${rviz_default_plugins_headers_to_moc}")
  qt5_wrap_cpp(rviz_default_plugins_moc_files "${header}")
endforeach ()

set(rviz_default_plugins_source_files
        src/AngleProperty.cpp
        src/OnlineProperty.cpp
        src/LineObject.cpp
        src/Visual.cpp
        src/Shape.cpp
        src/displays/object/ObjectDisplay.cpp
        src/displays/object/ObjectVisual.cpp
        src/displays/object/ObjectSelectionHandler.cpp
        src/displays/wheel/WheelDisplay.cpp
        src/displays/wheel/WheelVisual.cpp
        src/displays/wheel/WheelSelectionHandler.cpp
        src/displays/track/TrackDisplay.cpp
        src/displays/track/TrackVisual.cpp
        src/displays/track/TrackSelectionHandler.cpp
    )

add_library(ssl_vehicle_tracking_visualization SHARED
    ${rviz_default_plugins_moc_files}
    ${rviz_default_plugins_source_files}
    )

target_include_directories(ssl_vehicle_tracking_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${Qt5Widgets_INCLUDE_DIRS}
    )

target_link_libraries(ssl_vehicle_tracking_visualization PUBLIC
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
    )
    
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ssl_vehicle_tracking_visualization PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(ssl_vehicle_tracking_visualization
    PUBLIC
    rclcpp
    rviz_common
    rviz_rendering
    tf2
    tf2_geometry_msgs
    tf2_ros
    ssl_vehicle_tracking_msgs
    )

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(ssl_vehicle_tracking_visualization HAS_LIBRARY_TARGET)

ament_export_dependencies(
    rclcpp
    rviz_common
    rviz_ogre_vendor
    tf2
    tf2_geometry_msgs
    tf2_ros
    ssl_vehicle_tracking_msgs
)

install(
    TARGETS ssl_vehicle_tracking_visualization
    EXPORT ssl_vehicle_tracking_visualization
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
    DESTINATION "share/${PROJECT_NAME}"
)

install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ogre_media"
    DESTINATION "share/${PROJECT_NAME}"
)

register_rviz_ogre_media_exports(DIRECTORIES "ogre_media")

ament_package()


