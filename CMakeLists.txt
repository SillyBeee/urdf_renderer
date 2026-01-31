cmake_minimum_required(VERSION 3.10)
project(urdf_renderer)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OGRE REQUIRED)
find_package(assimp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Threads REQUIRED)

# OpenCV is optional - enables cv::Mat API
find_package(OpenCV QUIET COMPONENTS core)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}, enabling cv::Mat API")
    set(HAVE_OPENCV TRUE)
else()
    message(STATUS "OpenCV not found, cv::Mat API will be disabled")
    set(HAVE_OPENCV FALSE)
endif()

# urdfdom includes urdf_parser and urdf_model
set(urdf_model_LIBRARIES ${urdfdom_LIBRARIES})
set(urdf_model_INCLUDE_DIRS ${urdfdom_INCLUDE_DIRS})

# Build shared library plugin
add_library(urdf_renderer_plugin SHARED
    urdf_loader.cpp
    urdf_renderer_plugin.cpp
    urdf_renderer_plugin_c.cpp
)

target_compile_definitions(urdf_renderer_plugin PRIVATE URDF_PLUGIN_EXPORTS)

# Enable OpenCV support if available
if(HAVE_OPENCV)
    target_compile_definitions(urdf_renderer_plugin PRIVATE HAVE_OPENCV)
    target_include_directories(urdf_renderer_plugin PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(urdf_renderer_plugin PRIVATE opencv_core)
endif()

target_include_directories(urdf_renderer_plugin PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OGRE_INCLUDE_DIRS} 
    ${ASSIMP_INCLUDE_DIRS} 
    ${urdfdom_INCLUDE_DIRS}
    ${urdf_model_INCLUDE_DIRS}
)

target_link_libraries(urdf_renderer_plugin PUBLIC
    ${OGRE_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${urdf_model_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    Threads::Threads
)

set_target_properties(urdf_renderer_plugin PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
    PUBLIC_HEADER "urdf_renderer_plugin.h"
)


# Install targets
install(TARGETS urdf_renderer_plugin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

# Build C example
add_executable(example_c examples/example_c.c)
target_link_libraries(example_c urdf_renderer_plugin)
target_include_directories(example_c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Build video generator
add_executable(video_generator examples/video_generator.c)
target_link_libraries(video_generator urdf_renderer_plugin m)
target_include_directories(video_generator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Build OpenCV example (if OpenCV is available)
if(HAVE_OPENCV)
    add_executable(example_opencv examples/example_opencv.cpp)
    target_compile_definitions(example_opencv PRIVATE HAVE_OPENCV)
    target_link_libraries(example_opencv urdf_renderer_plugin opencv_core opencv_imgproc opencv_imgcodecs)
    target_include_directories(example_opencv PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
    message(STATUS "Building example_opencv with OpenCV support")
endif()
