cmake_minimum_required(VERSION 3.10)
project(urdf_viewer)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(OGRE REQUIRED)
find_package(assimp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdf_model REQUIRED)

add_executable(urdf_viewer
    main.cpp
    ogre_viewer.cpp
    urdf_loader.cpp
)

target_include_directories(urdf_viewer PRIVATE 
${OGRE_INCLUDE_DIRS} 
${ASSIMP_INCLUDE_DIRS} 
${urdfdom_INCLUDE_DIRS}
${urdf_model_INCLUDE_DIRS}
)
target_link_libraries(urdf_viewer ${OGRE_LIBRARIES})
