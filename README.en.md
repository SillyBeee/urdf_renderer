# URDF Renderer Plugin

[中文](README.md) | [English](README.en.md)

A lightweight shared library plugin for rendering URDF robot models, supporting headless rendering and forward kinematics.

## Features

- **URDF Model Loading**: Supports standard URDF files parsing via `urdfdom`.
- **Mesh Rendering**: Loads STL meshes using `assimp`.
- **Material & Texture**: Supports material colors and textures defined in URDF.
- **Headless Rendering**: Off-screen rendering support with RGBA output (supports transparent background).
- **Forward Kinematics**: Drives link positions via joint angles/states.

## Dependencies

- **OGRE** (1.12.10+)
- **urdfdom**
- **assimp**
- **pthreads**
- **OpenCV** (Optional, enables direct `cv::Mat` output support)

## Build & Install

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Usage

### C++ API Example

```cpp
#include "urdf_renderer_plugin.hpp"

// Initialize
auto plugin = std::make_unique<URDFRendererPlugin>();
UrdfRenderConfig config;
config.width = 800;
config.height = 600;
config.use_transparent_background = true;

plugin->initialize(&config);

// Load Model
plugin->loadURDF("robot.urdf");

// Set Joint Angles
plugin->setJointAngle("joint1", 0.5);

// Render Frame
plugin->renderFrame();

// Access Raw Buffer
const auto* buffer = plugin->getImageBuffer();
// buffer->data contains raw RGBA pixels
```

### C API Example

```c
#include "urdf_renderer_plugin.h"

// Initialize
UrdfRenderConfig config = {800, 600, true, {0,0,0,0}, 0};
UrdfPluginHandle plugin = urdf_plugin_create(&config);

// Load Model
urdf_plugin_load_file(plugin, "robot.urdf");

// Set Joint Angle (radians)
urdf_plugin_set_joint_angle(plugin, "joint1", 0.5);

// Render
urdf_plugin_render_frame(plugin);

// Get Image Data
UrdfImageData img;
urdf_plugin_get_image_buffer(plugin, &img);

// Cleanup
urdf_plugin_destroy(plugin);
```

## OpenCV Integration

If OpenCV is found during the build process, the plugin provides additional methods to return `cv::Mat` objects directly.

```cpp
#ifdef HAVE_OPENCV
// Get deep copy as cv::Mat (Format: RGBA)
cv::Mat rgba_image = plugin->getImageAsMat();

// Convert to BGR for standard OpenCV usage
cv::Mat bgr_image;
cv::cvtColor(rgba_image, bgr_image, cv::COLOR_RGBA2BGR);
#endif
```

## Tools

The build process produces the following executables:

- `liburdf_renderer_plugin.so`: The shared library.
- `video_generator`: Tool to generate animation frames from URDF.
- `example_c`: Example usage of the C API.
- `example_opencv`: Example usage with OpenCV integration.
