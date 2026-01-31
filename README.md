# URDF Renderer Plugin

[中文](README.md) | [English](README.en.md)

一个用于渲染 URDF 机器人模型的轻量级共享库插件，支持无头渲染（Headless Rendering）和正向运动学计算。

## 功能特性

- **URDF 模型加载**: 通过 `urdfdom` 解析标准 URDF 文件。
- **网格渲染**: 使用 `assimp` 加载 STL 模型文件。
- **材质与纹理**: 支持 URDF 中定义的材质颜色和纹理贴图。
- **无头渲染**: 支持离屏渲染，输出 RGBA 格式图像（支持透明背景）。
- **正向运动学**: 通过设置关节角度实时驱动连杆位置。

## 依赖项

- **OGRE** (1.12.10+)
- **urdfdom**
- **assimp**
- **pthreads**
- **OpenCV** (可选，用于支持 `cv::Mat` 直接输出)

## 构建与安装

### 编译

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 安装

默认安装至系统目录（如 `/usr/local`）：

```bash
sudo make install
```

## 使用说明

### C++ API 示例

```cpp
#include "urdf_renderer_plugin.hpp"

// 初始化
auto plugin = std::make_unique<URDFRendererPlugin>();
UrdfRenderConfig config;
config.width = 800;
config.height = 600;
config.use_transparent_background = true;

plugin->initialize(&config);

// 加载模型
plugin->loadURDF("robot.urdf");

// 设置关节角度
plugin->setJointAngle("joint1", 0.5);

// 渲染帧
plugin->renderFrame();

// 获取原始数据
const auto* buffer = plugin->getImageBuffer();
// buffer->data 包含 RGBA 像素数据
```

### C API 示例

本项目提供了 C 语言接口，便于与其他语言（如 Python/Rust）进行绑定。

```c
#include "urdf_renderer_plugin.h"

// 创建插件实例
UrdfRenderConfig config = {800, 600, true, {0,0,0,0}, 0};
UrdfPluginHandle plugin = urdf_plugin_create(&config);

// 加载 URDF
urdf_plugin_load_file(plugin, "robot.urdf");

// 设置关节角度 (弧度)
urdf_plugin_set_joint_angle(plugin, "joint1", 0.5);

// 渲染
urdf_plugin_render_frame(plugin);

// 获取图像缓冲区
UrdfImageData img;
urdf_plugin_get_image_buffer(plugin, &img);

// 销毁实例
urdf_plugin_destroy(plugin);
```

## OpenCV 集成

如果在构建环境中检测到 OpenCV，插件将自动启用 OpenCV 支持。

```cpp
#ifdef HAVE_OPENCV
// 获取 cv::Mat 对象 (格式: RGBA, 深拷贝)
cv::Mat rgba_image = plugin->getImageAsMat();

// 转换为 OpenCV 标准 BGR 格式
cv::Mat bgr_image;
cv::cvtColor(rgba_image, bgr_image, cv::COLOR_RGBA2BGR);
#endif
```

## 附带工具

构建完成后会生成以下文件：

- `liburdf_renderer_plugin.so` - 核心动态库
- `video_generator` - 批量生成动画帧的工具
- `example_c` - C 接口调用示例
- `example_opencv` - OpenCV 集成示例


