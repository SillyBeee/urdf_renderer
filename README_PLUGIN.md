# URDF Viewer Plugin - 使用说明

## ✅ 已实现功能

### 核心功能
- ✅ **URDF模型加载** - 支持urdfdom解析
- ✅ **STL网格渲染** - 通过assimp加载STL文件
- ✅ **材质颜色** - 读取URDF中的材质定义并应用
- ✅ **纹理支持** - 支持加载纹理（如果URDF中有定义）
- ✅ **透明背景** - RGBA格式，alpha=0透明背景
- ✅ **正向运动学** - 关节角度正确驱动连杆位置
- ✅ **实时渲染** - 单帧渲染和批量动画生成

## 📦 构建

```bash
cd /home/ma/桌面/urdf_viewer
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

生成的文件：
- `liburdf_viewer_plugin.so` - 共享库插件
- `urdf_viewer` - 原始独立可执行文件
- `example_c` - C语言示例程序
- `video_generator` - 视频/动画生成工具
- `example_opencv` - OpenCV集成示例（需要OpenCV）

## 🔗 OpenCV集成 (可选)

插件提供了OpenCV集成支持，允许直接获取`cv::Mat`格式的渲染图像。

### 编译要求
- OpenCV >= 3.0 (仅需core模块)
- CMake会自动检测OpenCV，如未安装则跳过此功能

### C++ API
```cpp
#include "urdf_viewer_plugin.hpp"

auto plugin = std::make_unique<URDFViewerPlugin>();
plugin->initialize(&config);
plugin->loadURDF("robot.urdf");
plugin->renderFrame();

// 获取cv::Mat（RGBA格式，深拷贝）
cv::Mat rgba_image = plugin->getImageAsMat();

// 转换为BGR（OpenCV标准格式）
cv::Mat bgr_image;
cv::cvtColor(rgba_image, bgr_image, cv::COLOR_RGBA2BGR);
cv::imwrite("output.png", bgr_image);

// 进行图像处理
cv::GaussianBlur(bgr_image, bgr_image, cv::Size(15, 15), 0);
cv::Canny(bgr_image, edges, 50, 150);
```

### 运行示例
```bash
./example_opencv arm_description/urdf/miniarm.urdf
```

生成文件：
- `opencv_rgba_output.png` - RGBA原始格式
- `opencv_bgr_output.png` - BGR格式
- `opencv_rgb_output.jpg` - RGB JPEG格式
- `opencv_frame_*.png` - 30帧动画序列（带帧号标记）
- `opencv_edges.png` - Canny边缘检测
- `opencv_blurred.png` - 高斯模糊效果

### 性能说明
- `getImageAsMat()`执行深拷贝，确保内存安全
- 性能开销：800×600约1-2ms，1920×1080约3-5ms
- 如需零拷贝，使用`getImageBuffer()`直接访问原始缓冲区

### 颜色格式
⚠️ **注意**：`getImageAsMat()`返回**RGBA**格式，与OpenCV默认**BGR**不同！
- 如需BGR：`cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);`
- 如需RGB：`cv::cvtColor(rgba, rgb, cv::COLOR_RGBA2RGB);`
- 去除alpha：`cv::cvtColor(rgba, rgb, cv::COLOR_RGBA2RGB);`

## 🎬 视频动画生成

### 生成帧序列
```bash
./video_generator <urdf_file> <num_frames>

# 示例：生成4秒动画（120帧 @ 30fps）
./video_generator ../arm_description/urdf/miniarm.urdf 120
```

### 转换为GIF动画
```bash
# 使用ImageMagick
convert -delay 3 -loop 0 frame_*.ppm robot_animation.gif
```

### 转换为MP4视频（需要ffmpeg）
```bash
ffmpeg -framerate 30 -i frame_%04d.ppm -c:v libx264 -pix_fmt yuv420p robot_animation.mp4
```

## 📝 C API 使用示例

### 基本渲染
```c
#include "urdf_viewer_plugin.h"

// 创建插件（800x600，透明背景）
UrdfRenderConfig config = {800, 600, true, {0,0,0,0}, 0};
UrdfPluginHandle plugin = urdf_plugin_create(&config);

// 加载URDF
urdf_plugin_load_file(plugin, "robot.urdf");

// 设置关节角度
urdf_plugin_set_joint_angle(plugin, "joint1", 0.5); // 弧度

// 渲染一帧
urdf_plugin_render_frame(plugin);

// 获取图像数据
UrdfImageData img;
urdf_plugin_get_image_buffer(plugin, &img);
// img.data 包含 RGBA 像素数据

// 清理
urdf_plugin_destroy(plugin);
```

### 动画渲染
```c
for (int frame = 0; frame < num_frames; frame++) {
    // 更新关节角度
    double angle = sin(frame * 0.1);
    urdf_plugin_set_joint_angle(plugin, "joint1", angle);
    
    // 渲染并保存
    urdf_plugin_render_frame(plugin);
    urdf_plugin_get_image_buffer(plugin, &img);
    save_frame(img, frame);
}
```

## 🎨 材质和颜色

### URDF材质定义
插件自动读取URDF中的材质颜色：

```xml
<material name="gray">
  <color rgba="0.75 0.75 0.75 1.0"/>
</material>
```

### 纹理支持
如果URDF中定义了纹理，插件会自动加载：

```xml
<material name="textured">
  <texture filename="package://pkg/textures/metal.png"/>
  <color rgba="1 1 1 1"/>
</material>
```

## 🔧 关节控制

### 支持的关节类型
- **Revolute** - 旋转关节
- **Prismatic** - 移动关节
- **Continuous** - 连续旋转关节
- **Fixed** - 固定关节

### 正向运动学
插件自动计算完整的运动学链：
- 递归变换计算
- 支持任意深度的关节树
- 实时更新场景图

## 📊 渲染特性

### 透明背景
- 格式：RGBA（PF_BYTE_RGBA）
- 背景alpha通道：0（完全透明）
- 适合合成到其他界面（如Slint）

### 光照
- 环境光：Ambient light (0.8, 0.8, 0.8)
- 方向光：Directional light from (-1, -1, -1)
- 镜面反射：Specular highlighting enabled

### 相机控制
```c
UrdfCameraConfig cam = {
    .position = {2.0f, 2.0f, 1.5f},
    .look_at = {0.0f, 0.0f, 0.4f},
    .up = {0.0f, 0.0f, 1.0f},
    .fov_degrees = 50.0f,
    .near_clip = 0.1f,
    .far_clip = 1000.0f
};
urdf_plugin_set_camera(plugin, &cam);
```

## 🎯 输出文件

本目录包含示例输出：
- `robot_animation.gif` - 2秒循环动画（60帧）
- `robot_full_animation.gif` - 4秒循环动画（120帧）
- `robot_posed.png` - 单帧姿态示例
- `frame_*.png` - 关键帧样本

## 🔗 Slint集成

插件通过C API提供FFI接口，可直接从Slint调用：

```rust
// Slint FFI绑定示例
extern "C" {
    fn urdf_plugin_create(config: *const UrdfRenderConfig) -> UrdfPluginHandle;
    fn urdf_plugin_load_file(handle: UrdfPluginHandle, path: *const c_char) -> i32;
    fn urdf_plugin_render_frame(handle: UrdfPluginHandle) -> i32;
    fn urdf_plugin_get_image_buffer(handle: UrdfPluginHandle, data: *mut UrdfImageData) -> i32;
}
```

## 📈 性能

- **网格加载**：~140万顶点（mini1机器人）
- **渲染速度**：30+ FPS @ 800x600
- **内存占用**：~50MB（包括网格数据）

## 🐛 已知限制

- **连续渲染**：需要在主线程循环调用`renderFrame()`
- **图像保存**：目前仅支持内存缓冲区，PNG/JPEG保存待实现
- **纹理格式**：支持OGRE兼容的格式（PNG, JPG, TGA等）

## 📚 依赖

- OGRE 1.12.10+
- urdfdom
- assimp
- pthreads

