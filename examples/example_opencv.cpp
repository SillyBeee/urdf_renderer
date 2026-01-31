/**
 * @file example_opencv.cpp
 * @brief URDF Viewer Plugin OpenCV集成示例
 * @details 演示如何使用getImageAsMat()获取cv::Mat格式的渲染图像，
 *          并使用OpenCV进行图像处理和保存。
 * 
 * 编译：已在CMakeLists.txt中配置
 * 运行：./example_opencv <urdf_path>
 */

#include "urdf_viewer_plugin.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cmath>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <urdf_path>" << std::endl;
        std::cerr << "示例: " << argv[0] << " arm_description/urdf/miniarm.urdf" << std::endl;
        return 1;
    }

    std::string urdf_path = argv[1];
    
    std::cout << "=== URDF Viewer Plugin - OpenCV集成示例 ===" << std::endl;
    std::cout << std::endl;

    // 1. 创建插件实例
    std::cout << "1. 创建插件上下文..." << std::endl;
    auto plugin = std::make_unique<URDFViewerPlugin>();
    
    // 2. 初始化插件
    UrdfRenderConfig render_config;
    render_config.width = 800;
    render_config.height = 600;
    render_config.transparent_background = true;  // RGBA格式
    render_config.anti_aliasing = 0;
    
    if (!plugin->initialize(&render_config)) {
        std::cerr << "初始化失败: " << plugin->getLastError() << std::endl;
        return 1;
    }
    std::cout << "   ✓ 插件初始化成功" << std::endl;

    // 3. 加载URDF
    std::cout << "\n2. 加载URDF: " << urdf_path << std::endl;
    if (plugin->loadURDF(urdf_path) != URDF_SUCCESS) {
        std::cerr << "加载URDF失败: " << plugin->getLastError() << std::endl;
        return 1;
    }
    
    size_t joint_count = plugin->getJointCount();
    std::cout << "   ✓ 加载成功，关节数: " << joint_count << std::endl;

    // 4. 设置相机
    std::cout << "\n3. 设置相机参数..." << std::endl;
    UrdfCameraConfig camera;
    camera.position[0] = 2.5f; camera.position[1] = 2.5f; camera.position[2] = 2.0f;
    camera.look_at[0] = 0.0f; camera.look_at[1] = 0.0f; camera.look_at[2] = 0.5f;
    camera.up[0] = 0.0f; camera.up[1] = 0.0f; camera.up[2] = 1.0f;
    camera.fov_degrees = 45.0f;
    camera.near_clip = 0.1f;
    camera.far_clip = 100.0f;
    plugin->setCamera(camera);
    std::cout << "   ✓ 相机设置完成" << std::endl;

    // 5. 渲染初始姿态并保存（RGBA格式）
    std::cout << "\n4. 渲染初始姿态..." << std::endl;
    if (plugin->renderFrame() != URDF_SUCCESS) {
        std::cerr << "渲染失败: " << plugin->getLastError() << std::endl;
        return 1;
    }
    
    try {
        // 获取RGBA cv::Mat
        cv::Mat rgba_image = plugin->getImageAsMat();
        std::cout << "   ✓ 获取cv::Mat: " << rgba_image.cols << "x" << rgba_image.rows 
                  << " RGBA (" << rgba_image.channels() << " channels)" << std::endl;
        std::cout << "   - Mat类型: " << (rgba_image.type() == CV_8UC4 ? "CV_8UC4" : "未知") << std::endl;
        std::cout << "   - 数据大小: " << rgba_image.total() * rgba_image.elemSize() / 1024.0 << " KB" << std::endl;
        std::cout << "   - 内存连续: " << (rgba_image.isContinuous() ? "是" : "否") << std::endl;

        // 保存原始RGBA图像
        cv::imwrite("opencv_rgba_output.png", rgba_image);
        std::cout << "   ✓ 保存RGBA图像: opencv_rgba_output.png" << std::endl;

        // 转换为BGR格式（OpenCV标准格式）
        cv::Mat bgr_image;
        cv::cvtColor(rgba_image, bgr_image, cv::COLOR_RGBA2BGR);
        cv::imwrite("opencv_bgr_output.png", bgr_image);
        std::cout << "   ✓ 保存BGR图像: opencv_bgr_output.png" << std::endl;

        // 转换为RGB格式（去除alpha）
        cv::Mat rgb_image;
        cv::cvtColor(rgba_image, rgb_image, cv::COLOR_RGBA2RGB);
        cv::imwrite("opencv_rgb_output.jpg", rgb_image);
        std::cout << "   ✓ 保存RGB图像: opencv_rgb_output.jpg" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "OpenCV操作失败: " << e.what() << std::endl;
        return 1;
    }

    // 6. 生成多姿态动画序列
    std::cout << "\n5. 生成多姿态动画序列..." << std::endl;
    
    if (joint_count > 0) {
        std::vector<std::string> joint_names = plugin->getJointNames();

        const int num_frames = 30;
        for (int frame = 0; frame < num_frames; frame++) {
            // 计算关节角度（正弦波动画）
            double t = frame / static_cast<double>(num_frames - 1);
            double angle = std::sin(t * 2.0 * M_PI) * 0.8;  // ±45度

            // 设置所有关节角度
            for (size_t i = 0; i < joint_count; i++) {
                plugin->setJointAngle(joint_names[i], angle * (i + 1) / joint_count);
            }

            // 渲染并获取cv::Mat
            plugin->renderFrame();
            cv::Mat frame_image = plugin->getImageAsMat();

            // 转换为BGR并添加帧号标记
            cv::Mat bgr_frame;
            cv::cvtColor(frame_image, bgr_frame, cv::COLOR_RGBA2BGR);
            
            // 在图像上绘制帧号
            std::string frame_text = "Frame " + std::to_string(frame + 1) + "/" + std::to_string(num_frames);
            cv::putText(bgr_frame, frame_text, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // 保存帧
            char filename[256];
            snprintf(filename, sizeof(filename), "opencv_frame_%03d.png", frame);
            cv::imwrite(filename, bgr_frame);

            if (frame % 10 == 0) {
                std::cout << "   - 已生成 " << (frame + 1) << "/" << num_frames << " 帧" << std::endl;
            }
        }
        
        std::cout << "   ✓ 完成 " << num_frames << " 帧动画序列" << std::endl;
        std::cout << "\n提示：使用以下命令合成GIF动画：" << std::endl;
        std::cout << "  convert -delay 3 -loop 0 opencv_frame_*.png opencv_animation.gif" << std::endl;
    }

    // 7. 图像处理示例：边缘检测
    std::cout << "\n6. OpenCV图像处理示例..." << std::endl;
    plugin->resetJoints();
    plugin->renderFrame();
    
    try {
        cv::Mat rgba = plugin->getImageAsMat();
        cv::Mat gray, edges, colored_edges;

        // 转换为灰度
        cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
        
        // Canny边缘检测
        cv::Canny(gray, edges, 50, 150);
        
        // 彩色边缘（绿色）
        cv::cvtColor(edges, colored_edges, cv::COLOR_GRAY2BGR);
        
        cv::imwrite("opencv_edges.png", colored_edges);
        std::cout << "   ✓ 边缘检测结果: opencv_edges.png" << std::endl;

        // 高斯模糊
        cv::Mat blurred;
        cv::cvtColor(rgba, blurred, cv::COLOR_RGBA2BGR);
        cv::GaussianBlur(blurred, blurred, cv::Size(15, 15), 0);
        cv::imwrite("opencv_blurred.png", blurred);
        std::cout << "   ✓ 高斯模糊结果: opencv_blurred.png" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "图像处理失败: " << e.what() << std::endl;
    }

    std::cout << "\n=== 所有测试完成 ===" << std::endl;
    std::cout << "\n生成的文件：" << std::endl;
    std::cout << "  - opencv_rgba_output.png (RGBA原始格式)" << std::endl;
    std::cout << "  - opencv_bgr_output.png (BGR格式)" << std::endl;
    std::cout << "  - opencv_rgb_output.jpg (RGB JPEG格式)" << std::endl;
    std::cout << "  - opencv_frame_*.png (动画帧序列)" << std::endl;
    std::cout << "  - opencv_edges.png (边缘检测)" << std::endl;
    std::cout << "  - opencv_blurred.png (高斯模糊)" << std::endl;

    return 0;
}
