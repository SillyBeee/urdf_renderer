/**
 * @file urdf_renderer_plugin.hpp
 * @brief URDF渲染器插件C++实现类
 * @details 提供完整的URDF模型渲染功能，包括正向运动学、材质管理、
 *          透明背景渲染、图像输出等。使用OGRE引擎进行3D渲染。
 * @author SillyBee
 * @date 2026-01-31
 */

#pragma once

#include "urdf_renderer_plugin.h"
#include "urdf_loader.hpp"
#include <Ogre.h>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

// OpenCV支持（可选）
#ifdef HAVE_OPENCV
#include <opencv2/core.hpp>
#endif

/**
 * @class URDFRendererPlugin
 * @brief URDF渲染器插件主类
 * @details 封装了URDF模型的完整渲染管线，包括：
 *          - OGRE渲染引擎初始化和管理
 *          - URDF模型加载和场景构建
 *          - 正向运动学计算和应用
 *          - 相机和渲染参数控制
 *          - 图像缓冲区捕获和导出
 * 
 * @note 线程安全：大部分操作需要在创建插件的同一线程调用
 */
class URDFRendererPlugin {
public:
    /**
     * @brief 构造函数
     * @details 初始化成员变量，设置默认相机和渲染配置
     */
    URDFRendererPlugin();
    
    /**
     * @brief 析构函数
     * @details 自动调用shutdown()清理所有资源
     */
    ~URDFRendererPlugin();

    // ========================================================================
    // 初始化和清理
    // ========================================================================
    
    /**
     * @brief 初始化插件
     * @param config 渲染配置（可为nullptr使用默认配置）
     * @return 成功返回true，失败返回false
     * @details 初始化OGRE渲染系统，创建隐藏窗口和渲染纹理。
     *          必须在调用其他方法前先初始化。
     */
    bool initialize(const UrdfRenderConfig* config);
    
    /**
     * @brief 关闭插件并释放所有资源
     * @details 清理OGRE场景、纹理、渲染系统等。调用后需要重新initialize才能使用。
     */
    void shutdown();

    // ========================================================================
    // URDF操作
    // ========================================================================
    
    /**
     * @brief 加载URDF模型文件
     * @param path URDF文件路径
     * @return 错误码（URDF_SUCCESS表示成功）
     * @details 解析URDF文件，加载网格，创建场景节点，构建关节树，
     *          应用初始的正向运动学变换。
     */
    UrdfPluginError loadURDF(const std::string& path);
    
    /**
     * @brief 获取当前加载的模型名称
     * @return 模型名称字符串
     */
    std::string getModelName() const;
    
    /**
     * @brief 获取关节数量
     * @return 关节总数
     */
    size_t getJointCount() const;
    
    /**
     * @brief 获取所有关节名称
     * @return 关节名称列表
     */
    std::vector<std::string> getJointNames() const;
    
    /**
     * @brief 获取指定关节的详细信息
     * @param name 关节名称
     * @param info 输出参数，接收关节信息
     * @return 错误码
     * @details 返回关节的当前角度、限位、类型等信息
     */
    UrdfPluginError getJointInfo(const std::string& name, UrdfJointInfo* info) const;

    // ========================================================================
    // 关节控制
    // ========================================================================
    
    /**
     * @brief 设置单个关节角度
     * @param name 关节名称
     * @param angle 角度值（弧度）
     * @return 错误码
     * @details 设置关节角度并立即应用正向运动学更新场景。
     *          下次renderFrame()时将使用新的姿态。
     */
    UrdfPluginError setJointAngle(const std::string& name, double angle);
    
    /**
     * @brief 获取关节当前角度
     * @param name 关节名称
     * @param angle 输出参数，接收角度值（弧度）
     * @return 错误码
     */
    UrdfPluginError getJointAngle(const std::string& name, double* angle) const;
    
    /**
     * @brief 批量设置多个关节角度
     * @param names 关节名称列表
     * @param angles 对应的角度值列表（弧度）
     * @return 错误码
     * @details 一次性设置多个关节，只执行一次正向运动学计算，效率更高
     */
    UrdfPluginError setMultipleJoints(const std::vector<std::string>& names, 
                                      const std::vector<double>& angles);
    
    /**
     * @brief 重置所有关节到零位
     * @return 错误码
     * @details 将所有关节角度设为0并更新场景
     */
    UrdfPluginError resetJoints();

    // ========================================================================
    // 相机控制
    // ========================================================================
    
    /**
     * @brief 设置相机参数
     * @param config 相机配置结构体
     * @return 错误码
     * @details 设置相机位置、朝向、视场角、近远裁剪面等参数
     */
    UrdfPluginError setCamera(const UrdfCameraConfig& config);
    
    /**
     * @brief 获取当前相机参数
     * @param config 输出参数，接收相机配置
     * @return 错误码
     */
    UrdfPluginError getCamera(UrdfCameraConfig* config) const;

    // ========================================================================
    // 渲染控制
    // ========================================================================
    
    /**
     * @brief 更新渲染配置
     * @param config 新的渲染配置
     * @return 错误码
     * @details 可以更改分辨率、背景透明度、抗锯齿等。
     *          如果分辨率改变，会重新创建渲染纹理。
     */
    UrdfPluginError setRenderConfig(const UrdfRenderConfig& config);
    
    /**
     * @brief 渲染单帧
     * @return 错误码
     * @details 执行一次完整的渲染流程：更新OGRE场景 -> 渲染到纹理 -> 
     *          捕获帧缓冲区。渲染结果可通过getImageBuffer()获取。
     * @note 这是主要的渲染方法，适合单次渲染或在循环中调用
     */
    UrdfPluginError renderFrame();
    
    /**
     * @brief 启动连续渲染模式
     * @param target_fps 目标帧率
     * @return 错误码
     * @details 标记为连续渲染模式。用户需要在自己的循环中持续调用renderFrame()。
     * @note 当前实现不使用独立线程，需要调用者控制渲染循环
     */
    UrdfPluginError startContinuousRender(uint32_t target_fps);
    
    /**
     * @brief 停止连续渲染模式
     * @return 错误码
     */
    UrdfPluginError stopContinuousRender();
    
    /**
     * @brief 检查是否处于连续渲染模式
     * @return true表示连续渲染已启用
     */
    bool isRendering() const;

    // ========================================================================
    // 图像输出
    // ========================================================================
    
    /**
     * @brief 获取最后一次渲染的图像缓冲区
     * @param image_data 输出参数，接收图像数据指针和元信息
     * @return 错误码
     * @details 返回的数据指针由插件管理，有效期到下次renderFrame()调用。
     *          图像格式为RGBA或RGB，取决于渲染配置。
     * @warning 不要释放返回的data指针！
     */
    UrdfPluginError getImageBuffer(UrdfImageData* image_data);
    
    /**
     * @brief 保存图像到文件
     * @param path 输出文件路径
     * @param format 图像格式（PNG或JPEG）
     * @return 错误码
     * @note 当前未实现，返回URDF_ERROR_NOT_IMPLEMENTED
     */
    UrdfPluginError saveImage(const std::string& path, UrdfImageFormat format);
    
    /**
     * @brief 拷贝图像数据到用户缓冲区
     * @param buffer 用户分配的缓冲区
     * @param buffer_size 缓冲区大小（字节）
     * @param bytes_written 输出参数，实际写入的字节数
     * @return 错误码
     * @details 将图像数据拷贝到用户提供的缓冲区，适合需要持久化数据的场景
     */
    UrdfPluginError copyImageBuffer(uint8_t* buffer, size_t buffer_size, size_t* bytes_written);

#ifdef HAVE_OPENCV
    // ========================================================================
    // OpenCV集成 (可选特性)
    // ========================================================================
    
    /**
     * @brief 获取最近渲染的图像为cv::Mat格式
     * @return cv::Mat RGBA格式图像 (CV_8UC4)
     * @throws std::runtime_error 如果尚未渲染或buffer为空
     * 
     * @details 返回图像数据的深拷贝，调用者拥有所有权。
     *          颜色格式为RGBA，与当前渲染格式一致。
     *          
     * @note OpenCV默认使用BGR顺序，此函数返回RGBA顺序。
     *       如需转换为BGR，请使用：
     *       @code
     *       cv::Mat rgba_mat = plugin.getImageAsMat();
     *       cv::Mat bgr_mat;
     *       cv::cvtColor(rgba_mat, bgr_mat, cv::COLOR_RGBA2BGR);
     *       @endcode
     * 
     *       如需去除alpha通道转换为RGB：
     *       @code
     *       cv::Mat rgba_mat = plugin.getImageAsMat();
     *       cv::Mat rgb_mat;
     *       cv::cvtColor(rgba_mat, rgb_mat, cv::COLOR_RGBA2RGB);
     *       @endcode
     * 
     * @warning 此函数执行深拷贝，对于大图像(>1080p)可能有2-5ms延迟。
     *          如需零拷贝访问，请直接使用getImageBuffer()。
     * 
     * @par 性能参考
     *      - 800×600 RGBA: ~1.8MB, 拷贝时间约1-2ms
     *      - 1920×1080 RGBA: ~8MB, 拷贝时间约3-5ms
     * 
     * @see getImageBuffer() 零拷贝的原始缓冲区访问
     */
    cv::Mat getImageAsMat() const;
#endif

    // ========================================================================
    // 错误处理
    // ========================================================================
    
    /**
     * @brief 获取最后一次错误信息
     * @return 错误描述字符串
     * @details 当操作返回错误码时，可以调用此方法获取详细的错误信息
     */
    const std::string& getLastError() const { return last_error_; }

private:
    // ========================================================================
    // 内部初始化方法
    // ========================================================================
    
    /**
     * @brief 初始化OGRE渲染系统
     * @return 成功返回true
     * @details 创建OGRE Root，加载OpenGL插件，创建隐藏窗口，
     *          初始化场景管理器和相机，设置光照
     */
    bool initializeOgre();
    
    /**
     * @brief 创建渲染纹理
     * @return 成功返回true
     * @details 根据配置的分辨率和格式创建离屏渲染目标
     */
    bool createRenderTexture();
    
    /**
     * @brief 构建场景
     * @details 遍历URDF中的所有连杆，加载网格，创建实体和场景节点，
     *          应用材质和纹理，然后执行初始的正向运动学
     */
    void setupScene();
    
    /**
     * @brief 使用Assimp加载网格文件
     * @param filepath 网格文件路径
     * @param mesh_name OGRE网格资源名称
     * @return OGRE网格指针
     * @details 支持STL等格式，自动三角化和生成法线，
     *          转换为OGRE手动网格对象
     */
    Ogre::MeshPtr loadMeshWithAssimp(const std::string& filepath, const std::string& mesh_name);
    
    /**
     * @brief 从关节状态更新场景（预留接口）
     * @note 当前未使用，功能由applyJointTransforms实现
     */
    void updateSceneFromJoints();
    
    /**
     * @brief 应用正向运动学变换到场景节点
     * @details 遍历所有连杆节点，计算其在世界坐标系中的变换，
     *          更新场景节点的位置和旋转
     */
    void applyJointTransforms();
    
    /**
     * @brief 捕获当前帧缓冲区
     * @details 从渲染纹理读取像素数据到image_buffer_
     */
    void captureFrameBuffer();
    
    // ========================================================================
    // 辅助方法
    // ========================================================================
    
    /**
     * @brief 设置错误信息
     * @param error 错误描述
     * @details 记录错误信息并输出到stderr
     */
    void setError(const std::string& error);
    
    /**
     * @brief 查找连杆对应的场景节点
     * @param link_name 连杆名称
     * @return 场景节点指针，未找到返回nullptr
     */
    Ogre::SceneNode* findNodeForLink(const std::string& link_name);
    
    /**
     * @brief 递归计算连杆的世界变换
     * @param link_name 连杆名称
     * @param transform 输出参数，计算得到的4x4变换矩阵
     * @details 从根连杆开始递归计算，累积每个关节的变换
     */
    void computeLinkTransform(const std::string& link_name, Ogre::Matrix4& transform);
    
    /**
     * @brief 构建关节树数据结构
     * @details 遍历URDF模型，建立关节到连杆的映射表，
     *          用于正向运动学的快速查询
     */
    void buildJointTree();

    // ========================================================================
    // OGRE渲染资源
    // ========================================================================
    Ogre::Root* ogre_root_;              ///< OGRE根对象
    Ogre::SceneManager* scene_mgr_;      ///< 场景管理器
    Ogre::Camera* camera_;               ///< 相机对象
    Ogre::RenderTexture* render_texture_;///< 渲染纹理（离屏渲染目标）
    Ogre::TexturePtr texture_ptr_;       ///< 纹理智能指针
    
    // ========================================================================
    // URDF数据
    // ========================================================================
    URDFLoader urdf_loader_;  ///< URDF加载器实例
    
    // ========================================================================
    // 配置
    // ========================================================================
    UrdfRenderConfig render_config_;  ///< 当前渲染配置
    UrdfCameraConfig camera_config_;  ///< 当前相机配置
    
    // ========================================================================
    // 图像数据
    // ========================================================================
    std::vector<uint8_t> image_buffer_;  ///< 图像像素缓冲区
    uint32_t image_width_;               ///< 图像宽度
    uint32_t image_height_;              ///< 图像高度
    
    // ========================================================================
    // 场景图映射
    // ========================================================================
    std::unordered_map<std::string, Ogre::SceneNode*> link_nodes_;  ///< 连杆名到场景节点的映射
    Ogre::SceneNode* robot_root_node_;                                 ///< 机器人根节点（用于坐标系修正）
    
    // ========================================================================
    // 正向运动学数据结构
    // ========================================================================
    std::unordered_map<std::string, std::string> joint_to_child_link_;   ///< 关节名 -> 子连杆名
    std::unordered_map<std::string, std::string> joint_to_parent_link_;  ///< 关节名 -> 父连杆名
    std::unordered_map<std::string, std::string> link_to_parent_joint_;  ///< 连杆名 -> 父关节名
    
    // ========================================================================
    // 状态标志
    // ========================================================================
    bool initialized_;    ///< 是否已初始化
    bool model_loaded_;   ///< 是否已加载模型
    std::string last_error_;  ///< 最后一次错误信息
    
    // ========================================================================
    // 连续渲染（不使用独立线程）
    // ========================================================================
    std::atomic<bool> continuous_render_;  ///< 连续渲染标志
    uint32_t target_fps_;                  ///< 目标帧率
    mutable std::mutex render_mutex_;      ///< 渲染互斥锁
};
