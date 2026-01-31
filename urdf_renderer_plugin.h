/**
 * @file urdf_renderer_plugin.h
 * @brief URDF渲染器插件C语言API接口
 * @details 提供用于Slint或其他语言通过FFI调用的C接口。
 *          支持URDF模型加载、渲染、关节控制、图像导出等功能。
 * @author SillyBee
 * @date 2026-01-31
 * 
 * @note 本API是线程安全的前提是：同一个handle只在创建它的线程中使用
 * 
 * @example 基本使用示例
 * @code
 * // 创建插件
 * UrdfRenderConfig config = {800, 600, true, {0,0,0,0}, 0};
 * UrdfPluginHandle plugin = urdf_plugin_create(&config);
 * 
 * // 加载URDF
 * urdf_plugin_load_file(plugin, "robot.urdf");
 * 
 * // 设置关节角度
 * urdf_plugin_set_joint_angle(plugin, "joint1", 0.5);
 * 
 * // 渲染并获取图像
 * urdf_plugin_render_frame(plugin);
 * UrdfImageData img;
 * urdf_plugin_get_image_buffer(plugin, &img);
 * 
 * // 清理
 * urdf_plugin_destroy(plugin);
 * @endcode
 */

#ifndef URDF_RENDERER_PLUGIN_H
#define URDF_RENDERER_PLUGIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// 平台相关导出宏
// ============================================================================

#ifdef _WIN32
    #ifdef URDF_PLUGIN_EXPORTS
        #define URDF_PLUGIN_API __declspec(dllexport)  ///< Windows导出符号
    #else
        #define URDF_PLUGIN_API __declspec(dllimport)  ///< Windows导入符号
    #endif
#else
    #define URDF_PLUGIN_API __attribute__((visibility("default")))  ///< Linux/Unix导出符号
#endif

// ============================================================================
// 不透明句柄类型
// ============================================================================

/**
 * @typedef UrdfPluginHandle
 * @brief 插件上下文句柄（不透明指针）
 * @details 由urdf_plugin_create()创建，urdf_plugin_destroy()销毁。
 *          不要尝试直接访问或修改此指针指向的内容。
 */
typedef struct UrdfPluginContext* UrdfPluginHandle;

// ============================================================================
// 错误码
// ============================================================================

/**
 * @enum UrdfPluginError
 * @brief 插件操作错误码
 */
typedef enum {
    URDF_SUCCESS = 0,                    ///< 操作成功
    URDF_ERROR_INVALID_HANDLE = -1,      ///< 无效的插件句柄
    URDF_ERROR_INVALID_PARAMETER = -2,   ///< 无效的参数
    URDF_ERROR_FILE_NOT_FOUND = -3,      ///< 文件未找到
    URDF_ERROR_PARSE_FAILED = -4,        ///< URDF解析失败
    URDF_ERROR_INIT_FAILED = -5,         ///< 初始化失败
    URDF_ERROR_RENDER_FAILED = -6,       ///< 渲染失败
    URDF_ERROR_NO_MODEL_LOADED = -7,     ///< 尚未加载模型
    URDF_ERROR_JOINT_NOT_FOUND = -8,     ///< 关节未找到
    URDF_ERROR_OUT_OF_MEMORY = -9,       ///< 内存不足
    URDF_ERROR_NOT_IMPLEMENTED = -10     ///< 功能未实现
} UrdfPluginError;

// ============================================================================
// 图像格式
// ============================================================================

/**
 * @enum UrdfImageFormat
 * @brief 图像格式枚举
 */
typedef enum {
    URDF_IMAGE_FORMAT_RGBA = 0,  ///< RGBA原始格式（每像素4字节）
    URDF_IMAGE_FORMAT_RGB = 1,   ///< RGB原始格式（每像素3字节）
    URDF_IMAGE_FORMAT_PNG = 2,   ///< PNG压缩格式
    URDF_IMAGE_FORMAT_JPEG = 3   ///< JPEG压缩格式
} UrdfImageFormat;

// ============================================================================
// 渲染模式
// ============================================================================

/**
 * @enum UrdfRenderMode
 * @brief 渲染模式枚举
 */
typedef enum {
    URDF_RENDER_MODE_SINGLE = 0,      ///< 单帧模式（手动调用renderFrame）
    URDF_RENDER_MODE_CONTINUOUS = 1   ///< 连续模式（需循环调用renderFrame）
} UrdfRenderMode;

// ============================================================================
// 相机配置结构体
// ============================================================================

/**
 * @struct UrdfCameraConfig
 * @brief 相机配置参数
 * @details 定义相机在3D空间中的位置、朝向和投影参数
 */
typedef struct {
    float position[3];      ///< 相机位置 [x, y, z]（世界坐标系）
    float look_at[3];       ///< 观察目标点 [x, y, z]（世界坐标系）
    float up[3];            ///< 上方向向量 [x, y, z]（通常为[0, 0, 1]）
    float fov_degrees;      ///< 视场角（度数，推荐30-60度）
    float near_clip;        ///< 近裁剪面距离（推荐0.1）
    float far_clip;         ///< 远裁剪面距离（推荐1000）
} UrdfCameraConfig;

// ============================================================================
// 渲染配置结构体
// ============================================================================

/**
 * @struct UrdfRenderConfig
 * @brief 渲染配置参数
 * @details 控制渲染分辨率、背景、抗锯齿等
 */
typedef struct {
    uint32_t width;                    ///< 图像宽度（像素）
    uint32_t height;                   ///< 图像高度（像素）
    bool transparent_background;       ///< 是否使用透明背景（alpha=0）
    float background_color[4];         ///< 背景颜色 [R, G, B, A]（仅当transparent_background=false时使用）
    uint32_t anti_aliasing;            ///< MSAA抗锯齿采样数（0=关闭, 2, 4, 8, 16）
} UrdfRenderConfig;

// ============================================================================
// 图像数据结构体
// ============================================================================

/**
 * @struct UrdfImageData
 * @brief 图像数据描述符
 * @details 包含渲染结果的像素数据和元信息
 * @warning data指针由插件管理，不要手动释放！有效期到下次renderFrame()调用。
 */
typedef struct {
    uint8_t* data;          ///< 像素数据指针（行优先，从上到下）
    uint32_t width;         ///< 图像宽度（像素）
    uint32_t height;        ///< 图像高度（像素）
    uint32_t channels;      ///< 通道数（3=RGB, 4=RGBA）
    size_t data_size;       ///< 数据总大小（字节）= width * height * channels
    UrdfImageFormat format;  ///< 图像格式
} UrdfImageData;

// ============================================================================
// 关节信息结构体
// ============================================================================

/**
 * @struct UrdfJointInfo
 * @brief 关节详细信息
 * @details 包含关节的名称、当前角度、限位和类型
 */
typedef struct {
    char name[256];          ///< 关节名称（最长255字符）
    double current_angle;    ///< 当前角度（弧度）
    double min_limit;        ///< 关节下限（弧度或米）
    double max_limit;        ///< 关节上限（弧度或米）
    int joint_type;          ///< 关节类型：0=未知, 1=旋转, 2=移动, 3=连续旋转
} UrdfJointInfo;

// ============================================================================
// 上下文管理 / Context Management
// ============================================================================

/**
 * @brief 创建新的URDF查看器插件上下文
 * @param render_config 初始渲染配置（可为NULL使用默认值）
 * @return 插件上下文句柄，失败返回NULL
 * 
 * @details 初始化OGRE渲染系统，创建离屏渲染目标。
 *          默认配置：800x600分辨率，透明背景，无抗锯齿。
 * 
 * @note 必须在同一线程中使用返回的句柄
 * 
 * @code
 * UrdfRenderConfig config = {800, 600, true, {0,0,0,0}, 0};
 * UrdfPluginHandle plugin = urdf_plugin_create(&config);
 * if (!plugin) {
 *     fprintf(stderr, "Failed to create plugin\n");
 * }
 * @endcode
 */
URDF_PLUGIN_API UrdfPluginHandle urdf_plugin_create(const UrdfRenderConfig* render_config);

/**
 * @brief 销毁插件上下文并释放所有资源
 * @param handle 要销毁的插件句柄
 * 
 * @details 清理OGRE场景、纹理、网格等所有资源。
 *          调用后handle将失效，不可再使用。
 * 
 * @note 即使handle为NULL也是安全的（无操作）
 */
URDF_PLUGIN_API void urdf_plugin_destroy(UrdfPluginHandle handle);

/**
 * @brief 获取最后一次错误的详细信息
 * @param handle 插件句柄
 * @return 错误信息字符串（由插件管理，不要释放），handle无效返回"Invalid handle"
 * 
 * @details 当API函数返回错误码时，调用此函数获取详细的错误描述。
 * 
 * @code
 * UrdfPluginError err = urdf_plugin_load_file(plugin, "robot.urdf");
 * if (err != URDF_SUCCESS) {
 *     printf("Error: %s\n", urdf_plugin_get_error(plugin));
 * }
 * @endcode
 */
URDF_PLUGIN_API const char* urdf_plugin_get_error(UrdfPluginHandle handle);

// ============================================================================
// URDF操作 / URDF Operations
// ============================================================================

/**
 * @brief 加载URDF模型文件
 * @param handle 插件句柄
 * @param urdf_path URDF文件路径（相对或绝对路径）
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 解析URDF文件，加载网格，创建场景，构建关节树，
 *          应用正向运动学。支持package://路径。
 * 
 * @note 如果已加载模型，会先清除旧场景再加载新模型
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_load_file(UrdfPluginHandle handle, const char* urdf_path);

/**
 * @brief 获取已加载模型的名称
 * @param handle 插件句柄
 * @return 模型名称字符串（由插件管理，不要释放），未加载模型返回NULL
 * 
 * @details 返回URDF中<robot name="...">定义的名称
 */
URDF_PLUGIN_API const char* urdf_plugin_get_model_name(UrdfPluginHandle handle);

/**
 * @brief 获取模型中的关节数量
 * @param handle 插件句柄
 * @param count 输出参数：关节总数
 * @return URDF_SUCCESS表示成功，其他值为错误码
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_get_joint_count(UrdfPluginHandle handle, size_t* count);

/**
 * @brief 获取所有关节的名称
 * @param handle 插件句柄
 * @param names 输出参数：字符串指针数组（由调用者分配内存）
 * @param max_names 最多获取的关节数量
 * @param name_buffer_size 每个名称缓冲区的大小（字节）
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 将关节名称复制到调用者提供的缓冲区数组中
 * 
 * @code
 * size_t count;
 * urdf_plugin_get_joint_count(plugin, &count);
 * char** names = malloc(count * sizeof(char*));
 * for (size_t i = 0; i < count; i++) {
 *     names[i] = malloc(256);
 * }
 * urdf_plugin_get_joint_names(plugin, names, count, 256);
 * // 使用names...
 * for (size_t i = 0; i < count; i++) free(names[i]);
 * free(names);
 * @endcode
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_get_joint_names(
    UrdfPluginHandle handle, 
    char** names, 
    size_t max_names,
    size_t name_buffer_size);

/**
 * @brief 获取指定关节的详细信息
 * @param handle 插件句柄
 * @param joint_name 关节名称
 * @param info 输出参数：关节信息结构体
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 返回关节的类型、当前角度、限位等信息
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_get_joint_info(
    UrdfPluginHandle handle,
    const char* joint_name,
    UrdfJointInfo* info);

// ============================================================================
// 关节控制 / Joint Control
// ============================================================================

/**
 * @brief 设置关节角度（弧度）
 * @param handle 插件句柄
 * @param joint_name 关节名称
 * @param angle 角度值（弧度）
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 更新关节角度并自动应用正向运动学，更新场景中所有子链接的变换。
 *          旋转关节：angle单位为弧度
 *          移动关节：angle单位为米
 * 
 * @note 调用后需要renderFrame()才能看到更新效果
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_set_joint_angle(
    UrdfPluginHandle handle,
    const char* joint_name,
    double angle);

/**
 * @brief 获取关节的当前角度（弧度）
 * @param handle 插件句柄
 * @param joint_name 关节名称
 * @param angle 输出参数：当前角度（弧度）
 * @return URDF_SUCCESS表示成功，其他值为错误码
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_get_joint_angle(
    UrdfPluginHandle handle,
    const char* joint_name,
    double* angle);

/**
 * @brief 批量设置多个关节角度
 * @param handle 插件句柄
 * @param joint_names 关节名称数组
 * @param angles 角度数组（弧度），与joint_names长度相同
 * @param count 关节数量
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 比逐个调用set_joint_angle()更高效，只应用一次正向运动学
 * 
 * @code
 * const char* joints[] = {"joint1", "joint2", "joint3"};
 * double angles[] = {0.5, -0.3, 1.2};
 * urdf_plugin_set_multiple_joints(plugin, joints, angles, 3);
 * @endcode
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_set_multiple_joints(
    UrdfPluginHandle handle,
    const char** joint_names,
    const double* angles,
    size_t count);

/**
 * @brief 重置所有关节到零位
 * @param handle 插件句柄
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 将所有关节角度设置为0，并应用正向运动学
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_reset_joints(UrdfPluginHandle handle);

// ============================================================================
// 相机控制 / Camera Control
// ============================================================================

/**
 * @brief 设置相机配置
 * @param handle 插件句柄
 * @param camera_config 相机配置结构体
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 更新相机的位置、观察目标、视场角等参数
 * 
 * @code
 * UrdfCameraConfig cam;
 * cam.position[0] = 3.0; cam.position[1] = 3.0; cam.position[2] = 2.0;
 * cam.look_at[0] = 0.0; cam.look_at[1] = 0.0; cam.look_at[2] = 0.5;
 * cam.up[0] = 0.0; cam.up[1] = 0.0; cam.up[2] = 1.0;
 * cam.fov_degrees = 45.0;
 * cam.near_clip = 0.1; cam.far_clip = 100.0;
 * urdf_plugin_set_camera(plugin, &cam);
 * @endcode
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_set_camera(
    UrdfPluginHandle handle,
    const UrdfCameraConfig* camera_config);

/**
 * @brief 获取当前相机配置
 * @param handle 插件句柄
 * @param camera_config 输出参数：相机配置结构体
 * @return URDF_SUCCESS表示成功，其他值为错误码
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_get_camera(
    UrdfPluginHandle handle,
    UrdfCameraConfig* camera_config);

// ============================================================================
// 渲染控制 / Rendering Control
// ============================================================================

/**
 * @brief 更新渲染配置
 * @param handle 插件句柄
 * @param render_config 新的渲染配置
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 可以动态更改分辨率、背景透明度、抗锯齿等设置
 * 
 * @note 更改分辨率会重建渲染目标纹理
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_set_render_config(
    UrdfPluginHandle handle,
    const UrdfRenderConfig* render_config);

/**
 * @brief 渲染单帧
 * @param handle 插件句柄
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 执行一次渲染，将结果保存到内部缓冲区。
 *          调用后可通过get_image_buffer()获取图像数据。
 * 
 * @note 在连续渲染模式下，此函数仍然有效
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_render_frame(UrdfPluginHandle handle);

/**
 * @brief 启动连续渲染模式
 * @param handle 插件句柄
 * @param target_fps 目标帧率（0表示无限制）
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 设置连续渲染标志。实际渲染循环由用户控制，需持续调用renderFrame()。
 * 
 * @note 本实现中此函数仅设置标志，不创建独立线程
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_start_continuous_render(
    UrdfPluginHandle handle,
    uint32_t target_fps);

/**
 * @brief 停止连续渲染模式
 * @param handle 插件句柄
 * @return URDF_SUCCESS表示成功，其他值为错误码
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_stop_continuous_render(UrdfPluginHandle handle);

/**
 * @brief 检查是否处于连续渲染模式
 * @param handle 插件句柄
 * @return true表示连续渲染模式已激活，false表示未激活
 */
URDF_PLUGIN_API bool urdf_plugin_is_rendering(UrdfPluginHandle handle);

// ============================================================================
// 图像输出 / Image Output
// ============================================================================

/**
 * @brief 获取最近一次渲染的图像缓冲区
 * @param handle 插件句柄
 * @param image_data 输出参数：图像数据结构体
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 返回的data指针由插件管理，在下次renderFrame()调用前有效。
 *          不要手动释放data指针！
 * 
 * @warning 数据生命周期：仅在下次renderFrame()之前有效
 * 
 * @code
 * urdf_plugin_render_frame(plugin);
 * UrdfImageData img;
 * urdf_plugin_get_image_buffer(plugin, &img);
 * // 使用img.data（RGBA格式，每像素4字节）
 * // 不要free(img.data)!
 * @endcode
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_get_image_buffer(
    UrdfPluginHandle handle,
    UrdfImageData* image_data);

/**
 * @brief 将最近一次渲染的图像保存到文件
 * @param handle 插件句柄
 * @param file_path 输出文件路径
 * @param format 图像格式（PNG或JPEG）
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 使用OGRE内置编码器保存图像。
 *          PNG格式：无损，支持透明通道
 *          JPEG格式：有损，不支持透明通道
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_save_image(
    UrdfPluginHandle handle,
    const char* file_path,
    UrdfImageFormat format);

/**
 * @brief 将最近一次渲染的图像复制到用户提供的缓冲区
 * @param handle 插件句柄
 * @param buffer 用户分配的缓冲区（用于接收图像数据）
 * @param buffer_size 缓冲区大小（字节）
 * @param bytes_written 输出参数：实际写入的字节数
 * @return URDF_SUCCESS表示成功，其他值为错误码
 * 
 * @details 将图像数据复制到用户控制的内存，与get_image_buffer()不同，
 *          复制后的数据由用户管理生命周期，不受后续renderFrame()影响。
 * 
 * @note 确保buffer_size >= width * height * channels
 * 
 * @code
 * UrdfImageData img;
 * urdf_plugin_get_image_buffer(plugin, &img);
 * uint8_t* my_buffer = malloc(img.data_size);
 * size_t written;
 * urdf_plugin_copy_image_buffer(plugin, my_buffer, img.data_size, &written);
 * // 现在my_buffer包含图像副本，可以安全使用
 * free(my_buffer);
 * @endcode
 */
URDF_PLUGIN_API UrdfPluginError urdf_plugin_copy_image_buffer(
    UrdfPluginHandle handle,
    uint8_t* buffer,
    size_t buffer_size,
    size_t* bytes_written);

#ifdef __cplusplus
}
#endif

#endif // URDF_RENDERER_PLUGIN_H
