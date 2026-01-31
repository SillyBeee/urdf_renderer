/**
 * @file urdf_loader.hpp
 * @brief URDF模型加载器
 * @details 提供URDF文件解析、网格路径解析、关节角度管理等功能
 * @author GitHub Copilot CLI
 * @date 2026-01-31
 */

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <filesystem>

#include <urdf_model/model.h>
#include <urdf_model/types.h>

namespace urdf {
using ModelInterfaceSharedPtr = std::shared_ptr<ModelInterface>;
}

/**
 * @class URDFLoader
 * @brief URDF模型加载和管理类
 * @details 负责加载URDF文件，解析模型结构，管理关节状态，提供网格和材质信息
 */
class URDFLoader {
public:
  /**
   * @struct VisualInfo
   * @brief 视觉元素信息结构体
   * @details 包含网格路径、缩放、原点变换、颜色和纹理信息
   */
  struct VisualInfo {
    std::string mesh_path;        ///< 网格文件路径（已解析为绝对路径）
    urdf::Vector3 scale{1.0, 1.0, 1.0};  ///< 网格缩放比例（默认1:1:1）
    urdf::Pose origin;            ///< 视觉元素相对于连杆的原点变换
    urdf::Color color;            ///< 材质颜色（RGBA格式）
    std::string texture_filename; ///< 可选纹理文件名
  };

  /**
   * @brief 加载URDF文件
   * @param path URDF文件路径（相对或绝对路径）
   * @return 成功返回true，失败返回false
   * @details 解析URDF文件，构建模型结构，初始化所有关节角度为0
   */
  bool load(const std::string& path);
  
  /**
   * @brief 获取模型名称
   * @return 模型名称字符串
   * @details 返回URDF中定义的robot name属性
   */
  std::string getModelName() const;
  
  /**
   * @brief 获取所有关节名称列表
   * @return 关节名称向量
   * @details 返回模型中所有关节的名称，顺序不保证
   */
  std::vector<std::string> getJointNames() const;
  
  /**
   * @brief 获取所有连杆名称列表
   * @return 连杆名称向量
   * @details 返回模型中所有连杆的名称，包括base_link
   */
  std::vector<std::string> getLinkNames() const;
  
  /**
   * @brief 获取指定连杆的视觉元素信息
   * @param link_name 连杆名称
   * @return 视觉元素信息向量
   * @details 返回该连杆的所有视觉元素，包括网格、颜色、纹理等信息。
   *          如果连杆不存在或无视觉元素，返回空向量
   */
  std::vector<VisualInfo> getLinkVisuals(const std::string& link_name) const;
  
  /**
   * @brief 获取URDF文件所在目录
   * @return URDF目录路径的optional包装
   * @details 用于解析相对路径的网格文件。如果尚未加载URDF，返回nullopt
   */
  std::optional<std::string> getURDFDirectory() const;
  
  /**
   * @brief 设置关节角度
   * @param name 关节名称
   * @param degrees 角度值（弧度）
   * @return 成功返回true，关节不存在返回false
   * @details 设置指定关节的角度。注意：参数名为degrees但实际单位是弧度（历史遗留）
   */
  bool setJointAngle(const std::string& name, double degrees);
  
  /**
   * @brief 获取关节当前角度
   * @param name 关节名称
   * @return 角度值的optional包装（弧度）
   * @details 如果关节不存在，返回nullopt
   */
  std::optional<double> getJointAngle(const std::string& name) const;

  urdf::ModelInterfaceSharedPtr model;  ///< URDF模型接口共享指针（urdfdom库提供）

private:
  std::unordered_map<std::string, double> joint_angles;  ///< 关节名称到角度的映射表
  std::filesystem::path urdf_directory;  ///< URDF文件所在目录路径
  
  /**
   * @brief 解析网格路径
   * @param raw_path 原始路径（可能包含package://前缀）
   * @return 解析后的绝对路径
   * @details 处理三种路径格式：
   *          1. package:// URI - 自动查找包目录
   *          2. 绝对路径 - 直接返回
   *          3. 相对路径 - 相对于URDF目录解析
   */
  std::string resolveMeshPath(const std::string& raw_path) const;
};
