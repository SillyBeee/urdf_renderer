#include "urdf_loader.hpp"

#include <urdf_parser/urdf_parser.h>
#include <iostream>

bool URDFLoader::load(const std::string& path) {
    auto parsed = urdf::parseURDFFile(path);
    if (!parsed) {
        std::cerr << "failed to parse URDF: " << path << '\n';
        return false;
    }
    model = parsed;
    joint_angles.clear();
    urdf_directory = std::filesystem::path(path).parent_path();
    for (const auto& [name, joint] : model->joints_) {
        (void)joint;
        joint_angles[name] = 0.0;
    }
    std::cout << "Successfully loaded URDF model: " << model->getName()
                        << " with " << joint_angles.size() << " joints." << '\n';
    return true;
}

std::string URDFLoader::getModelName() const {
    if (model) {
        return model->getName();
    }
    return {};
}

std::vector<std::string> URDFLoader::getJointNames() const {
    std::vector<std::string> names;
    names.reserve(joint_angles.size());
    for (const auto& [name, _] : joint_angles) {
        names.push_back(name);
    }
    return names;
}

bool URDFLoader::setJointAngle(const std::string& name, double degrees) {
    const auto it = joint_angles.find(name);
    if (it == joint_angles.end()) {
        return false;
    }
    it->second = degrees;
    return true;
}

std::optional<double> URDFLoader::getJointAngle(const std::string& name) const {
    const auto it = joint_angles.find(name);
    if (it == joint_angles.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<std::string> URDFLoader::getLinkNames() const {
    std::vector<std::string> names;
    if (!model) {
        return names;
    }
    names.reserve(model->links_.size());
    for (const auto& [name, link] : model->links_) {
        (void)link;
        names.push_back(name);
    }
    return names;
}

std::vector<URDFLoader::VisualInfo> URDFLoader::getLinkVisuals(const std::string& link_name) const {
    std::vector<VisualInfo> visuals;
    if (!model) {
        return visuals;
    }
    const auto it = model->links_.find(link_name);
    if (it == model->links_.end()) {
        return visuals;
    }
    auto addVisual = [&visuals, this](const urdf::VisualSharedPtr& visual) {
        if (!visual || !visual->geometry || visual->geometry->type != urdf::Geometry::MESH) {
            return;
        }
        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
        if (!mesh || mesh->filename.empty()) {
            return;
        }
        VisualInfo info;
        info.mesh_path = resolveMeshPath(mesh->filename);
        info.scale = mesh->scale;
        info.origin = visual->origin;
        
        // Get material color if available
        if (visual->material) {
            info.color = visual->material->color;
            info.texture_filename = visual->material->texture_filename;
        } else {
            // Default gray color
            info.color.r = 0.8;
            info.color.g = 0.8;
            info.color.b = 0.8;
            info.color.a = 1.0;
        }
        
        visuals.push_back(info);
    };
    addVisual(it->second->visual);
    for (const auto& visual : it->second->visual_array) {
        addVisual(visual);
    }
    return visuals;
}

std::optional<std::string> URDFLoader::getURDFDirectory() const {
    if (urdf_directory.empty()) {
        return std::nullopt;
    }
    return urdf_directory.string();
}

std::string URDFLoader::resolveMeshPath(const std::string& raw_path) const {
    if (raw_path.empty()) {
        return {};
    }
    
    // Handle package:// URIs
    if (raw_path.rfind("package://", 0) == 0) {
        // Extract the path after package://
        std::string package_path = raw_path.substr(10); // Remove "package://"
        
        // Find the package name (first part before /)
        size_t slash_pos = package_path.find('/');
        if (slash_pos != std::string::npos) {
            std::string package_name = package_path.substr(0, slash_pos);
            std::string relative_path = package_path.substr(slash_pos + 1);
            
            // Try to resolve relative to URDF directory parent (assuming package is there)
            if (!urdf_directory.empty()) {
                // Go up until we find the package directory
                std::filesystem::path search_path = urdf_directory;
                for (int i = 0; i < 3; ++i) { // Search up to 3 levels
                    std::filesystem::path candidate = search_path / package_name / relative_path;
                    if (std::filesystem::exists(candidate)) {
                        try {
                            return std::filesystem::canonical(candidate).string();
                        } catch (...) {
                            return std::filesystem::absolute(candidate).string();
                        }
                    }
                    search_path = search_path.parent_path();
                    if (search_path.empty()) break;
                }
                
                // If not found searching up, try as direct child of urdf_directory parent
                std::filesystem::path parent_candidate = urdf_directory.parent_path() / package_name / relative_path;
                if (std::filesystem::exists(parent_candidate)) {
                    try {
                        return std::filesystem::canonical(parent_candidate).string();
                    } catch (...) {
                        return std::filesystem::absolute(parent_candidate).string();
                    }
                }
                
                // Last resort: assume meshes are in a sibling meshes directory
                std::filesystem::path meshes_candidate = urdf_directory.parent_path() / relative_path;
                if (std::filesystem::exists(meshes_candidate)) {
                    try {
                        return std::filesystem::canonical(meshes_candidate).string();
                    } catch (...) {
                        return std::filesystem::absolute(meshes_candidate).string();
                    }
                }
            }
        }
        // If we can't resolve it, return the original path and let OGRE try
        return raw_path;
    }
    
    std::filesystem::path candidate(raw_path);
    if (candidate.is_absolute()) {
        if (std::filesystem::exists(candidate)) {
            try {
                return std::filesystem::canonical(candidate).string();
            } catch (...) {
                return raw_path;
            }
        }
        return raw_path;
    }
    
    if (urdf_directory.empty()) {
        return raw_path;
    }
    
    // Make relative paths absolute
    std::filesystem::path resolved = urdf_directory / candidate;
    if (std::filesystem::exists(resolved)) {
        try {
            return std::filesystem::canonical(resolved).string();
        } catch (...) {
            return std::filesystem::absolute(resolved).string();
        }
    }
    
    return (urdf_directory / candidate).string();
}
