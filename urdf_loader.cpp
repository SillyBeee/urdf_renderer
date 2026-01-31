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
    if (raw_path.rfind("package://", 0) == 0) {
        return raw_path;
    }
    std::filesystem::path candidate(raw_path);
    if (candidate.is_absolute() || urdf_directory.empty()) {
        return raw_path;
    }
    return (urdf_directory / candidate).string();
}
