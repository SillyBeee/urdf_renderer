#include "urdf_viewer_plugin.h"
#include "urdf_viewer_plugin.hpp"
#include <cstring>

// Internal context structure
struct UrdfPluginContext {
    URDFViewerPlugin* plugin;
};

// ============================================================================
// Context Management
// ============================================================================

UrdfPluginHandle urdf_plugin_create(const UrdfRenderConfig* render_config) {
    try {
        auto* context = new UrdfPluginContext();
        context->plugin = new URDFViewerPlugin();
        
        if (!context->plugin->initialize(render_config)) {
            delete context->plugin;
            delete context;
            return nullptr;
        }
        
        return context;
    } catch (...) {
        return nullptr;
    }
}

void urdf_plugin_destroy(UrdfPluginHandle handle) {
    if (!handle) return;
    
    if (handle->plugin) {
        delete handle->plugin;
    }
    delete handle;
}

const char* urdf_plugin_get_error(UrdfPluginHandle handle) {
    if (!handle || !handle->plugin) {
        return "Invalid handle";
    }
    return handle->plugin->getLastError().c_str();
}

// ============================================================================
// URDF Operations
// ============================================================================

UrdfPluginError urdf_plugin_load_file(UrdfPluginHandle handle, const char* urdf_path) {
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!urdf_path) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->loadURDF(urdf_path);
}

const char* urdf_plugin_get_model_name(UrdfPluginHandle handle) {
    if (!handle || !handle->plugin) {
        return nullptr;
    }
    
    static std::string model_name;
    model_name = handle->plugin->getModelName();
    return model_name.c_str();
}

UrdfPluginError urdf_plugin_get_joint_count(UrdfPluginHandle handle, size_t* count) {
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!count) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    *count = handle->plugin->getJointCount();
    return URDF_SUCCESS;
}

UrdfPluginError urdf_plugin_get_joint_names(
    UrdfPluginHandle handle,
    char** names,
    size_t max_names,
    size_t name_buffer_size) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!names) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    const auto joint_names = handle->plugin->getJointNames();
    const size_t count = std::min(max_names, joint_names.size());
    
    for (size_t i = 0; i < count; ++i) {
        strncpy(names[i], joint_names[i].c_str(), name_buffer_size - 1);
        names[i][name_buffer_size - 1] = '\0';
    }
    
    return URDF_SUCCESS;
}

UrdfPluginError urdf_plugin_get_joint_info(
    UrdfPluginHandle handle,
    const char* joint_name,
    UrdfJointInfo* info) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!joint_name || !info) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->getJointInfo(joint_name, info);
}

// ============================================================================
// Joint Control
// ============================================================================

UrdfPluginError urdf_plugin_set_joint_angle(
    UrdfPluginHandle handle,
    const char* joint_name,
    double angle) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!joint_name) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->setJointAngle(joint_name, angle);
}

UrdfPluginError urdf_plugin_get_joint_angle(
    UrdfPluginHandle handle,
    const char* joint_name,
    double* angle) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!joint_name || !angle) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->getJointAngle(joint_name, angle);
}

UrdfPluginError urdf_plugin_set_multiple_joints(
    UrdfPluginHandle handle,
    const char** joint_names,
    const double* angles,
    size_t count) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!joint_names || !angles) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    std::vector<std::string> names;
    std::vector<double> angle_vec;
    
    names.reserve(count);
    angle_vec.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
        names.push_back(joint_names[i]);
        angle_vec.push_back(angles[i]);
    }
    
    return handle->plugin->setMultipleJoints(names, angle_vec);
}

UrdfPluginError urdf_plugin_reset_joints(UrdfPluginHandle handle) {
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    
    return handle->plugin->resetJoints();
}

// ============================================================================
// Camera Control
// ============================================================================

UrdfPluginError urdf_plugin_set_camera(
    UrdfPluginHandle handle,
    const UrdfCameraConfig* camera_config) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!camera_config) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->setCamera(*camera_config);
}

UrdfPluginError urdf_plugin_get_camera(
    UrdfPluginHandle handle,
    UrdfCameraConfig* camera_config) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!camera_config) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->getCamera(camera_config);
}

// ============================================================================
// Rendering
// ============================================================================

UrdfPluginError urdf_plugin_set_render_config(
    UrdfPluginHandle handle,
    const UrdfRenderConfig* render_config) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!render_config) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->setRenderConfig(*render_config);
}

UrdfPluginError urdf_plugin_render_frame(UrdfPluginHandle handle) {
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    
    return handle->plugin->renderFrame();
}

UrdfPluginError urdf_plugin_start_continuous_render(
    UrdfPluginHandle handle,
    uint32_t target_fps) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    
    return handle->plugin->startContinuousRender(target_fps);
}

UrdfPluginError urdf_plugin_stop_continuous_render(UrdfPluginHandle handle) {
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    
    return handle->plugin->stopContinuousRender();
}

bool urdf_plugin_is_rendering(UrdfPluginHandle handle) {
    if (!handle || !handle->plugin) {
        return false;
    }
    
    return handle->plugin->isRendering();
}

// ============================================================================
// Image Output
// ============================================================================

UrdfPluginError urdf_plugin_get_image_buffer(
    UrdfPluginHandle handle,
    UrdfImageData* image_data) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!image_data) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->getImageBuffer(image_data);
}

UrdfPluginError urdf_plugin_save_image(
    UrdfPluginHandle handle,
    const char* file_path,
    UrdfImageFormat format) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!file_path) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->saveImage(file_path, format);
}

UrdfPluginError urdf_plugin_copy_image_buffer(
    UrdfPluginHandle handle,
    uint8_t* buffer,
    size_t buffer_size,
    size_t* bytes_written) {
    
    if (!handle || !handle->plugin) {
        return URDF_ERROR_INVALID_HANDLE;
    }
    if (!buffer || !bytes_written) {
        return URDF_ERROR_INVALID_PARAMETER;
    }
    
    return handle->plugin->copyImageBuffer(buffer, buffer_size, bytes_written);
}
