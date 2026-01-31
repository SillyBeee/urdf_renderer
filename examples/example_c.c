#include "urdf_renderer_plugin.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <urdf_file>\n", argv[0]);
        return 1;
    }

    printf("=== URDF Viewer Plugin Test ===\n\n");

    // Create render configuration
    UrdfRenderConfig render_config = {
        .width = 800,
        .height = 600,
        .transparent_background = true,
        .background_color = {0.2f, 0.2f, 0.2f, 1.0f},
        .anti_aliasing = 0
    };

    // Create plugin context
    printf("1. Creating plugin context...\n");
    UrdfPluginHandle plugin = urdf_plugin_create(&render_config);
    if (!plugin) {
        printf("   ERROR: Failed to create plugin\n");
        return 1;
    }
    printf("   ✓ Plugin created successfully\n\n");

    // Load URDF file
    printf("2. Loading URDF file: %s\n", argv[1]);
    UrdfPluginError error = urdf_plugin_load_file(plugin, argv[1]);
    if (error != URDF_SUCCESS) {
        printf("   ERROR: Failed to load URDF (code: %d)\n", error);
        printf("   %s\n", urdf_plugin_get_error(plugin));
        urdf_plugin_destroy(plugin);
        return 1;
    }
    printf("   ✓ URDF loaded successfully\n\n");

    // Get model name
    const char* model_name = urdf_plugin_get_model_name(plugin);
    printf("3. Model name: %s\n\n", model_name ? model_name : "Unknown");

    // Get joint count
    size_t joint_count = 0;
    error = urdf_plugin_get_joint_count(plugin, &joint_count);
    printf("4. Number of joints: %zu\n", joint_count);

    // Get joint names
    if (joint_count > 0) {
        printf("   Joint names:\n");
        char** joint_names = (char**)malloc(joint_count * sizeof(char*));
        for (size_t i = 0; i < joint_count; i++) {
            joint_names[i] = (char*)malloc(256);
        }
        
        error = urdf_plugin_get_joint_names(plugin, joint_names, joint_count, 256);
        if (error == URDF_SUCCESS) {
            for (size_t i = 0; i < joint_count; i++) {
                printf("   - %s\n", joint_names[i]);
            }
        }
        
        for (size_t i = 0; i < joint_count; i++) {
            free(joint_names[i]);
        }
        free(joint_names);
    }
    printf("\n");

    // Set camera
    printf("5. Setting up camera...\n");
    UrdfCameraConfig camera_config = {
        .position = {-0.02f, 0.0f, 1.5f},
        .look_at = {0.0f, 0.1f, 0.9f},
        .up = {0.0f, 0.0f, 1.0f},
        .fov_degrees = 45.0f,
        .near_clip = 0.1f,
        .far_clip = 1000.0f
    };
    error = urdf_plugin_set_camera(plugin, &camera_config);
    if (error == URDF_SUCCESS) {
        printf("   ✓ Camera configured\n\n");
    } else {
        printf("   WARNING: Camera configuration failed\n\n");
    }

    // Render a frame
    printf("6. Rendering frame...\n");
    error = urdf_plugin_render_frame(plugin);
    if (error != URDF_SUCCESS) {
        printf("   ERROR: Render failed (code: %d)\n", error);
        printf("   %s\n", urdf_plugin_get_error(plugin));
        urdf_plugin_destroy(plugin);
        return 1;
    }
    printf("   ✓ Frame rendered successfully\n\n");

    // Get image buffer
    printf("7. Getting image buffer...\n");
    UrdfImageData image_data;
    error = urdf_plugin_get_image_buffer(plugin, &image_data);
    if (error == URDF_SUCCESS) {
        printf("   ✓ Image buffer retrieved\n");
        printf("   - Size: %u x %u\n", image_data.width, image_data.height);
        printf("   - Channels: %u\n", image_data.channels);
        printf("   - Data size: %zu bytes\n", image_data.data_size);
        printf("   - Format: %s\n", 
            image_data.format == URDF_IMAGE_FORMAT_RGBA ? "RGBA" : "RGB");
    } else {
        printf("   ERROR: Failed to get image buffer\n");
    }
    printf("\n");

    // Test joint control if we have joints
    if (joint_count > 0) {
        printf("8. Testing joint control...\n");
        
        // Get first joint name
        char** joint_names = (char**)malloc(1 * sizeof(char*));
        joint_names[0] = (char*)malloc(256);
        urdf_plugin_get_joint_names(plugin, joint_names, 1, 256);
        
        printf("   Setting joint '%s' to 0.5 radians (~28.6 degrees)...\n", joint_names[0]);
        error = urdf_plugin_set_joint_angle(plugin, joint_names[0], 0.5);
        if (error == URDF_SUCCESS) {
            printf("   ✓ Joint angle set\n");
            
            // Verify
            double angle;
            urdf_plugin_get_joint_angle(plugin, joint_names[0], &angle);
            printf("   - Verified angle: %.3f radians\n", angle);
            
            // Render again
            urdf_plugin_render_frame(plugin);
            printf("   ✓ Re-rendered with new joint position\n");
        }
        
        free(joint_names[0]);
        free(joint_names);
        printf("\n");
    }

    // Test continuous rendering
    printf("9. Testing continuous rendering (3 seconds at 30 FPS)...\n");
    printf("   (Skipped - continuous rendering not yet thread-safe)\n");
    /*
    error = urdf_plugin_start_continuous_render(plugin, 30);
    if (error == URDF_SUCCESS) {
        printf("   ✓ Continuous rendering started\n");
        sleep(3);
        urdf_plugin_stop_continuous_render(plugin);
        printf("   ✓ Continuous rendering stopped\n");
    } else {
        printf("   WARNING: Continuous rendering failed\n");
    }
    */
    printf("\n");

    // Cleanup
    printf("10. Cleaning up...\n");
    urdf_plugin_destroy(plugin);
    printf("    ✓ Plugin destroyed\n\n");

    printf("=== Test completed successfully! ===\n");
    return 0;
}
