#include "urdf_viewer_plugin.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

void save_frame_ppm(const UrdfImageData* img, int frame_num) {
    char filename[256];
    snprintf(filename, sizeof(filename), "frame_%04d.ppm", frame_num);
    
    FILE* f = fopen(filename, "wb");
    if (!f) return;
    
    fprintf(f, "P6\n%u %u\n255\n", img->width, img->height);
    for (size_t i = 0; i < img->width * img->height; i++) {
        fputc(img->data[i*4+0], f); // R
        fputc(img->data[i*4+1], f); // G
        fputc(img->data[i*4+2], f); // B
    }
    fclose(f);
}

int main(int argc, char** argv) {
    const char* urdf_file = argc > 1 ? argv[1] : "../arm_description/urdf/miniarm.urdf";
    const int num_frames = argc > 2 ? atoi(argv[2]) : 120; // 4 seconds at 30fps
    const int fps = 30;
    
    printf("=== URDF Video Generator ===\n");
    printf("URDF: %s\n", urdf_file);
    printf("Frames: %d (%.1f seconds at %d FPS)\n\n", num_frames, num_frames/(float)fps, fps);
    
    // Create plugin
    UrdfRenderConfig config = {800, 600, true, {0,0,0,0}, 0};
    UrdfPluginHandle plugin = urdf_plugin_create(&config);
    if (!plugin) {
        printf("Failed to create plugin\n");
        return 1;
    }
    
    // Load URDF
    if (urdf_plugin_load_file(plugin, urdf_file) != URDF_SUCCESS) {
        printf("Failed to load URDF\n");
        urdf_plugin_destroy(plugin);
        return 1;
    }
    
    printf("Loaded model: %s\n", urdf_plugin_get_model_name(plugin));
    
    // Get joint names
    size_t joint_count;
    urdf_plugin_get_joint_count(plugin, &joint_count);
    
    char** joint_names = (char**)malloc(joint_count * sizeof(char*));
    for (size_t i = 0; i < joint_count; i++) {
        joint_names[i] = (char*)malloc(256);
    }
    urdf_plugin_get_joint_names(plugin, joint_names, joint_count, 256);
    
    printf("Joints: %zu\n", joint_count);
    for (size_t i = 0; i < joint_count; i++) {
        printf("  - %s\n", joint_names[i]);
    }
    printf("\n");
    
    // Set camera
    UrdfCameraConfig cam = {
        {2.0f, 2.0f, 1.5f},
        {0.0f, 0.0f, 0.4f},
        {0.0f, 0.0f, 1.0f},
        50.0f, 0.1f, 1000.0f
    };
    urdf_plugin_set_camera(plugin, &cam);
    
    // Render animation
    printf("Rendering %d frames...\n", num_frames);
    
    for (int frame = 0; frame < num_frames; frame++) {
        // Animate joints with sinusoidal motion
        double t = (double)frame / num_frames * 2.0 * M_PI;
        
        for (size_t i = 0; i < joint_count; i++) {
            double angle = 0.5 * sin(t + i * M_PI / 3.0); // Different phase for each joint
            urdf_plugin_set_joint_angle(plugin, joint_names[i], angle);
        }
        
        // Render frame
        if (urdf_plugin_render_frame(plugin) != URDF_SUCCESS) {
            printf("Failed to render frame %d\n", frame);
            break;
        }
        
        // Get and save image
        UrdfImageData img;
        if (urdf_plugin_get_image_buffer(plugin, &img) == URDF_SUCCESS) {
            save_frame_ppm(&img, frame);
        }
        
        // Progress indicator
        if ((frame + 1) % 30 == 0 || frame == num_frames - 1) {
            printf("  Progress: %d/%d frames (%.1f%%)\n", 
                   frame + 1, num_frames, 100.0 * (frame + 1) / num_frames);
        }
    }
    
    printf("\nDone! Generated %d frames.\n", num_frames);
    printf("\nTo create video with ffmpeg:\n");
    printf("  ffmpeg -framerate %d -i frame_%%04d.ppm -c:v libx264 -pix_fmt yuv420p robot_animation.mp4\n", fps);
    printf("\nOr convert to animated GIF:\n");
    printf("  convert -delay %d frame_*.ppm robot_animation.gif\n", 100/fps);
    
    // Cleanup
    for (size_t i = 0; i < joint_count; i++) {
        free(joint_names[i]);
    }
    free(joint_names);
    
    urdf_plugin_destroy(plugin);
    return 0;
}
