#include "ogre_viewer.cpp"
#include "urdf_loader.cpp"
int main() {
    OgreApp app;
    app.run();
    URDFLoader loader;
    loader.load("path/to/your/urdf/file.urdf");
    
    return 0;

    
}
