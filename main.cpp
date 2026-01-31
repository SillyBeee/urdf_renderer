#include "urdf_loader.hpp"
#include "ogre_viewer.cpp"

int main() {
    URDFLoader loader;
    if (!loader.load("path/to/your/urdf/file.urdf")) {
        return 1;
    }
    OgreApp app(loader);
    app.run();
    return 0;
}
