#include <Ogre.h>

class OgreApp {
public:
  void run() {
    root = new Ogre::Root("plugins.cfg", "resources.cfg");
    if (!root->restoreConfig()) {
      root->showConfigDialog(nullptr);
    }
    window = root->initialise(true, "URDF Viewer");

    sceneMgr = root->createSceneManager();
    camera = sceneMgr->createCamera("MainCamera");
    camera->setNearClipDistance(0.1);

    viewport = window->addViewport(camera);
    viewport->setBackgroundColour(Ogre::ColourValue(0.2, 0.2, 0.2));

    sceneMgr->setAmbientLight(Ogre::ColourValue(1, 1, 1));

    Ogre::Entity *entity = sceneMgr->createEntity("base.stl");
    Ogre::SceneNode *node =
        sceneMgr->getRootSceneNode()->createChildSceneNode();
    node->attachObject(entity);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        "../assets/meshes", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    root->startRendering();

    // 清理资源
    root->shutdown();

    delete root;
  }

private:
  Ogre::Root *root;
  Ogre::RenderWindow *window;
  Ogre::SceneManager *sceneMgr;
  Ogre::Camera *camera;
  Ogre::Viewport *viewport;
};
