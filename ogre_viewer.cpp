#include <Ogre.h>
#include <iostream>
#include <string>

#include "urdf_loader.hpp"

class OgreApp {
public:
  explicit OgreApp(URDFLoader &loader) : loader_(loader) {}
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

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        "../assets/meshes", "FileSystem");
    if (const auto dir = loader_.getURDFDirectory()) {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          *dir, "FileSystem");
    }
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    const auto link_names = loader_.getLinkNames();
    if (!link_names.empty()) {
      std::cout << "Will render " << link_names.size() << " URDF links." << '\n';
    }
    for (const auto &link_name : link_names) {
      const auto visuals = loader_.getLinkVisuals(link_name);
      for (size_t idx = 0; idx < visuals.size(); ++idx) {
        const auto &visual = visuals[idx];
        if (visual.mesh_path.empty()) {
          continue;
        }
        const std::string entity_name = link_name + "_" + std::to_string(idx);
        try {
          Ogre::Entity *entity =
              sceneMgr->createEntity(entity_name, visual.mesh_path);
          Ogre::SceneNode *node =
              sceneMgr->getRootSceneNode()->createChildSceneNode();
          node->attachObject(entity);
          node->setScale(visual.scale.x, visual.scale.y, visual.scale.z);
          node->setPosition(visual.origin.position.x,
                            visual.origin.position.y,
                            visual.origin.position.z);
          node->setOrientation(
              Ogre::Quaternion(visual.origin.rotation.w,
                               visual.origin.rotation.x,
                               visual.origin.rotation.y,
                               visual.origin.rotation.z));
        } catch (const Ogre::Exception &err) {
          std::cerr << "failed to create entity " << entity_name << ": "
                    << err.getDescription() << '\n';
        }
      }
    }

    const auto joints = loader_.getJointNames();
    std::cout << "URDF joints available: " << joints.size() << '\n';
    if (!joints.empty()) {
      loader_.setJointAngle(joints.front(), 25.0);
      std::cout << "preset " << joints.front() << " -> 25 deg" << '\n';
    }

    root->startRendering();

    root->shutdown();
    delete root;
  }

private:
  URDFLoader &loader_;
  Ogre::Root *root = nullptr;
  Ogre::RenderWindow *window = nullptr;
  Ogre::SceneManager *sceneMgr = nullptr;
  Ogre::Camera *camera = nullptr;
  Ogre::Viewport *viewport = nullptr;
};
