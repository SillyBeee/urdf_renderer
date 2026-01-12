#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <iostream>
#include <string>

class URDFLoader {
public:
    bool load(const std::string& path) {
        auto parsed = urdf::parseURDFFile(path);
        if (!parsed) {
            std::cerr << "failed to parse URDF: " << path << '\n';
            return false;
        }
        this->model = parsed;
        std::cout << "Successfully loaded URDF model: " << model->getName() << '\n';
        return true;
    }

    std::string getModelName() const {
        if (model) {
            return model->getName();
        }
        return "";
    }

    

    urdf::ModelInterfaceSharedPtr model;
};
