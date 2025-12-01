#pragma once
#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>
#include <memory>
#include <vector>

namespace minbil {

    inline void findWheels(threepp::Object3D* root, std::vector<threepp::Object3D*>& out){
        const char* names[]={"wheel_FL","wheel_FR","wheel_BL","wheel_BR"};
        root->traverse([&](threepp::Object3D& o){ for(auto n:names) if(o.name==n){ out.push_back(&o); break; }});
    }

    struct VehicleRig {
        std::shared_ptr<threepp::Object3D> root;
        std::shared_ptr<threepp::Object3D> chassis;
        std::vector<threepp::Object3D*> wheels;
    };

    inline VehicleRig buildVehicle(threepp::AssimpLoader& loader, const std::string& file, threepp::Scene& scene){
        VehicleRig rig;
        auto model = loader.load(file);
        rig.root = threepp::Object3D::create();
        rig.chassis = threepp::Object3D::create();
        rig.chassis->add(model);
        rig.root->add(rig.chassis);
        rig.root->position.y = 0.f;
        scene.add(rig.root);
        findWheels(rig.chassis.get(), rig.wheels);
        return rig;
    }

}