#pragma once

#include <memory>
#include <string>
#include <vector>

#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

namespace MyCar {

    struct CarRig {
        std::shared_ptr<threepp::Object3D> root;
        std::shared_ptr<threepp::Object3D> chassis;
        std::vector<threepp::Object3D*> wheels;
    };

    struct VehicleFactory {

        static inline CarRig buildCar(threepp::AssimpLoader& loader,
                                      const std::string& modelFile,
                                      threepp::Scene& scene) {
            CarRig rig;

            auto modelNode = loader.load(modelFile);

            rig.root    = threepp::Object3D::create();
            rig.chassis = threepp::Object3D::create();

            rig.chassis->add(modelNode);
            rig.root->add(rig.chassis);
            rig.root->position.y = 0.f;

            scene.add(rig.root);

            findWheels(rig.chassis.get(), rig.wheels);
            return rig;
        }

    private:
        static void findWheels(threepp::Object3D* root, std::vector<threepp::Object3D*>& out) {
            static const char* names[] = {"wheel_FL","wheel_FR","wheel_BL","wheel_BR"};
            root->traverse([&](threepp::Object3D& o){
                for (const auto n : names) {
                    if (o.name == n) { out.push_back(&o); break; }
                }
            });
        }
    };

}