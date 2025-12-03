#pragma once
#include <threepp/threepp.hpp>
#include <vector>
#include <memory>
#include "BilSimulator/PowerUps.hpp"

namespace MyCar {

    class Game;

    class PowerUpManager {
    public:
        void add(std::shared_ptr<threepp::Object3D> object, PowerUp::Type type) {
            powerUps_.emplace_back(std::move(object), type);
        }

        void update(Game& game, const threepp::Object3D* car) {
            const auto& position = car->position;
            for (auto& power_up : powerUps_) {
                if (!power_up.active()) continue;
                const auto* object = power_up.object(); if (!object) continue;

                const float dx = position.x - object->position.x;
                if (const float dz = position.z - object->position.z; dx*dx + dz*dz < pickupRadiusSquared_) {
                    power_up.apply(game);
                    power_up.hide();
                }
            }
        }

        void setPickupRadius(const float r) { pickupRadiusSquared_ = r*r; }

    private:
        std::vector<PowerUp> powerUps_;
        float pickupRadiusSquared_{2.25f}; // default pickup radius
    };

}