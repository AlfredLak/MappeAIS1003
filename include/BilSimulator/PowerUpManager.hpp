#pragma once
#include <threepp/threepp.hpp>
#include <vector>
#include <memory>
#include "BilSimulator/PowerUps.hpp"

namespace minbil {

    class Game; // fwd

    class PowerUpManager {
    public:
        inline void add(std::shared_ptr<threepp::Object3D> object, PowerUp::Type type) {
            powerUps_.emplace_back(std::move(object), type);
        }

        inline void update(Game& game, threepp::Object3D* car) {
            const auto& p = car->position;
            for (auto& pu : powerUps_) {
                if (!pu.active()) continue;
                auto* o = pu.object(); if (!o) continue;

                const float dx = p.x - o->position.x;
                const float dz = p.z - o->position.z;
                if (dx*dx + dz*dz < pickupRadiusSquared_) {
                    pu.apply(game);
                    pu.hide();
                }
            }
        }

        inline void setPickupRadius(float r) { pickupRadiusSquared_ = r*r; }

    private:
        std::vector<PowerUp> powerUps_;
        float pickupRadiusSquared_{2.25f}; // default = 1.5m radius
    };

}