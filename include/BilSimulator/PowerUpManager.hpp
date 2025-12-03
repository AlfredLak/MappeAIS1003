#pragma once

#include <threepp/threepp.hpp>
#include <vector>
#include <memory>
#include "BilSimulator/PowerUps.hpp"

namespace MyCar {

    class Game;

    class PowerUpManager {
    public:
        // store a new power-up with its scene object and type
        void add(std::shared_ptr<threepp::Object3D> object, PowerUp::Type type) {
            powerUps_.emplace_back(std::move(object), type); // emplace avoids extra copies
        }

        // check for pickups and apply effects if the car is close enough
        void update(Game& game, const threepp::Object3D* car) {
            const auto& position = car->position; // read car world position once

            for (auto& power_up : powerUps_) {
                if (!power_up.active()) continue;            // skip already collected power-ups
                const auto* object = power_up.object();      // get underlying scene node
                if (!object) continue;                       // safety: object may be null

                // compute XZ-plane distance to avoid Y/height affecting pickup
                const float distanceX = position.x - object->position.x;
                const float distanceZ = position.z - object->position.z;

                // compare squared distance to squared radius (no sqrt needed)
                if (distanceX*distanceX + distanceZ*distanceZ < pickupRadiusSquared_) {
                    power_up.apply(game); // trigger effect on the game state
                    power_up.hide();      // hide and mark as inactive
                }
            }
        }

        // set pickup radius in meters (stored internally as squared)
        void setPickupRadius(float r) { pickupRadiusSquared_ = r*r; }

    private:
        std::vector<PowerUp> powerUps_;   // all power-ups currently in the level
        float pickupRadiusSquared_{2.25f}; // default radius^2
    };

}