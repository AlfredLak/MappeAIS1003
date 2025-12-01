#pragma once
#include <threepp/threepp.hpp>
#include <memory>
#include <vector>

namespace minbil {

    class Game; // fwd

    class Trees {
    public:
        void add(std::shared_ptr<threepp::Object3D> t){ trees_.push_back(std::move(t)); }
        void update(Game& g, threepp::Object3D* car, float currentSpeed){
            auto p=car->position;
            for (auto& t: trees_){
                auto* o=t.get(); if(!o) continue;
                float dx=p.x-o->position.x, dz=p.z-o->position.z;
                if (dx*dx+dz*dz < collisionRadius*collisionRadius){
                    onCollision(g, currentSpeed);
                    break;
                }
            }
        }
    private:
        inline void onCollision(Game& g, float speed);
        std::vector<std::shared_ptr<threepp::Object3D>> trees_;
        float collisionRadius{1.8f};
    };

}