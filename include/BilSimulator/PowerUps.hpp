#pragma once
#include <threepp/threepp.hpp>
#include <memory>
#include <vector>

namespace minbil {

    class Game; // fwd

    class PowerUp {
    public:
        enum class Type { Grow, Faster, Shrink };
        PowerUp(std::shared_ptr<threepp::Object3D> obj, Type t): obj_(std::move(obj)), type_(t) {}
        inline threepp::Object3D* object(){ return obj_.get(); }
        inline bool active() const { return active_; }

        inline void apply(Game& g);
        inline void hide(){ active_=false; if(obj_) obj_->visible=false; }

    private:
        std::shared_ptr<threepp::Object3D> obj_;
        Type type_;
        bool active_{true};
    };

    class PowerUpManager {
    public:
        inline void add(std::shared_ptr<threepp::Object3D> obj, PowerUp::Type t){ items_.emplace_back(std::move(obj),t); }
        void update(Game& g, threepp::Object3D* car){
            auto p=car->position;
            for(auto& pu:items_){
                if(!pu.active()) continue;
                auto* o=pu.object(); if(!o) continue;
                float dx=p.x-o->position.x, dz=p.z-o->position.z;
                if(dx*dx+dz*dz < 2.25f){ pu.apply(g); pu.hide(); }
            }
        }
    private:
        std::vector<PowerUp> items_;
    };


}