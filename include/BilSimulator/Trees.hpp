#pragma once
#include <threepp/threepp.hpp>
#include <memory>
#include <vector>
#include "CollisionSink.hpp"

namespace minbil {
    class CollisionSink;

    class Trees {
    public:
        Trees() = default;

        inline void setCollisionSink(CollisionSink* sink) { sink_ = sink; }

        inline void add(std::shared_ptr<threepp::Object3D> t) { trees_.push_back(std::move(t)); }

        inline void update(threepp::Object3D* car, float currentSpeed) {
            if (!car) return;
            const auto p = car->position;

            for (auto& t : trees_) {
                auto* o = t.get();
                if (!o) continue;
                const float dx = p.x - o->position.x;
                const float dz = p.z - o->position.z;
                if ((dx * dx + dz * dz) < (radius_ * radius_)) {
                    if (sink_) sink_->onTreeCollision(currentSpeed);
                    break;
                }
            }
        }

        inline void setRadius(float r) { radius_ = r; }

    private:
        std::vector<std::shared_ptr<threepp::Object3D>> trees_;
        CollisionSink* sink_{nullptr};
        float radius_{1.8f};
    };

}