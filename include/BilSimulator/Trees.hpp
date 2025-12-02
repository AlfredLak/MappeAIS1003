#pragma once
#include <threepp/threepp.hpp>
#include <memory>
#include <vector>
#include "BilSimulator/CollisionSink.hpp"

namespace minbil {

    class Trees {
    public:
        inline void setCollisionSink(CollisionSink* sink) { sink_ = sink; }
        inline void add(std::shared_ptr<threepp::Object3D> t) { trees_.push_back(std::move(t)); }

        // 2D circle collision against car position; forwards hit to sink.
        inline void update(threepp::Object3D* car, float currentSpeed) {
            if (!car) return;
            const auto p = car->position;
            for (auto& t : trees_) {
                auto* obj = t.get();
                if (!obj) continue;
                const float dx = p.x - obj->position.x;
                const float dz = p.z - obj->position.z;
                if (dx * dx + dz * dz < radius_ * radius_) {
                    if (sink_) sink_->onTreeCollision(currentSpeed);
                    break; // one hit per frame
                }
            }
        }

        inline void setCollisionRadius(float r) { radius_ = r; }
        inline float radius() const { return radius_; }
        inline const std::vector<std::shared_ptr<threepp::Object3D>>& all() const { return trees_; }

    private:
        std::vector<std::shared_ptr<threepp::Object3D>> trees_;
        float radius_{1.8f};
        CollisionSink* sink_{nullptr};
    };

}