#pragma once
#include <threepp/threepp.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

#include "BilSimulator/VehiclePhysics.hpp"

namespace minbil {

class VehicleVisuals {
public:
    VehicleVisuals(threepp::Object3D* chassis, const std::vector<threepp::Object3D*>& wheels)
        : chassis_(chassis), wheels_(wheels) {}

    // drift visual
    float yawDriftScale=0.6f, lateralOffsetDrift=0.6f;
    float yawSlipScale =0.35f, lateralOffsetSlip =0.35f;
    float yawFollow=10.f, offsetFollow=12.f;

    inline void triggerCrashTilt(){ tiltTimer_ = tiltDuration_; }

    inline void apply(float dt, const struct CarPose& pose){
        for (auto* w:wheels_) w->rotation.x += pose.wheelSpinDelta;

        float yawTarget=0.f, offsetTarget=0.f;
        if (!pose.reversing){
            float safeV = (std::abs(pose.forwardVel)>1e-3f)? pose.forwardVel : (pose.forwardVel>=0? 1e-3f:-1e-3f);
            float slipAngle = std::atan2(pose.lateralVel, safeV);
            if (pose.drifting){
                float boost = 1.f + 0.8f*std::min(std::abs(pose.forwardVel)/15.f, 1.5f);
                slipAngle *= boost;
                yawTarget   =  yawDriftScale * slipAngle;
                offsetTarget= -lateralOffsetDrift * std::clamp(pose.lateralVel/10.f,-1.f,1.f);
            } else if (pose.slipping){
                float boost = 1.f + 0.55f*std::min(std::abs(pose.forwardVel)/15.f, 1.25f);
                slipAngle *= boost;
                yawTarget   =  yawSlipScale * slipAngle;
                offsetTarget= -lateralOffsetSlip * std::clamp(pose.lateralVel/10.f,-1.f,1.f);
            }
        }

        float aY = 1.f-std::exp(-yawFollow*dt);
        visualYaw_ += (yawTarget - visualYaw_) * aY;
        chassis_->rotation.y = visualYaw_;

        float aO = 1.f-std::exp(-offsetFollow*dt);
        visualOffset_ += (offsetTarget - visualOffset_) * aO;
        chassis_->position.set(visualOffset_, 0, 0);

        if (tiltTimer_ > 0.f){
            tiltTimer_ -= dt;
            float t = std::max(0.f, tiltTimer_ / tiltDuration_);
            float angle = tiltMax_ * t * (2.f - t);
            chassis_->rotation.x = angle;
        } else {
            chassis_->rotation.x *= std::exp(-10.f*dt);
        }
    }

private:
    threepp::Object3D* chassis_{};
    std::vector<threepp::Object3D*> wheels_;
    float visualYaw_{0}, visualOffset_{0};

    float tiltTimer_{0.f};
    float tiltDuration_{0.15f};
    float tiltMax_{0.10f};
};

}