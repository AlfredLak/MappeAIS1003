#pragma once
#include <threepp/threepp.hpp>

#include "InputState.hpp"
#include "CameraController.hpp"
#include "VehiclePhysics.hpp"
#include "VehicleVisuals.hpp"
#include "PowerUps.hpp"
#include "PowerUpManager.hpp"
#include "Trees.hpp"
#include "BilSimulator/CollisionSink.hpp"

namespace MyCar {

class Game : public CollisionSink {
public:
    Game(threepp::Canvas* canvas,
         threepp::GLRenderer* renderer,
         threepp::Scene* scene,
         threepp::PerspectiveCamera* camera,
         threepp::Object3D* carRoot,
         threepp::Object3D* chassis,
         const std::vector<threepp::Object3D*>& wheels,
         InputState* input,
         PowerUpManager* powerUps,
         Trees* trees)
        : canvas_(canvas), renderer_(renderer), scene_(scene), camera_(camera),
          carRoot_(carRoot), input_(input), powerUps_(powerUps), trees_(trees),
          visuals_(chassis, wheels) {

        cameraController_.attach(camera_);
        cameraController_.setMode(CameraMode::Behind);

        lastTime_ = clock_.getElapsedTime();
        lastSafePosition_ = carRoot_->position;
    }

    void updateFrame() {
        // timestep
        const double now = clock_.getElapsedTime();
        auto dt = static_cast<float>(now - lastTime_);
        lastTime_ = now;
        if (dt > 0.033f) dt = 0.033f;

        lastSafePosition_ = carRoot_->position;

        // step physics
        VehiclePhysics::InputLite in{ input_->forward, input_->backward, input_->left, input_->right, input_->drift };
        const CarPose pose = physics_.step(in, dt);

        // apply to scene graph
        carRoot_->position.copy(pose.position);
        carRoot_->rotation.y = pose.yaw;
        visuals_.apply(dt, pose);

        // interactions
        if (powerUps_) powerUps_->update(*this, carRoot_);
        if (trees_)    trees_->update(carRoot_, physics_.speed());

        // camera
        if (input_->camToggle && !prevCamToggle_) cameraController_.cycle();
        prevCamToggle_ = input_->camToggle;
        cameraController_.update(carRoot_->position, pose.yaw);

        // render
        if (renderer_ && scene_ && camera_) {
            renderer_->render(*scene_, *camera_);
        }
    }

    inline void reset() {
        physics_.setPosition({0,0,0});
        physics_.hardStop();
        carRoot_->position.set(0,0,0);
        carRoot_->rotation.set(0,0,0);
        carRoot_->scale.set(1,1,1);
        lastSafePosition_ = carRoot_->position;
        cameraController_.setMode(CameraMode::Behind);
        prevCamToggle_ = false;
    }

    // power-ups
    inline void growCar(float factor) { carRoot_->scale.multiplyScalar(factor); }
    inline void faster(float maxFactor, float accelFactor) {
        physics_.maxForward   *= maxFactor;
        physics_.accelForward *= accelFactor;
    }

    // per-vehicle tuning used by tests or model swaps
    inline void applyVehicleTuning(float accelF, float accelB, float maxF, float maxB) {
        physics_.accelForward  = accelF;
        physics_.accelBackward = accelB;
        physics_.maxForward    = maxF;
        physics_.maxBackward   = maxB;
    }

    // CollisionSink implementation
    inline void onTreeCollision(float impactSpeed) override { onCollision(impactSpeed); }

    // collision response
    inline void onCollision(float impactSpeed) {
        const double now = clock_.getElapsedTime();
        if (impactSpeed < crashSpeedThreshold_ || (now - lastCrashTime_) < crashCooldown_) {
            physics_.hardStop();
            physics_.setPosition(lastSafePosition_);
            carRoot_->position.copy(lastSafePosition_);
            return;
        }
        lastCrashTime_ = now;
        physics_.hardStop();
        physics_.setPosition(lastSafePosition_);
        carRoot_->position.copy(lastSafePosition_);
        visuals_.triggerCrashTilt();
    }

    inline VehiclePhysics& physics() { return physics_; }

private:
    // externals
    threepp::Canvas* canvas_{nullptr};
    threepp::GLRenderer* renderer_{nullptr};
    threepp::Scene* scene_{nullptr};
    threepp::PerspectiveCamera* camera_{nullptr};

    // scene graph
    threepp::Object3D* carRoot_{nullptr};

    // systems
    InputState*     input_{nullptr};
    PowerUpManager* powerUps_{nullptr};
    Trees*          trees_{nullptr};

    // subsystems
    VehiclePhysics  physics_{};
    VehicleVisuals  visuals_;
    CameraController cameraController_{};

    // timing/state
    threepp::Clock clock_{};
    double lastTime_{0.0};
    threepp::Vector3 lastSafePosition_{};
    bool prevCamToggle_{false};

    // collision tuning
    double lastCrashTime_{-1.0};
    float crashCooldown_{0.25f};
    float crashSpeedThreshold_{2.5f};
};

    inline void PowerUp::apply(Game& game) const {
        switch (type_) {
            case Type::Grow:   game.growCar(1.4f);       break;
            case Type::Faster: game.faster(1.35f, 1.3f); break;
            case Type::Shrink: game.growCar(0.7f);       break;
        }
    }

}