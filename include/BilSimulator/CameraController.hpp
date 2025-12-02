#pragma once

#include <threepp/threepp.hpp>
#include <cmath>

namespace minbil {

enum class CameraMode { Behind = 0, Side = 1, Front = 2 };

class CameraController {
public:
    CameraController() = default;

    void attach(threepp::PerspectiveCamera* cam) { camera_ = cam; }

    void setMode(CameraMode mode) { mode_ = mode; }

    CameraMode mode() const { return mode_; }

    void cycle() {
        switch (mode_) {
            case CameraMode::Behind: mode_ = CameraMode::Side;  break;
            case CameraMode::Side:   mode_ = CameraMode::Front; break;
            case CameraMode::Front:  mode_ = CameraMode::Behind;break;
        }
    }

    void update(const threepp::Vector3& carPos, float carYaw) {
        if (!camera_) return;

        threepp::Vector3 desired;
        switch (mode_) {
            case CameraMode::Behind: {
                const float back = 6.f, height = 3.f;
                desired.set(carPos.x - std::sin(carYaw) * back,
                            carPos.y + height,
                            carPos.z - std::cos(carYaw) * back);
                lookAt_ = carPos;
                break;
            }
            case CameraMode::Front: {
                const float front = 6.f, height = 3.f;
                desired.set(carPos.x + std::sin(carYaw) * front,
                            carPos.y + height,
                            carPos.z + std::cos(carYaw) * front);
                lookAt_ = carPos;
                break;
            }
            case CameraMode::Side: {
                const float side = 5.f, height = 3.f;
                const float sideYaw = carYaw - threepp::math::PI/2.f;
                desired.set(carPos.x - std::sin(sideYaw) * side,
                            carPos.y + height,
                            carPos.z - std::cos(sideYaw) * side);
                lookAt_ = carPos;
                break;
            }
        }

        const float follow = 10.f; // higher = snappier
        const double now = clock_.getElapsedTime();
        const float dt = static_cast<float>(now - lastT_);
        lastT_ = now;
        const float alpha = 1.f - std::exp(-follow * dt);

        camera_->position.lerp(desired, alpha);
        camera_->lookAt(lookAt_);
    }

private:
    threepp::PerspectiveCamera* camera_{nullptr};
    CameraMode mode_{CameraMode::Behind};
    threepp::Vector3 lookAt_{0,0,0};
    threepp::Clock clock_{};
    double lastT_{clock_.getElapsedTime()};
};

}