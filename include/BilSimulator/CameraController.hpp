#pragma once

#include <threepp/threepp.hpp>
#include <cmath>

namespace MyCar {

enum class CameraMode { Behind = 0, Side = 1, Front = 2 };
//Class controlling three camera perspectives
class CameraController {
public:
    CameraController() = default;

    // Provide the camera the controller should move
    void attach(threepp::PerspectiveCamera* cam) { camera_ = cam; }

    // Explicitly pick a mode
    void setMode(CameraMode mode) { mode_ = mode; }

    [[nodiscard]] CameraMode mode() const { return mode_; }

    // Cycle modes in order: Behind -> Side -> Front -> Behind
    void cycle() {
        switch (mode_) {
            case CameraMode::Behind: mode_ = CameraMode::Side;  break;
            case CameraMode::Side:   mode_ = CameraMode::Front; break;
            case CameraMode::Front:  mode_ = CameraMode::Behind;break;
        }
    }

    void update(const threepp::Vector3& carPosition, float carYaw) {
        if (!camera_) return;

        threepp::Vector3 desired;
        switch (mode_) {
            case CameraMode::Behind: {
                const float back = 6.f, height = 3.f;
                desired.set(carPosition.x - std::sin(carYaw) * back,
                            carPosition.y + height,
                            carPosition.z - std::cos(carYaw) * back);
                lookAt_ = carPosition;
                break;
            }
            case CameraMode::Front: {
                constexpr float front = 6.f, height = 3.f;
                desired.set(carPosition.x + std::sin(carYaw) * front,
                            carPosition.y + height,
                            carPosition.z + std::cos(carYaw) * front);
                lookAt_ = carPosition;
                break;
            }
            case CameraMode::Side: {
                constexpr float side = 5.f, height = 3.f;
                const float sideYaw = carYaw - threepp::math::PI/2.f;
                desired.set(carPosition.x - std::sin(sideYaw) * side,
                            carPosition.y + height,
                            carPosition.z - std::cos(sideYaw) * side);
                lookAt_ = carPosition;
                break;
            }
        }

        constexpr float follow = 10.f; // higher = snappier
        const double now = clock_.getElapsedTime();
        const auto dt = static_cast<float>(now - lastT_);
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