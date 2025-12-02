#pragma once
#include <threepp/threepp.hpp>

namespace minbil {

enum class CameraMode { Behind, SideRight, Front };

class CameraController {
public:
    CameraController() = default;

    inline void attach(threepp::PerspectiveCamera* cam) { cam_ = cam; }
    inline void setMode(CameraMode m) { mode_ = m; }
    inline CameraMode mode() const { return mode_; }

    inline void cycle() {
        switch (mode_) {
            case CameraMode::Behind:   mode_ = CameraMode::SideRight; break;
            case CameraMode::SideRight:mode_ = CameraMode::Front;     break;
            case CameraMode::Front:    mode_ = CameraMode::Behind;    break;
        }
    }

    inline void update(const threepp::Vector3& carPos, float carYaw) {
        if (!cam_) return;

        threepp::Vector3 desired;
        switch (mode_) {
            case CameraMode::Behind: {
                float back = 6.f, h = 3.f;
                desired.set(carPos.x - std::sin(carYaw) * back,
                            carPos.y + h,
                            carPos.z - std::cos(carYaw) * back);
                cam_->position.copy(desired);
                cam_->lookAt(carPos);
                break;
            }
            case CameraMode::Front: {
                float front = 6.f, h = 3.f;
                desired.set(carPos.x + std::sin(carYaw) * front,
                            carPos.y + h,
                            carPos.z + std::cos(carYaw) * front);
                cam_->position.copy(desired);
                cam_->lookAt(carPos);
                break;
            }
            case CameraMode::SideRight: {
                float side = 5.f, h = 3.f;
                float sideYaw = carYaw - threepp::math::PI / 2.f;
                desired.set(carPos.x - std::sin(sideYaw) * side,
                            carPos.y + h,
                            carPos.z - std::cos(sideYaw) * side);
                cam_->position.copy(desired);
                cam_->lookAt(carPos);
                break;
            }
        }
    }

private:
    threepp::PerspectiveCamera* cam_{nullptr};
    CameraMode mode_{CameraMode::Behind};
};

}