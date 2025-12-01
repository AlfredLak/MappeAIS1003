#pragma once
#include <threepp/threepp.hpp>
#include <cmath>

namespace minbil {

    enum class CameraMode { Behind, Side, Front };

    class CameraController {
    public:
        inline void attach(threepp::PerspectiveCamera* cam) { camera_ = cam; }
        inline void setMode(CameraMode m) { mode_ = m; }
        inline CameraMode mode() const { return mode_; }

        inline void cycle() {
            switch (mode_) {
                case CameraMode::Behind: mode_ = CameraMode::Side;  break;
                case CameraMode::Side:   mode_ = CameraMode::Front; break;
                case CameraMode::Front:  mode_ = CameraMode::Behind; break;
            }
        }

        inline void update(const threepp::Vector3& carPos, float yaw) {
            if (!camera_) return;
            switch (mode_) {
                case CameraMode::Behind: placeBehind(carPos, yaw); break;
                case CameraMode::Side:   placeSide(carPos, yaw);   break;
                case CameraMode::Front:  placeFront(carPos, yaw);  break;
            }
        }

    private:
        inline void placeBehind(const threepp::Vector3& p, float yaw, float back=6.f, float h=3.f) {
            camera_->position.set(p.x-std::sin(yaw)*back, p.y+h, p.z-std::cos(yaw)*back);
            camera_->lookAt(p);
        }
        inline void placeFront(const threepp::Vector3& p, float yaw, float front=6.f, float h=3.f) {
            camera_->position.set(p.x+std::sin(yaw)*front, p.y+h, p.z+std::cos(yaw)*front);
            camera_->lookAt(p);
        }
        inline void placeSide(const threepp::Vector3& p, float yaw, float side=5.f, float h=3.f) {
            float sy = yaw - threepp::math::PI/2.f;
            camera_->position.set(p.x-std::sin(sy)*side, p.y+h, p.z-std::cos(sy)*side);
            camera_->lookAt(p);
        }

        threepp::PerspectiveCamera* camera_{nullptr};
        CameraMode mode_{CameraMode::Behind};
    };

}