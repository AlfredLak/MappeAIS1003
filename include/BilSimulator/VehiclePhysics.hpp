#pragma once
#include <threepp/threepp.hpp>
#include <algorithm>
#include <cmath>

namespace minbil {

struct CarPose {
    threepp::Vector3 position{};
    float yaw{0};
    float forwardVel{0}, lateralVel{0};
    float yawRate{0};
    bool reversing{false};
    bool drifting{false};
    bool slipping{false};
    float wheelSpinDelta{0};
};

class VehiclePhysics {
public:
    // Tunables
    float accelForward{20.f}, accelBackward{18.f};
    float brakeForce{30.f}, coastDrag{2.5f};
    float maxForward{30.f}, maxBackward{-25.f};

    float maxSteer{1.05f}, steerAccel{3.8f}, steerFriction{5.2f};
    float reverseSteerScale{0.30f}, reverseSteerAccel{2.0f}, reverseSteerFriction{6.5f}, reverseSteerClamp{0.30f};
    float reverseGrip{24.f};

    float gripNormal{12.f}, gripDriftBase{1.5f}, driftKick{80.f}, driftMinVel{2.f}, driftGripSpeed{0.08f}, driftGripMin{0.8f};
    float normalSlipFactor{0.45f}, normalKick{18.f};

    float airDrag{0.05f}, wheelRadius{0.35f};

    struct InputLite { bool forward{}, backward{}, left{}, right{}, drift{}; };

    inline CarPose step(const InputLite& input, float dt){
        CarPose pose{};

        float fx=std::sin(yaw_), fz=std::cos(yaw_), rx=std::cos(yaw_), rz=-std::sin(yaw_);
        float forwardV = velX_*fx + velZ_*fz;
        float lateralV = velX_*rx + velZ_*rz;

        float inputTurn = (input.left?+maxSteer:0.f) + (input.right?-maxSteer:0.f);
        bool onlyBrake = input.backward && !input.forward && !input.left && !input.right;
        bool reversing = (forwardV < -0.05f);
        float targetTurn = reversing ? (reverseSteerScale*inputTurn) : inputTurn;
        if (onlyBrake) targetTurn = 0.f;

        float response = (std::abs(targetTurn)>0)
                         ? (reversing?reverseSteerAccel:steerAccel)
                         : (reversing?reverseSteerFriction:steerFriction);
        float alpha = 1.f - std::exp(-response*dt);
        yawRate_ += (targetTurn - yawRate_) * alpha;
        if (reversing){
            if (yawRate_> reverseSteerClamp) yawRate_= reverseSteerClamp;
            if (yawRate_<-reverseSteerClamp) yawRate_=-reverseSteerClamp;
        }

        bool braking = input.backward && (forwardV>0.05f);
        if (input.forward) forwardV += accelForward*dt;
        if (input.backward){
            if (forwardV>0){ forwardV -= brakeForce*dt; if (forwardV<0) forwardV=0; }
            else { forwardV -= accelBackward*dt; }
        }

        if (!input.forward && !input.backward){
            if (forwardV>0){ forwardV -= coastDrag*dt; if (forwardV<0) forwardV=0; }
            else if (forwardV<0){ forwardV += coastDrag*dt; if (forwardV>0) forwardV=0; }
        }
        forwardV = std::clamp(forwardV, maxBackward, maxForward);

        bool allowDrift = input.drift && (forwardV>0.05f) && !onlyBrake;
        bool normalSlip = !allowDrift && (forwardV>0.05f) && (std::abs(yawRate_)>1e-4f) && !reversing;

        float grip = gripNormal;
        if (allowDrift){
            float speedFactor = std::clamp(std::abs(forwardV)*driftGripSpeed,0.f,3.f);
            grip = std::max(driftGripMin, gripDriftBase/(1.f+speedFactor));
            float sgn = (yawRate_>=0)? 1.f : -1.f;
            if (forwardV>driftMinVel){
                float g = 0.3f + 0.7f*std::min(std::abs(forwardV)/20.f,1.f);
                lateralV += sgn * driftKick * std::abs(yawRate_) * g * dt;
            }
        } else if (normalSlip){
            float sp = std::clamp(std::abs(forwardV)/15.f,0.f,1.5f);
            grip = gripNormal / (1.f + normalSlipFactor*sp);
            float sgn = (yawRate_>=0)? 1.f : -1.f;
            lateralV += sgn * normalKick * std::abs(yawRate_) * (0.5f+0.5f*sp) * dt;
        }

        if (reversing && !(input.left||input.right)) grip = reverseGrip;
        if (braking){ grip = std::max(grip, 40.f); yawRate_ *= (1.f - std::min(1.f, 8.f*dt)); }

        float latA = 1.f-std::exp(-grip*dt);
        lateralV += (0.f - lateralV) * latA;

        float air = std::max(0.f, 1.f - airDrag*dt);
        forwardV*=air; lateralV*=air;

        velX_ = fx*forwardV + rx*lateralV;
        velZ_ = fz*forwardV + rz*lateralV;
        position_.x += velX_*dt; position_.z += velZ_*dt;

        bool moving = (velX_*velX_+velZ_*velZ_ > 1e-6f);
        bool revNoTurn = reversing && !(input.left||input.right);
        bool freezeAlign = !moving || revNoTurn;
        float desiredYaw = moving ? std::atan2(velX_, velZ_) : yaw_;
        float alignRate = (reversing? 0.f : (allowDrift? 1.0f : 6.0f));
        float alignAlpha = freezeAlign? 0.f : (1.f - std::exp(-alignRate*dt));

        yaw_ += yawRate_*dt;
        if (!freezeAlign){
            float d = desiredYaw - yaw_;
            while (d> threepp::math::PI) d -= 2*threepp::math::PI;
            while (d<-threepp::math::PI) d += 2*threepp::math::PI;
            yaw_ += d * alignAlpha;
        }

        pose.position = position_;
        pose.yaw = yaw_;
        pose.forwardVel = forwardV;
        pose.lateralVel = lateralV;
        pose.yawRate = yawRate_;
        pose.reversing = reversing;
        pose.drifting = allowDrift;
        pose.slipping = normalSlip;
        pose.wheelSpinDelta = (wheelRadius>0)? (forwardV/wheelRadius)*dt : 0.f;
        return pose;
    }

    inline void setPosition(const threepp::Vector3& p){ position_=p; }
    inline void hardStop(){ velX_=0; velZ_=0; }
    inline const threepp::Vector3& position() const { return position_; }
    inline float speed() const { return std::sqrt(velX_*velX_ + velZ_*velZ_); }

private:
    threepp::Vector3 position_{0,0,0};
    float velX_{0}, velZ_{0};
    float yaw_{0}, yawRate_{0};
};

}