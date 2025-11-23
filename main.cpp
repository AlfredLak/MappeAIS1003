#include <iostream>
#include <cmath>
#include <functional>
#include <chrono>
#include <random>
#include <memory>
#include <algorithm>

#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

using namespace threepp;

// ------------ input ------------
class DriveInput : public KeyListener {
public:
    bool forward{false}, backward{false}, left{false}, right{false}, view{false}, drift{false};
    void onKeyPressed(KeyEvent evt) override {
        switch (evt.key) {
            case Key::W: forward = true; break;
            case Key::S: backward = true; break;
            case Key::A: left = true; break;
            case Key::D: right = true; break;
            case Key::V: view = true; break;
            case Key::SPACE: drift = true; break;
            default: break;
        }
    }
    void onKeyReleased(KeyEvent evt) override {
        switch (evt.key) {
            case Key::W: forward = false; break;
            case Key::S: backward = false; break;
            case Key::A: left = false; break;
            case Key::D: right = false; break;
            case Key::V: view = false; break;
            case Key::SPACE: drift = false; break;
            default: break;
        }
    }
};

// ------------ camera helpers ------------
static void placeCameraThirdPerson(PerspectiveCamera& cam, const Vector3& pos, float yaw, float back=6.f, float h=3.f) {
    const float bx = std::sin(yaw) * back, bz = std::cos(yaw) * back;
    cam.position.set(pos.x - bx, pos.y + h, pos.z - bz);
    cam.lookAt(pos);
}
static void placeCameraFront(PerspectiveCamera& cam, const Vector3& pos, float yaw, float front=6.f, float h=3.f) {
    const float fx = std::sin(yaw) * front, fz = std::cos(yaw) * front;
    cam.position.set(pos.x + fx, pos.y + h, pos.z + fz);
    cam.lookAt(pos);
}
static void placeCameraSideRight(PerspectiveCamera& cam, const Vector3& pos, float yaw, float side=5.f, float h=3.f) {
    const float sy = yaw - math::PI/2.f, sx = std::sin(sy) * side, sz = std::cos(sy) * side;
    cam.position.set(pos.x - sx, pos.y + h, pos.z - sz);
    cam.lookAt(pos);
}

// ------------ wheels lookup ------------
static void collectWheels(Object3D* root, std::vector<Object3D*>& out) {
    const char* names[] = {"wheel_FL","wheel_FR","wheel_BL","wheel_BR"};
    root->traverse([&](Object3D& o){ for (auto n: names) if (o.name==n) { out.push_back(&o); break; } });
}

class Game; // fwd

// ------------ power-ups ------------
class PowerUp {
public:
    enum class Type { Grow, Faster, Shrink };
    PowerUp(std::shared_ptr<Object3D> obj, Type t): obj_(std::move(obj)), type_(t) {}
    Object3D* object() const { return obj_.get(); }
    bool isActive() const { return active_; }
    Type type() const { return type_; }
    void apply(Game& game);
    void deactivate(){ active_=false; if (obj_) obj_->visible=false; }
private:
    std::shared_ptr<Object3D> obj_;
    Type type_;
    bool active_{true};
};

class PowerUpManager {
public:
    void add(std::shared_ptr<Object3D> obj, PowerUp::Type t){ items_.emplace_back(std::move(obj), t); }
    void update(Game& game, Object3D* car){
        if (!car) return;
        const auto& p = car->position;
        for (auto& pu: items_){
            if (!pu.isActive()) continue;
            auto* o = pu.object(); if (!o) continue;
            const float dx=p.x-o->position.x, dz=p.z-o->position.z;
            if (dx*dx+dz*dz < 1.5f*1.5f){ pu.apply(game); pu.deactivate(); }
        }
    }
private:
    std::vector<PowerUp> items_;
};

// ------------ game------------
class Game {
public:
    Game(Canvas* canvas, GLRenderer* renderer, Scene* scene, PerspectiveCamera* camera,
         Object3D* carRoot, Object3D* chassis, DriveInput* input, PowerUpManager* powerups)
        : canvas_(canvas), renderer_(renderer), scene_(scene), camera_(camera),
          carRoot_(carRoot), chassis_(chassis), input_(input), powerups_(powerups) {
        collectWheels(chassis_, wheels_);
        placeCameraThirdPerson(*camera_, carRoot_->position, yaw_);
        lastTime_ = clock_.getElapsedTime();
    }

    void update() {
        const double now = clock_.getElapsedTime();
        float dt = static_cast<float>(now - lastTime_);
        lastTime_ = now;
        if (dt > 0.033f) dt = 0.033f;

        // basis
        const float fwdX = std::sin(yaw_), fwdZ = std::cos(yaw_);
        const float rightX = std::cos(yaw_), rightZ = -std::sin(yaw_);

        // local velocity
        float v_f = velX_*fwdX + velZ_*fwdZ;
        float v_r = velX_*rightX + velZ_*rightZ;

        // steering (soft + stable in reverse)
        float inputTurn = 0.f;
        if (input_->left)  inputTurn += maxTurnRate_;
        if (input_->right) inputTurn -= maxTurnRate_;

        const bool reversing = (v_f < -0.05f);
        float targetTurn = reversing ? (reverseTurnScale_ * inputTurn) : inputTurn; // no sign flip

        const bool onlyS = input_->backward && !input_->forward && !input_->left && !input_->right;
        if (onlyS) targetTurn = 0.f;

        {
            const float resp = (std::abs(targetTurn) > 0.f)
                               ? (reversing ? reverseTurnAccel_ : turnAccel_)
                               : (reversing ? reverseTurnFriction_ : turnFriction_);
            const float alpha = 1.f - std::exp(-resp * dt);
            turnVel_ += (targetTurn - turnVel_) * alpha;

            if (reversing) {
                const float lim = reverseTurnVelClamp_;
                if (turnVel_ >  lim) turnVel_ =  lim;
                if (turnVel_ < -lim) turnVel_ = -lim;
            }
        }

        // throttle + brake
        const bool brakingPhase = input_->backward && (v_f > 0.05f); // S while moving forward
        if (input_->forward)  v_f += accelForward_  * dt;
        if (input_->backward){
            if (v_f > 0.f){ v_f -= brakeDecel_ * dt; if (v_f < 0.f) v_f = 0.f; }
            else           { v_f -= accelBackward_ * dt; }
        }

        // passive drag
        if (!input_->forward && !input_->backward){
            if (v_f > 0.f){ v_f -= drag_ * dt; if (v_f < 0.f) v_f = 0.f; }
            else if (v_f < 0.f){ v_f += drag_ * dt; if (v_f > 0.f) v_f = 0.f; }
        }

        // caps
        v_f = std::clamp(v_f, maxBackward_, maxForward_);

        // slip logic
        const bool allowDrift  = input_->drift && (v_f > 0.05f) && !onlyS;       // forward-only drift
        const bool normalSlip  = !allowDrift && (v_f > 0.05f) && (std::abs(turnVel_) > 1e-4f) && !reversing;

        float grip = lateralGripNormal_;

        if (allowDrift) {
            // more slide at high speed
            const float sf = std::clamp(std::abs(v_f) * driftGripSpeedFactor_, 0.f, 3.f);
            grip = std::max(minDriftGrip_, lateralGripDrift_ / (1.f + sf));

            const float signTurn = (turnVel_>=0.f)? 1.f : -1.f;
            if (v_f > driftKickMinSpeed_){
                const float speedGain = 0.3f + 0.7f * std::min(std::abs(v_f)/20.f, 1.f);
                const float kick = driftLatAccel_ * std::abs(turnVel_) * speedGain;
                v_r += signTurn * kick * dt;
            }
        } else if (normalSlip) {
            const float speedFactor = std::clamp(std::abs(v_f) / normalVisRefSpeed_, 0.f, 1.5f);
            const float soften = 1.f + normalSlipGripFactor_ * speedFactor; // reduces grip slightly with speed
            grip = lateralGripNormal_ / soften;

            const float signTurn = (turnVel_>=0.f)? 1.f : -1.f;
            const float kick = normalSlipKick_ * std::abs(turnVel_) * (0.5f + 0.5f*speedFactor);
            v_r += signTurn * kick * dt;
        }

        if (reversing && !(input_->left || input_->right)) grip = lateralGripOnlyReverse_;

        //stabilization during braking phase (prevents spin on S)
        if (brakingPhase) {
            grip = std::max(grip, brakeGripStrong_);
            turnVel_ *= (1.f - std::min(1.f, brakeYawDamp_*dt));
        }

        // lateral relaxation toward 0
        const float latAlpha = 1.f - std::exp(-grip * dt);
        v_r += (0.f - v_r) * latAlpha;

        // mild air drag
        const float air = std::max(0.f, 1.f - airDrag_ * dt);
        v_f *= air; v_r *= air;

        velX_ = fwdX * v_f + rightX * v_r;
        velZ_ = fwdZ * v_f + rightZ * v_r;
        carRoot_->position.x += velX_ * dt;
        carRoot_->position.z += velZ_ * dt;

        // orientation
        const float speed2 = velX_*velX_ + velZ_*velZ_;
        const bool moving = (speed2 > 1e-6f);
        const bool reversingNoTurn = reversing && !(input_->left || input_->right);

        const bool freezeAlign = !moving || reversingNoTurn;
        const float desiredYaw = moving ? std::atan2(velX_, velZ_) : yaw_;
        const float alignRate = (reversing ? 0.f : (allowDrift ? yawAlignDrift_ : yawAlignNormal_));
        const float alignAlpha = freezeAlign ? 0.f : (1.f - std::exp(-alignRate * dt));

        yaw_ += turnVel_ * dt;
        if (!freezeAlign){
            float d = desiredYaw - yaw_;
            while (d >  math::PI) d -= 2.f * math::PI;
            while (d < -math::PI) d += 2.f * math::PI;
            yaw_ += d * alignAlpha;
        }
        carRoot_->rotation.y = yaw_;

        // visual slip
        const float safeVF = (std::abs(v_f)>1e-3f) ? v_f : (v_f>=0.f? 1e-3f : -1e-3f);
        float slipAngle = 0.f;
        float visYawFactor = 0.f;
        float visOffsetScale = 0.f;

        if (allowDrift) {
            slipAngle = std::atan2(v_r, safeVF);
            const float visBoost = 1.f + slipVisSpeedGain_ * std::min(std::abs(v_f)/visRefSpeed_, visBoostCap_);
            slipAngle *= visBoost;
            visYawFactor   = visualSlipYawFactor_;
            visOffsetScale = visualSlipOffset_;
        } else if (normalSlip) {
            // normal-turn visual slip
            slipAngle = std::atan2(v_r, safeVF);
            const float visBoost = 1.f + normalSlipVisSpeedGain_ * std::min(std::abs(v_f)/normalVisRefSpeed_, normalVisBoostCap_);
            slipAngle *= visBoost;
            visYawFactor   = visualSlipYawFactorNormal_;
            visOffsetScale = visualSlipOffsetNormal_;
        }

        const float targetChassisYaw = visYawFactor * slipAngle;
        const float yawVisAlpha      = 1.f - std::exp(-visualYawFollow_ * dt);
        chassisYawVis_ += (targetChassisYaw - chassisYawVis_) * yawVisAlpha;
        chassis_->rotation.y = chassisYawVis_;

        const float targetOffset = -visOffsetScale * std::clamp(v_r/10.f, -1.f, 1.f);
        const float offVisAlpha  = 1.f - std::exp(-visualOffsetFollow_ * dt);
        chassisSideOffset_ += (targetOffset - chassisSideOffset_) * offVisAlpha;
        chassis_->position.set(chassisSideOffset_, 0.f, 0.f);

        // wheels
        constexpr float wheelRadius = 0.35f;
        if (wheelRadius > 0.f){
            const float ang = (v_f / wheelRadius) * dt;
            for (auto* w: wheels_) w->rotation.x += ang;
        }

        // power-ups
        if (powerups_) powerups_->update(*this, carRoot_);

        // camera
        if (input_->view && !prevViewKey_) sideView_ = !sideView_;
        prevViewKey_ = input_->view;

        if (sideView_) {
            placeCameraSideRight(*camera_, carRoot_->position, yaw_);
        } else if (reversing) {
            placeCameraFront(*camera_, carRoot_->position, yaw_);
        } else {
            placeCameraThirdPerson(*camera_, carRoot_->position, yaw_);
        }

        renderer_->render(*scene_, *camera_);
    }

    // power-up effects
    void growCar(float f){ carRoot_->scale.multiplyScalar(f); }
    void makeFaster(float sf, float af){ maxForward_*=sf; accelForward_*=af; }

private:
    Canvas* canvas_; GLRenderer* renderer_; Scene* scene_; PerspectiveCamera* camera_;
    Object3D* carRoot_; Object3D* chassis_; DriveInput* input_; PowerUpManager* powerups_;

    // timing
    Clock clock_; double lastTime_{0.0};

    // velocity (world)
    float velX_{0.f}, velZ_{0.f};

    // heading
    float yaw_{0.f}, turnVel_{0.f};

    // steering dynamics
    float maxTurnRate_{1.05f};
    float turnAccel_{3.8f};
    float turnFriction_{5.2f};

    // reverse steering softeners
    float reverseTurnScale_{0.30f};
    float reverseTurnAccel_{2.0f};
    float reverseTurnFriction_{6.5f};
    float reverseTurnVelClamp_{0.30f};

    // speed tunables
    float accelForward_{20.f};
    float accelBackward_{18.f};
    float brakeDecel_{30.f};
    float drag_{2.5f};
    float maxForward_{30.f};
    float maxBackward_{-25.f};

    // drift/lateral
    float airDrag_{0.05f};
    float lateralGripNormal_{12.f};
    float lateralGripDrift_{1.5f};
    float lateralGripOnlyReverse_{24.f};
    float driftLatAccel_{80.f};
    float driftKickMinSpeed_{2.0f};
    float driftGripSpeedFactor_{0.08f};
    float minDriftGrip_{0.8f};


    float normalSlipGripFactor_{0.45f};
    float normalSlipKick_{18.f};
    float normalVisRefSpeed_{15.f};

    float brakeGripStrong_{40.f};
    float brakeYawDamp_{8.f};

    float yawAlignNormal_{6.0f};
    float yawAlignDrift_{1.0f};

    float visualSlipYawFactor_{0.6f};
    float visualYawFollow_{10.f};
    float visualSlipOffset_{0.6f};
    float visualOffsetFollow_{12.f};
    float slipVisSpeedGain_{0.8f};
    float offsetVisSpeedGain_{0.8f};
    float visRefSpeed_{15.f};
    float visBoostCap_{1.5f};

    float normalSlipVisSpeedGain_{0.55f};
    float visualSlipYawFactorNormal_{0.35f};
    float visualSlipOffsetNormal_{0.35f};
    float normalVisBoostCap_{1.25f};

    float chassisYawVis_{0.f};
    float chassisSideOffset_{0.f};

    bool sideView_{false}, prevViewKey_{false};
    std::vector<Object3D*> wheels_;
};

// power-up behavior
void PowerUp::apply(Game& game){
    switch (type_){
        case Type::Grow:   game.growCar(1.4f);           break;
        case Type::Faster: game.makeFaster(1.35f,1.3f);  break;
        case Type::Shrink: game.growCar(0.7f);           break;
    }
}

// ------------ main ------------
int main() {
    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());

    Scene scene; scene.background = Color::skyblue;
    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    auto planeGeo = PlaneGeometry::create(2000, 2000);
    auto planeMat = MeshPhongMaterial::create(); planeMat->color = Color::forestgreen;
    auto plane = Mesh::create(planeGeo, planeMat); plane->rotation.x = -math::PI/2.f; scene.add(plane);

    auto grid = GridHelper::create(2000, 2000, Color::black, Color::darkgray);
    grid->position.y = 0.01f; scene.add(grid);

    auto hemi = HemisphereLight::create(Color::white, Color::gray, 1.0f); scene.add(hemi);
    auto dir  = DirectionalLight::create(Color::white, 0.8f); dir->position.set(5,10,7); scene.add(dir);

    AssimpLoader assimp;

    auto model = assimp.load("suv_model.glb");
    auto carRoot = Object3D::create();
    auto chassis = Object3D::create();
    chassis->add(model);
    scene.add(carRoot);
    carRoot->add(chassis);
    carRoot->position.y = 0.f;

    // power-ups
    PowerUpManager puManager;
    std::vector<std::string> puFiles = {"power_up1.glb","power_up2.glb","power_up3.glb"};
    std::mt19937 rng(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<float> distXY(-950.f, 950.f);
    std::uniform_int_distribution<int> distType(0, 2);

    for (int i=0;i<120;++i){
        int t = distType(rng);
        auto obj = assimp.load(puFiles[t]);
        obj->position.set(distXY(rng), 0.f, distXY(rng));
        scene.add(obj);
        puManager.add(std::move(obj), t==0?PowerUp::Type::Grow: t==1?PowerUp::Type::Faster:PowerUp::Type::Shrink);
    }

    DriveInput input; canvas.addKeyListener(input);
    Game game(&canvas, &renderer, &scene, &camera, carRoot.get(), chassis.get(), &input, &puManager);

    WindowSize last = canvas.size();
    canvas.animate([&]{
        auto s = canvas.size();
        if (s.width()!=last.width() || s.height()!=last.height()){
            renderer.setSize(s);
            camera.aspect = static_cast<float>(s.width())/static_cast<float>(s.height());
            last = s;
        }
        game.update();
    });

    return 0;
}