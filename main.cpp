#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

#include <chrono>
#include <random>
#include <memory>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

using namespace threepp;

// =====================================================
// Input
// =====================================================

class InputState : public KeyListener {
public:
    bool forward{false}, backward{false}, left{false}, right{false}, drift{false};
    bool enter{false}, escape{false}, reset{false};
    bool camToggle{false};              // V
    bool select1{false}, select2{false}, select3{false};

    void onKeyPressed(KeyEvent e) override {
        switch (e.key) {
            case Key::W: forward = true; break;
            case Key::S: backward = true; break;
            case Key::A: left = true; break;
            case Key::D: right = true; break;
            case Key::SPACE: drift = true; break;

            case Key::V: camToggle = true; break;

            case Key::ENTER: enter = true; break;
            case Key::ESCAPE: escape = true; break;
            case Key::R: reset = true; break;

            case Key::NUM_1: select1 = true; break;
            case Key::NUM_2: select2 = true; break;
            case Key::NUM_3: select3 = true; break;
            default: break;
        }
    }

    void onKeyReleased(KeyEvent e) override {
        switch (e.key) {
            case Key::W: forward = false; break;
            case Key::S: backward = false; break;
            case Key::A: left = false; break;
            case Key::D: right = false; break;
            case Key::SPACE: drift = false; break;

            case Key::V: camToggle = false; break;

            case Key::ENTER: enter = false; break;
            case Key::ESCAPE: escape = false; break;
            case Key::R: reset = false; break;

            case Key::NUM_1: select1 = false; break;
            case Key::NUM_2: select2 = false; break;
            case Key::NUM_3: select3 = false; break;
            default: break;
        }
    }
};

// =====================================================
/* Camera controller: owns camera mode + placement.
   Modes: Behind -> Side -> Front -> Behind           */
// =====================================================

enum class CameraMode { Behind, Side, Front };

class CameraController {
public:
    void attach(PerspectiveCamera* cam) { camera_ = cam; }
    void setMode(CameraMode m) { mode_ = m; }
    CameraMode mode() const { return mode_; }

    // call when V is *just* pressed
    void cycle() {
        switch (mode_) {
            case CameraMode::Behind: mode_ = CameraMode::Side;  break;
            case CameraMode::Side:   mode_ = CameraMode::Front; break;
            case CameraMode::Front:  mode_ = CameraMode::Behind; break;
        }
    }

    // place camera each frame based on mode
    void update(const Vector3& carPos, float yaw) {
        if (!camera_) return;
        switch (mode_) {
            case CameraMode::Behind: placeBehind(carPos, yaw); break;
            case CameraMode::Side:   placeSide(carPos, yaw);   break;
            case CameraMode::Front:  placeFront(carPos, yaw);  break;
        }
    }

private:
    // simple placements (kept from your helpers)
    void placeBehind(const Vector3& p, float yaw, float back=6.f, float h=3.f) {
        camera_->position.set(p.x-std::sin(yaw)*back, p.y+h, p.z-std::cos(yaw)*back);
        camera_->lookAt(p);
    }
    void placeFront(const Vector3& p, float yaw, float front=6.f, float h=3.f) {
        camera_->position.set(p.x+std::sin(yaw)*front, p.y+h, p.z+std::cos(yaw)*front);
        camera_->lookAt(p);
    }
    void placeSide(const Vector3& p, float yaw, float side=5.f, float h=3.f) {
        float sy = yaw - math::PI/2.f;
        camera_->position.set(p.x-std::sin(sy)*side, p.y+h, p.z-std::cos(sy)*side);
        camera_->lookAt(p);
    }

    PerspectiveCamera* camera_{nullptr};
    CameraMode mode_{CameraMode::Behind};
};

// =====================================================
// Utilities
// =====================================================

static void findWheels(Object3D* root, std::vector<Object3D*>& out){
    const char* names[]={"wheel_FL","wheel_FR","wheel_BL","wheel_BR"};
    root->traverse([&](Object3D& o){ for(auto n:names) if(o.name==n){ out.push_back(&o); break; }});
}

// Pose snapshot for visuals
struct CarPose {
    Vector3 position{};
    float yaw{0};
    float forwardVel{0}, lateralVel{0};
    float yawRate{0};
    bool reversing{false};
    bool drifting{false};
    bool slipping{false};
    float wheelSpinDelta{0};
};

// Forward declare Game
class Game;

// =====================================================
// Trees (simple collision radius)
// =====================================================

class Trees {
public:
    void add(std::shared_ptr<Object3D> t){ trees_.push_back(std::move(t)); }
    void update(Game& g, Object3D* car, float currentSpeed);
private:
    std::vector<std::shared_ptr<Object3D>> trees_;
    float collisionRadius{1.8f};
};

// =====================================================
// Vehicle physics (renamed tunables for clarity)
// =====================================================

class VehiclePhysics {
public:
    // Speed/accel (per-vehicle settable)
    float accelForward{20.f}, accelBackward{18.f};
    float brakeForce{30.f}, coastDrag{2.5f};
    float maxForward{30.f}, maxBackward{-25.f};

    // Steering
    float maxSteer{1.05f}, steerAccel{3.8f}, steerFriction{5.2f};
    float reverseSteerScale{0.30f}, reverseSteerAccel{2.0f}, reverseSteerFriction{6.5f}, reverseSteerClamp{0.30f};
    float reverseGrip{24.f};

    // Grip & drift
    float gripNormal{12.f}, gripDriftBase{1.5f}, driftKick{80.f}, driftMinVel{2.f}, driftGripSpeed{0.08f}, driftGripMin{0.8f};
    float normalSlipFactor{0.45f}, normalKick{18.f};

    // Misc
    float airDrag{0.05f}, wheelRadius{0.35f};

    CarPose step(const InputState& input, float dt){
        CarPose pose{};

        // basis
        float fx=std::sin(yaw_), fz=std::cos(yaw_), rx=std::cos(yaw_), rz=-std::sin(yaw_);
        float forwardV = velX_*fx + velZ_*fz;
        float lateralV = velX_*rx + velZ_*rz;

        // steering target
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

        // throttle / brake
        bool braking = input.backward && (forwardV>0.05f);
        if (input.forward) forwardV += accelForward*dt;
        if (input.backward){
            if (forwardV>0){ forwardV -= brakeForce*dt; if (forwardV<0) forwardV=0; }
            else { forwardV -= accelBackward*dt; }
        }

        // coast drag
        if (!input.forward && !input.backward){
            if (forwardV>0){ forwardV -= coastDrag*dt; if (forwardV<0) forwardV=0; }
            else if (forwardV<0){ forwardV += coastDrag*dt; if (forwardV>0) forwardV=0; }
        }

        // clamp
        forwardV = std::clamp(forwardV, maxBackward, maxForward);

        // drift & normal slip
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

        // kill lateral
        float latA = 1.f-std::exp(-grip*dt);
        lateralV += (0.f - lateralV) * latA;

        // air drag
        float air = std::max(0.f, 1.f - airDrag*dt);
        forwardV*=air; lateralV*=air;

        // compose velocity and integrate
        velX_ = fx*forwardV + rx*lateralV;
        velZ_ = fz*forwardV + rz*lateralV;
        position_.x += velX_*dt; position_.z += velZ_*dt;

        // yaw alignment
        bool moving = (velX_*velX_+velZ_*velZ_ > 1e-6f);
        bool revNoTurn = reversing && !(input.left||input.right);
        bool freezeAlign = !moving || revNoTurn;
        float desiredYaw = moving ? std::atan2(velX_, velZ_) : yaw_;
        float alignRate = (reversing? 0.f : (allowDrift? 1.0f : 6.0f));
        float alignAlpha = freezeAlign? 0.f : (1.f - std::exp(-alignRate*dt));

        yaw_ += yawRate_*dt;
        if (!freezeAlign){
            float d = desiredYaw - yaw_;
            while (d> math::PI) d -= 2*math::PI;
            while (d<-math::PI) d += 2*math::PI;
            yaw_ += d * alignAlpha;
        }

        // pose out
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

    void hardStop(){ velX_=0; velZ_=0; }
    void setPosition(const Vector3& p){ position_=p; }
    const Vector3& position() const { return position_; }
    float speed() const { return std::sqrt(velX_*velX_ + velZ_*velZ_); }

private:
    Vector3 position_{0,0,0};
    float velX_{0}, velZ_{0};
    float yaw_{0}, yawRate_{0};
};

// =====================================================
// Visuals (drift look & small crash tilt)
// =====================================================

class VehicleVisuals {
public:
    VehicleVisuals(Object3D* chassis, const std::vector<Object3D*>& wheels)
        : chassis_(chassis), wheels_(wheels) {}

    // drift look / smoothing
    float yawDriftScale=0.6f, lateralOffsetDrift=0.6f;
    float yawSlipScale =0.35f, lateralOffsetSlip =0.35f;
    float yawFollow=10.f, offsetFollow=12.f;

    void triggerCrashTilt(){ tiltTimer_ = tiltDuration_; }

    void apply(float dt, const CarPose& pose){
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
    Object3D* chassis_{};
    std::vector<Object3D*> wheels_;
    float visualYaw_{0}, visualOffset_{0};

    float tiltTimer_{0.f};
    float tiltDuration_{0.15f};
    float tiltMax_{0.10f};
};

// =====================================================
// Power-ups
// =====================================================

class PowerUp {
public:
    enum class Type { Grow, Faster, Shrink };
    PowerUp(std::shared_ptr<Object3D> obj, Type t): obj_(std::move(obj)), type_(t) {}
    Object3D* object(){ return obj_.get(); }
    bool active() const { return active_; }
    void apply(class Game& g);
    void hide(){ active_=false; if(obj_) obj_->visible=false; }
private:
    std::shared_ptr<Object3D> obj_;
    Type type_;
    bool active_{true};
};

class PowerUpManager {
public:
    void add(std::shared_ptr<Object3D> obj, PowerUp::Type t){ items_.emplace_back(std::move(obj),t); }
    void update(Game& g, Object3D* car){
        auto p=car->position;
        for(auto& pu:items_){
            if(!pu.active()) continue;
            auto* o=pu.object(); if(!o) continue;
            float dx=p.x-o->position.x, dz=p.z-o->position.z;
            if(dx*dx+dz*dz < 2.25f){ pu.apply(g); pu.hide(); }
        }
    }
private:
    std::vector<PowerUp> items_;
};

// =====================================================
// Game
// =====================================================

enum class GameState { Menu, Playing, Paused };

class Game {
public:
    Game(Canvas* canvas, GLRenderer* renderer, Scene* scene, PerspectiveCamera* camera,
         Object3D* carRoot, Object3D* chassis, const std::vector<Object3D*>& wheels,
         InputState* input, PowerUpManager* powerUps, Trees* trees)
        : canvas_(canvas), renderer_(renderer), scene_(scene), camera_(camera),
          carRoot_(carRoot), input_(input), powerUps_(powerUps), trees_(trees),
          visuals_(chassis,wheels) {

        cameraController_.attach(camera_);
        cameraController_.setMode(CameraMode::Behind);

        lastTime_ = clock_.getElapsedTime();
        lastSafePosition_ = carRoot_->position;
    }

    void updateFrame(){
        double now=clock_.getElapsedTime();
        float dt = float(now-lastTime_); lastTime_=now; if(dt>0.033f) dt=0.033f;

        lastSafePosition_ = carRoot_->position;

        CarPose pose = physics_.step(*input_, dt);

        carRoot_->position.copy(pose.position);
        carRoot_->rotation.y = pose.yaw;

        visuals_.apply(dt, pose);

        if (powerUps_) powerUps_->update(*this, carRoot_);
        if (trees_)    trees_->update(*this, carRoot_, physics_.speed());

        // camera cycle on V press edge
        if (input_->camToggle && !prevCamToggle_) cameraController_.cycle();
        prevCamToggle_ = input_->camToggle;
        cameraController_.update(carRoot_->position, pose.yaw);

        renderer_->render(*scene_, *camera_);
    }

    void reset() {
        physics_.setPosition({0,0,0});
        physics_.hardStop();
        carRoot_->position.set(0,0,0);
        carRoot_->rotation.set(0,0,0);
        carRoot_->scale.set(1,1,1);
        lastSafePosition_ = carRoot_->position;
        cameraController_.setMode(CameraMode::Behind);
        prevCamToggle_ = false;
    }

    // power-up hooks
    void growCar(float factor){ carRoot_->scale.multiplyScalar(factor); }
    void faster(float speedFactor,float accelFactor){
        physics_.maxForward  *= speedFactor;
        physics_.accelForward*= accelFactor;
    }

    // per-vehicle tuning
    void applyVehicleTuning(float accelF, float accelB, float maxF, float maxB) {
        physics_.accelForward = accelF;
        physics_.accelBackward = accelB;
        physics_.maxForward = maxF;
        physics_.maxBackward = maxB;
    }

    // collision feedback with small tilt
    void onCollision(float impactSpeed){
        const double now = clock_.getElapsedTime();
        if (impactSpeed < crashSpeedThreshold_) {
            physics_.hardStop(); physics_.setPosition(lastSafePosition_); carRoot_->position.copy(lastSafePosition_);
            return;
        }
        if (now - lastCrashTime_ < crashCooldown_) {
            physics_.hardStop(); physics_.setPosition(lastSafePosition_); carRoot_->position.copy(lastSafePosition_);
            return;
        }
        lastCrashTime_ = now;
        physics_.hardStop(); physics_.setPosition(lastSafePosition_); carRoot_->position.copy(lastSafePosition_);
        visuals_.triggerCrashTilt();
    }

    // expose physics if you need it outside
    VehiclePhysics& physics() { return physics_; }

private:
    Canvas* canvas_; GLRenderer* renderer_; Scene* scene_; PerspectiveCamera* camera_;
    Object3D* carRoot_; InputState* input_; PowerUpManager* powerUps_; Trees* trees_;

    VehiclePhysics physics_;
    VehicleVisuals visuals_;
    CameraController cameraController_;

    Clock clock_; double lastTime_{0.0};
    Vector3 lastSafePosition_;
    bool prevCamToggle_{false};

    double lastCrashTime_{-1.0};
    float crashCooldown_{0.25f};
    float crashSpeedThreshold_{2.5f};
};

// Power-up behavior
void PowerUp::apply(Game& g){
    switch(type_){
        case Type::Grow:   g.growCar(1.4f); break;
        case Type::Faster: g.faster(1.7f,2.f); break;
        case Type::Shrink: g.growCar(0.7f); break;
    }
}

// Trees collision
void Trees::update(Game& g, Object3D* car, float currentSpeed){
    auto p=car->position;
    for (auto& t: trees_){
        auto* o=t.get(); if(!o) continue;
        float dx=p.x-o->position.x, dz=p.z-o->position.z;
        if (dx*dx+dz*dz < collisionRadius*collisionRadius){
            g.onCollision(currentSpeed);
            break;
        }
    }
}

// =====================================================
// UI state
// =====================================================

struct UIState { GameState state{GameState::Menu}; int selectedIndex{0}; };

// =====================================================
// Vehicle rig + data
// =====================================================

struct VehicleRig {
    std::shared_ptr<Object3D> root;
    std::shared_ptr<Object3D> chassis;
    std::vector<Object3D*> wheels;
};

static VehicleRig buildVehicle(AssimpLoader& loader, const std::string& file, Scene& scene){
    VehicleRig rig;
    auto model = loader.load(file);
    rig.root = Object3D::create();
    rig.chassis = Object3D::create();
    rig.chassis->add(model);
    rig.root->add(rig.chassis);
    rig.root->position.y = 0.f;
    scene.add(rig.root);
    findWheels(rig.chassis.get(), rig.wheels);
    return rig;
}

class VehicleSpec {
public:
    std::string file;
    float accelForward, accelBackward;
    float maxForward, maxBackward;
};

// =====================================================
// main
// =====================================================

int main(){
    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());
    Scene scene; scene.background = Color::skyblue;
    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    // Ground with your 2048x2048 map
    const float kGroundSize = 2048.f;
    TextureLoader texLoader;
    auto drivingMap = texLoader.load("driving_map.png");
    auto groundMat = MeshPhongMaterial::create();
    groundMat->color = Color::white;
    groundMat->map = drivingMap;
    auto ground = Mesh::create(PlaneGeometry::create(kGroundSize, kGroundSize), groundMat);
    ground->rotation.x = -math::PI/2.f;
    scene.add(ground);

    // Lights
    scene.add(HemisphereLight::create(Color::white, Color::gray,1.0f));
    auto sun=DirectionalLight::create(Color::white,0.8f); sun->position.set(5,10,7); scene.add(sun);

    // UI scene (start/pause screen)
    Scene uiScene;
    auto uiCam = OrthographicCamera::create(-1, 1, 1, -1, 0.0f, 10.0f);
    auto startTex = TextureLoader().load("startscreen_test.png");
    auto uiMat  = MeshBasicMaterial::create(); uiMat->map=startTex; uiMat->transparent=true;
    auto uiQuad = Mesh::create(PlaneGeometry::create(2,2), uiMat);
    uiQuad->position.z = -1.f; uiScene.add(uiQuad);

    AssimpLoader loader;

    // Vehicle specs (tractor slowest, sedan fastest)
    std::vector<VehicleSpec> vehicles = {
        {"suv_model.glb",     20.f, 18.f, 30.f, -25.f},
        {"sedan_model.glb",   24.f, 20.f, 36.f, -28.f},
        {"tractor_model.glb", 12.f, 10.f, 18.f, -14.f}
    };

    int currentVehicleIndex = 0;
    VehicleRig vehicleRig = buildVehicle(loader, vehicles[currentVehicleIndex].file, scene);

    // Power-ups
    PowerUpManager powerUps;
    std::vector<std::string> powerUpFiles={"power_up1.glb","power_up2.glb","power_up3.glb"};
    std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> randPos(-(kGroundSize/2.f-50.f),(kGroundSize/2.f-50.f));
    std::uniform_int_distribution<int> randType(0,2);
    for(int i=0;i<120;i++){
        int t=randType(rng);
        auto obj=loader.load(powerUpFiles[t]);
        obj->position.set(randPos(rng),0,randPos(rng));
        scene.add(obj);
        PowerUp::Type ty = (t==0)?PowerUp::Type::Grow:(t==1)?PowerUp::Type::Faster:PowerUp::Type::Shrink;
        powerUps.add(std::move(obj), ty);
    }

    // Trees
    Trees trees;
    for(int i=0;i<150;i++){
        auto t=loader.load("tree_model.glb");
        t->position.set(randPos(rng),0,randPos(rng));
        scene.add(t);
        trees.add(std::move(t));
    }

    // Input + game
    InputState input; canvas.addKeyListener(input);
    std::unique_ptr<Game> game = std::make_unique<Game>(
        &canvas, &renderer, &scene, &camera,
        vehicleRig.root.get(), vehicleRig.chassis.get(), vehicleRig.wheels,
        &input, &powerUps, &trees);

    // Apply initial vehicle tuning
    {
        const auto& v = vehicles[currentVehicleIndex];
        game->applyVehicleTuning(v.accelForward, v.accelBackward, v.maxForward, v.maxBackward);
    }

    UIState ui;
    WindowSize lastSize=canvas.size();

    auto swapVehicle = [&](int newIndex){
        if (newIndex < 0 || newIndex >= (int)vehicles.size()) return;
        if (newIndex == currentVehicleIndex) return;

        if (vehicleRig.root) scene.remove(*vehicleRig.root);
        vehicleRig = buildVehicle(loader, vehicles[newIndex].file, scene);
        currentVehicleIndex = newIndex;

        game = std::make_unique<Game>(&canvas,&renderer,&scene,&camera,
                                      vehicleRig.root.get(), vehicleRig.chassis.get(), vehicleRig.wheels,
                                      &input, &powerUps, &trees);

        const auto& v = vehicles[currentVehicleIndex];
        game->applyVehicleTuning(v.accelForward, v.accelBackward, v.maxForward, v.maxBackward);
    };

    canvas.animate([&]{
        auto s=canvas.size();
        if(s.width()!=lastSize.width()||s.height()!=lastSize.height()){
            renderer.setSize(s);
            camera.aspect=float(s.width())/float(s.height());
            lastSize=s;
        }

        // vehicle selection (works anytime)
        if (input.select1) ui.selectedIndex = 0, swapVehicle(0);
        if (input.select2) ui.selectedIndex = 1, swapVehicle(1);
        if (input.select3) ui.selectedIndex = 2, swapVehicle(2);

        switch (ui.state) {
            case GameState::Menu: {
                renderer.render(uiScene, *uiCam);
                if (input.enter) { swapVehicle(ui.selectedIndex); ui.state = GameState::Playing; }
                break;
            }
            case GameState::Playing: {
                if (input.escape) { ui.state = GameState::Paused; break; }
                if (input.reset)  { game->reset(); }
                game->updateFrame();
                break;
            }
            case GameState::Paused: {
                renderer.render(uiScene, *uiCam);
                if (input.enter) ui.state = GameState::Playing;
                if (input.reset) { game->reset(); ui.state = GameState::Menu; }
                break;
            }
        }
    });

    return 0;
}