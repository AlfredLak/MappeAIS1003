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

//========================= Input =========================

class InputController : public KeyListener {
public:
    // driving
    bool forward{false}, backward{false}, steerLeft{false}, steerRight{false};
    bool driftKey{false}, sideViewToggle{false};
    // ui
    bool keyEnter{false}, keyEsc{false}, keyReset{false};
    // vehicle select
    bool choose1{false}, choose2{false}, choose3{false};

    void onKeyPressed(KeyEvent e) override {
        switch (e.key) {
            case Key::W: forward = true; break;
            case Key::S: backward = true; break;
            case Key::A: steerLeft = true; break;
            case Key::D: steerRight = true; break;
            case Key::SPACE: driftKey = true; break;
            case Key::V: sideViewToggle = true; break;

            case Key::ENTER: keyEnter = true; break;
            case Key::ESCAPE: keyEsc = true; break;
            case Key::R: keyReset = true; break;

            case Key::NUM_1: choose1 = true; break;
            case Key::NUM_2: choose2 = true; break;
            case Key::NUM_3: choose3 = true; break;

            default: break;
        }
    }

    void onKeyReleased(KeyEvent e) override {
        switch (e.key) {
            case Key::W: forward = false; break;
            case Key::S: backward = false; break;
            case Key::A: steerLeft = false; break;
            case Key::D: steerRight = false; break;
            case Key::SPACE: driftKey = false; break;
            case Key::V: sideViewToggle = false; break;

            case Key::ENTER: keyEnter = false; break;
            case Key::ESCAPE: keyEsc = false; break;
            case Key::R: keyReset = false; break;

            case Key::NUM_1: choose1 = false; break;
            case Key::NUM_2: choose2 = false; break;
            case Key::NUM_3: choose3 = false; break;

            default: break;
        }
    }
};

//========================= Camera helpers =========================

static void cameraBehind(PerspectiveCamera& cam, const Vector3& carPos, float carYaw, float distance=6.f, float height=3.f) {
    cam.position.set(carPos.x - std::sin(carYaw) * distance, carPos.y + height, carPos.z - std::cos(carYaw) * distance);
    cam.lookAt(carPos);
}

static void cameraFront(PerspectiveCamera& cam, const Vector3& carPos, float carYaw, float distance=6.f, float height=3.f) {
    cam.position.set(carPos.x + std::sin(carYaw) * distance, carPos.y + height, carPos.z + std::cos(carYaw) * distance);
    cam.lookAt(carPos);
}

static void cameraRightSide(PerspectiveCamera& cam, const Vector3& carPos, float carYaw, float side=5.f, float height=3.f) {
    const float sideYaw = carYaw - math::PI/2.f;
    cam.position.set(carPos.x - std::sin(sideYaw) * side, carPos.y + height, carPos.z - std::cos(sideYaw) * side);
    cam.lookAt(carPos);
}

//========================= Wheel discovery =========================

static void findWheels(Object3D* root, std::vector<Object3D*>& outWheels) {
    const char* names[] = {"wheel_FL", "wheel_FR", "wheel_BL", "wheel_BR"};
    root->traverse([&](Object3D& node){
        for (auto n : names) {
            if (node.name == n) { outWheels.push_back(&node); break; }
        }
    });
}

//========================= Vehicle state (Pose) =========================

struct VehiclePose {
    Vector3 position{};
    float yaw{0.f};
    float forwardVel{0.f};        // velocity in forward axis
    float lateralVel{0.f};        // velocity in right axis
    float yawRate{0.f};
    bool reversing{false};
    bool drifting{false};
    bool normalSlip{false};
    float wheelSpinDelta{0.f};
};

// forward declarations
class Game;

//========================= World objects: Trees =========================

class TreeManager {
public:
    void add(std::shared_ptr<Object3D> tree) { trees_.push_back(std::move(tree)); }

    void update(Game& game, Object3D* carNode, float carSpeed);

private:
    std::vector<std::shared_ptr<Object3D>> trees_;
    float collisionRadius_{1.8f};
};

//========================= Vehicle Physics =========================
// (Encapsulates kinematics and handling. No rendering here.)

class VehiclePhysics {
public:
    // speed feel
    float accelForward{20.f}, accelBackward{18.f}, brakeDecel{30.f}, coastDrag{2.5f};
    float maxForward{30.f}, maxBackward{-25.f};
    // steering
    float maxTurn{1.05f}, turnAccel{3.8f}, turnFriction{5.2f};
    // reverse softness
    float reverseTurnScale{0.30f}, reverseTurnAccel{2.0f}, reverseTurnFriction{6.5f}, reverseTurnClamp{0.30f}, reverseGrip{24.f};
    // grip & drift
    float baseGrip{12.f}, driftGripBase{1.5f}, driftMinSpeed{2.f}, driftGripSpeed{0.08f}, driftGripMin{0.8f};
    float driftKick{80.f};                    // lateral push when drifting
    float normalSlipFactor{0.45f}, normalSlipKick{18.f};
    // misc
    float airDragFactor{0.05f};
    float wheelRadius{0.35f};

    VehiclePose step(const InputController& input, float dt) {
        VehiclePose out;

        // local axes
        const float fx = std::sin(yaw_), fz = std::cos(yaw_);
        const float rx = std::cos(yaw_), rz = -std::sin(yaw_);

        float forwardV = velX_ * fx + velZ_ * fz;
        float lateralV = velX_ * rx + velZ_ * rz;

        // steering target
        const float steerInput = (input.steerLeft ? +maxTurn : 0.f) + (input.steerRight ? -maxTurn : 0.f);
        const bool onlyReverseKey = input.backward && !input.forward && !input.steerLeft && !input.steerRight;
        const bool isReversing = (forwardV < -0.05f);

        float targetTurn = isReversing ? (reverseTurnScale * steerInput) : steerInput;
        if (onlyReverseKey) targetTurn = 0.f;

        const float steerResponse = (std::abs(targetTurn) > 0.f)
                                    ? (isReversing ? reverseTurnAccel : turnAccel)
                                    : (isReversing ? reverseTurnFriction : turnFriction);
        const float steerAlpha = 1.f - std::exp(-steerResponse * dt);
        yawRate_ += (targetTurn - yawRate_) * steerAlpha;
        if (isReversing) {
            yawRate_ = std::clamp(yawRate_, -reverseTurnClamp, reverseTurnClamp);
        }

        // throttle/brake/coast
        const bool braking = input.backward && (forwardV > 0.05f);
        if (input.forward) forwardV += accelForward * dt;
        if (input.backward) {
            if (forwardV > 0) { forwardV -= brakeDecel * dt; forwardV = std::max(0.f, forwardV); }
            else { forwardV -= accelBackward * dt; }
        }
        if (!input.forward && !input.backward) {
            if (forwardV > 0) { forwardV -= coastDrag * dt; forwardV = std::max(0.f, forwardV); }
            else if (forwardV < 0) { forwardV += coastDrag * dt; forwardV = std::min(0.f, forwardV); }
        }
        forwardV = std::clamp(forwardV, maxBackward, maxForward);

        // slip/drift model
        const bool allowDrift = input.driftKey && (forwardV > 0.05f) && !onlyReverseKey;
        const bool normalSlip = !allowDrift && (forwardV > 0.05f) && (std::abs(yawRate_) > 1e-4f) && !isReversing;

        float grip = baseGrip;
        if (allowDrift) {
            const float speedFactor = std::clamp(std::abs(forwardV) * driftGripSpeed, 0.f, 3.f);
            grip = std::max(driftGripMin, driftGripBase / (1.f + speedFactor));
            const float sgn = (yawRate_ >= 0) ? 1.f : -1.f;
            if (forwardV > driftMinSpeed) {
                const float gain = 0.3f + 0.7f * std::min(std::abs(forwardV) / 20.f, 1.f);
                lateralV += sgn * driftKick * std::abs(yawRate_) * gain * dt;
            }
        } else if (normalSlip) {
            const float sp = std::clamp(std::abs(forwardV) / 15.f, 0.f, 1.5f);
            grip = baseGrip / (1.f + normalSlipFactor * sp);
            const float sgn = (yawRate_ >= 0) ? 1.f : -1.f;
            lateralV += sgn * normalSlipKick * std::abs(yawRate_) * (0.5f + 0.5f * sp) * dt;
        }

        if (isReversing && !(input.steerLeft || input.steerRight)) grip = reverseGrip;
        if (braking) { grip = std::max(grip, 40.f); yawRate_ *= (1.f - std::min(1.f, 8.f * dt)); }

        const float lateralBlend = 1.f - std::exp(-grip * dt);
        lateralV += (0.f - lateralV) * lateralBlend;

        const float air = std::max(0.f, 1.f - airDragFactor * dt);
        forwardV *= air; lateralV *= air;

        velX_ = fx * forwardV + rx * lateralV;
        velZ_ = fz * forwardV + rz * lateralV;
        position_.x += velX_ * dt;
        position_.z += velZ_ * dt;

        const bool moving = (velX_ * velX_ + velZ_ * velZ_ > 1e-6f);
        const bool reversingNoSteer = isReversing && !(input.steerLeft || input.steerRight);
        const bool freezeAlign = !moving || reversingNoSteer;
        const float desiredYaw = moving ? std::atan2(velX_, velZ_) : yaw_;
        const float alignRate = (isReversing ? 0.f : (allowDrift ? 1.0f : 6.0f));
        const float alignAlpha = freezeAlign ? 0.f : (1.f - std::exp(-alignRate * dt));

        yaw_ += yawRate_ * dt;
        if (!freezeAlign) {
            float d = desiredYaw - yaw_;
            while (d >  math::PI) d -= 2 * math::PI;
            while (d < -math::PI) d += 2 * math::PI;
            yaw_ += d * alignAlpha;
        }

        out.position = position_;
        out.yaw = yaw_;
        out.forwardVel = forwardV;
        out.lateralVel = lateralV;
        out.yawRate = yawRate_;
        out.reversing = isReversing;
        out.drifting = allowDrift;
        out.normalSlip = normalSlip;
        out.wheelSpinDelta = (wheelRadius > 0.f) ? (forwardV / wheelRadius) * dt : 0.f;

        return out;
    }

    // controls from game/power-ups
    void setPosition(const Vector3& p) { position_ = p; }
    void fullStop() { velX_ = 0.f; velZ_ = 0.f; }
    void boostTopSpeed(float factor) { maxForward *= factor; }
    void boostAcceleration(float factor) { accelForward *= factor; }
    float currentSpeed() const { return std::sqrt(velX_ * velX_ + velZ_ * velZ_); }

private:
    Vector3 position_{0,0,0};
    float velX_{0.f}, velZ_{0.f};
    float yaw_{0.f};
    float yawRate_{0.f};
};

//========================= Vehicle Visuals =========================
// (Encapsulates only presentation of chassis & wheels in response to pose.)

class VehicleVisuals {
public:
    VehicleVisuals(Object3D* chassisNode, const std::vector<Object3D*>& wheelNodes)
        : chassis_(chassisNode), wheels_(wheelNodes) {}

    // tuning for visual response
    float visYawDrift{0.6f}, visOffsetDrift{0.6f};
    float visYawNormal{0.35f}, visOffsetNormal{0.35f};
    float followYawRate{10.f}, followOffsetRate{12.f};
    // crash tilt
    void triggerCrashTilt() { tiltTimer_ = tiltDuration_; }

    void apply(float dt, const VehiclePose& pose) {
        for (auto* w : wheels_) w->rotation.x += pose.wheelSpinDelta;

        float targetYawVis = 0.f, targetXOffset = 0.f;
        if (!pose.reversing) {
            const float safeV = (std::abs(pose.forwardVel) > 1e-3f) ? pose.forwardVel : (pose.forwardVel >= 0 ? 1e-3f : -1e-3f);
            float slipAngle = std::atan2(pose.lateralVel, safeV);
            if (pose.drifting) {
                const float boost = 1.f + 0.8f * std::min(std::abs(pose.forwardVel) / 15.f, 1.5f);
                slipAngle *= boost;
                targetYawVis  =  visYawDrift * slipAngle;
                targetXOffset = -visOffsetDrift * std::clamp(pose.lateralVel / 10.f, -1.f, 1.f);
            } else if (pose.normalSlip) {
                const float boost = 1.f + 0.55f * std::min(std::abs(pose.forwardVel) / 15.f, 1.25f);
                slipAngle *= boost;
                targetYawVis  =  visYawNormal * slipAngle;
                targetXOffset = -visOffsetNormal * std::clamp(pose.lateralVel / 10.f, -1.f, 1.f);
            }
        }

        const float yawAlpha = 1.f - std::exp(-followYawRate * dt);
        yawVisual_ += (targetYawVis - yawVisual_) * yawAlpha;
        chassis_->rotation.y = yawVisual_;

        const float offAlpha = 1.f - std::exp(-followOffsetRate * dt);
        xOffset_ += (targetXOffset - xOffset_) * offAlpha;
        chassis_->position.set(xOffset_, 0, 0);

        if (tiltTimer_ > 0.f) {
            tiltTimer_ -= dt;
            const float t = std::max(0.f, tiltTimer_ / tiltDuration_);
            const float angle = tiltMax_ * t * (2.f - t);
            chassis_->rotation.x = angle;
        } else {
            chassis_->rotation.x *= std::exp(-10.f * dt);
        }
    }

private:
    Object3D* chassis_{};
    std::vector<Object3D*> wheels_;
    float yawVisual_{0.f};
    float xOffset_{0.f};

    // crash tilt state
    float tiltTimer_{0.f};
    float tiltDuration_{0.15f};
    float tiltMax_{0.10f};
};

//========================= Power-ups =========================

class PowerUp {
public:
    enum class Kind { Grow, Faster, Shrink };

    PowerUp(std::shared_ptr<Object3D> object, Kind kind) : object_(std::move(object)), kind_(kind) {}

    Object3D* node() { return object_.get(); }
    bool active() const { return active_; }

    void apply(Game& game);
    void deactivate() { active_ = false; if (object_) object_->visible = false; }

private:
    std::shared_ptr<Object3D> object_;
    Kind kind_;
    bool active_{true};
};

class PowerUpManager {
public:
    void add(std::shared_ptr<Object3D> node, PowerUp::Kind kind) {
        items_.emplace_back(std::move(node), kind);
    }

    void update(Game& game, Object3D* carNode) {
        const auto& p = carNode->position;
        for (auto& pu : items_) {
            if (!pu.active()) continue;
            auto* obj = pu.node(); if (!obj) continue;
            const float dx = p.x - obj->position.x;
            const float dz = p.z - obj->position.z;
            if (dx*dx + dz*dz < 2.25f) { // ~1.5m radius
                pu.apply(game);
                pu.deactivate();
            }
        }
    }

private:
    std::vector<PowerUp> items_;
};

//========================= Game (composition root) =========================

class Game {
public:
    Game(Canvas& canvas,
         GLRenderer& renderer,
         Scene& scene,
         PerspectiveCamera& camera,
         Object3D& vehicleRoot,
         Object3D& chassisNode,
         const std::vector<Object3D*>& wheelNodes,
         InputController& input,
         PowerUpManager& powerUps,
         TreeManager& trees)
        : canvas_(canvas), renderer_(renderer), scene_(scene), camera_(camera),
          vehicleRoot_(&vehicleRoot), input_(&input), powerUps_(&powerUps), trees_(&trees),
          visuals_(&chassisNode, wheelNodes) {
        cameraBehind(camera_, vehicleRoot_->position, 0.f);
        lastFrameTime_ = clock_.getElapsedTime();
        lastSafePosition_ = vehicleRoot_->position;
    }

    void updateFrame() {
        const double now = clock_.getElapsedTime();
        float dt = static_cast<float>(now - lastFrameTime_);
        lastFrameTime_ = now;
        if (dt > 0.033f) dt = 0.033f; // clamp

        lastSafePosition_ = vehicleRoot_->position;

        const VehiclePose pose = physics_.step(*input_, dt);

        vehicleRoot_->position.copy(pose.position);
        vehicleRoot_->rotation.y = pose.yaw;

        visuals_.apply(dt, pose);

        if (powerUps_) powerUps_->update(*this, vehicleRoot_);
        if (trees_)    trees_->update(*this, vehicleRoot_, physics_.currentSpeed());

        if (input_->sideViewToggle && !sideViewPressedLastFrame_) sideViewEnabled_ = !sideViewEnabled_;
        sideViewPressedLastFrame_ = input_->sideViewToggle;

        if (sideViewEnabled_)              cameraRightSide(camera_, vehicleRoot_->position, pose.yaw);
        else if (pose.reversing)           cameraFront(camera_,      vehicleRoot_->position, pose.yaw);
        else                               cameraBehind(camera_,     vehicleRoot_->position, pose.yaw);

        renderer_.render(scene_, camera_);
    }

    // lifecycle
    void resetWorld() {
        physics_.setPosition({0,0,0});
        physics_.fullStop();
        vehicleRoot_->position.set(0,0,0);
        vehicleRoot_->rotation.set(0,0,0);
        vehicleRoot_->scale.set(1,1,1);
        lastSafePosition_ = vehicleRoot_->position;
    }

    // power-up hooks
    void makeBigger(float scaleFactor) { vehicleRoot_->scale.multiplyScalar(scaleFactor); }
    void makeFaster(float topSpeedFactor, float accelFactor) { physics_.boostTopSpeed(topSpeedFactor); physics_.boostAcceleration(accelFactor); }
    void makeSmaller(float scaleFactor) { vehicleRoot_->scale.multiplyScalar(scaleFactor); }

    // collisions (trees)
    void onTreeCollision(float impactSpeed) {
        const double now = clock_.getElapsedTime();
        if (impactSpeed < collisionSpeedThreshold_) {
            physics_.fullStop(); physics_.setPosition(lastSafePosition_); vehicleRoot_->position.copy(lastSafePosition_);
            return;
        }
        if (now - lastCrashTime_ < collisionCooldown_) {
            physics_.fullStop(); physics_.setPosition(lastSafePosition_); vehicleRoot_->position.copy(lastSafePosition_);
            return;
        }
        lastCrashTime_ = now;
        physics_.fullStop(); physics_.setPosition(lastSafePosition_); vehicleRoot_->position.copy(lastSafePosition_);
        visuals_.triggerCrashTilt();
    }

    // accessors
    VehiclePhysics& physics() { return physics_; }

private:
    Canvas& canvas_;
    GLRenderer& renderer_;
    Scene& scene_;
    PerspectiveCamera& camera_;
    Object3D* vehicleRoot_{};
    InputController* input_{};
    PowerUpManager* powerUps_{};
    TreeManager* trees_{};
    VehiclePhysics physics_;
    VehicleVisuals visuals_;

    Clock clock_;
    double lastFrameTime_{0.0};
    bool sideViewEnabled_{false};
    bool sideViewPressedLastFrame_{false};
    Vector3 lastSafePosition_;
    double lastCrashTime_{-1.0};
    float collisionCooldown_{0.25f};
    float collisionSpeedThreshold_{2.5f};
};

// Power-up behavior (kept small and focused)
void PowerUp::apply(Game& game) {
    switch (kind_) {
        case Kind::Grow:   game.makeBigger(1.4f); break;
        case Kind::Faster: game.makeFaster(1.35f, 1.3f); break;
        case Kind::Shrink: game.makeSmaller(0.7f); break;
    }
}

// TreeManager after Game definition
void TreeManager::update(Game& game, Object3D* carNode, float currentSpeed) {
    const auto& p = carNode->position;
    for (auto& t : trees_) {
        auto* node = t.get(); if (!node) continue;
        const float dx = p.x - node->position.x;
        const float dz = p.z - node->position.z;
        if (dx*dx + dz*dz < (collisionRadius_ * collisionRadius_)) {
            game.onTreeCollision(currentSpeed);
            break;
        }
    }
}

//========================= UI state =========================

enum class GameState { Menu, Playing, Paused };
struct UIState { GameState state{GameState::Menu}; int selectedVehicle{0}; };

//========================= Vehicle rig builder =========================

struct VehicleRig {
    std::shared_ptr<Object3D> root;
    std::shared_ptr<Object3D> chassis;
    std::vector<Object3D*> wheels;
};

static VehicleRig buildVehicle(AssimpLoader& loader, const std::string& modelFile, Scene& scene) {
    VehicleRig rig;
    auto model = loader.load(modelFile);

    rig.root   = Object3D::create();
    rig.chassis= Object3D::create();

    rig.chassis->add(model);
    rig.root->add(rig.chassis);
    rig.root->position.y = 0.f;

    scene.add(rig.root);
    findWheels(rig.chassis.get(), rig.wheels);
    return rig;
}

//========================= Main =========================

int main() {
    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());

    Scene scene; scene.background = Color::skyblue;
    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    const float kGroundSize = 2048.f; // matches PNG

    TextureLoader texLoader;
    auto drivingMap = texLoader.load("driving_map.png");

    auto groundMat = MeshPhongMaterial::create();
    //groundMat->color = Color::white;   // don’t tint
    groundMat->map = drivingMap;

    auto ground = Mesh::create(PlaneGeometry::create(kGroundSize, kGroundSize), groundMat);
    ground->rotation.x = -math::PI / 2.f;
    scene.add(ground);

    // lights
    scene.add(HemisphereLight::create(Color::white, Color::gray, 1.0f));
    auto dir = DirectionalLight::create(Color::white, 0.8f);
    dir->position.set(5, 10, 7);
    scene.add(dir);

    // UI scene (start/pause)
    Scene uiScene;
    auto uiCam = OrthographicCamera::create(-1, 1, 1, -1, 0.0f, 10.0f);
    auto startTexture = TextureLoader().load("startscreen_test.png");
    auto uiMat  = MeshBasicMaterial::create();
    uiMat->map = startTexture;
    uiMat->transparent = true;
    auto uiQuad = Mesh::create(PlaneGeometry::create(2,2), uiMat);
    uiQuad->position.z = -1.f;
    uiScene.add(uiQuad);

    // loader
    AssimpLoader loader;

    // vehicle choices
    std::vector<std::string> vehicleFiles = {
        "suv_model.glb",
        "sedan_model.glb",
        "tractor_model.glb"
    };

    // build initial vehicle
    int currentVehicleIndex = 0;
    VehicleRig rig = buildVehicle(loader, vehicleFiles[currentVehicleIndex], scene);

    // power-ups
    PowerUpManager powerUps;
    std::vector<std::string> powerUpFiles = {"power_up1.glb","power_up2.glb","power_up3.glb"};
    std::mt19937 rng(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<float> spawnDist(-950.f, 950.f);
    std::uniform_int_distribution<int>   kindDist(0,2);

    for (int i = 0; i < 120; ++i) {
        int k = kindDist(rng);
        auto node = loader.load(powerUpFiles[k]);
        node->position.set(spawnDist(rng), 0.f, spawnDist(rng));
        scene.add(node);

        PowerUp::Kind kind = (k==0) ? PowerUp::Kind::Grow
                           : (k==1) ? PowerUp::Kind::Faster
                                    : PowerUp::Kind::Shrink;

        powerUps.add(std::move(node), kind);
    }

    // trees
    TreeManager trees;
    for (int i = 0; i < 150; ++i) {
        auto t = loader.load("tree_model.glb");
        t->position.set(spawnDist(rng), 0.f, spawnDist(rng));
        scene.add(t);
        trees.add(std::move(t));
    }

    // input
    InputController input;
    canvas.addKeyListener(input);

    // game
    auto game = std::make_unique<Game>(canvas, renderer, scene, camera,
                                       *rig.root, *rig.chassis, rig.wheels,
                                       input, powerUps, trees);

    UIState ui;
    WindowSize lastSize = canvas.size();

    auto swapVehicle = [&](int newIndex) {
        if (newIndex < 0 || newIndex >= static_cast<int>(vehicleFiles.size())) return;
        if (newIndex == currentVehicleIndex) return;

        if (rig.root) scene.remove(*rig.root);
        rig = buildVehicle(loader, vehicleFiles[newIndex], scene);
        currentVehicleIndex = newIndex;

        // Recreate Game so visuals bind to new nodes (physics intentionally reset).
        game = std::make_unique<Game>(canvas, renderer, scene, camera,
                                      *rig.root, *rig.chassis, rig.wheels,
                                      input, powerUps, trees);
    };

    canvas.animate([&] {
        // handle resize
        auto s = canvas.size();
        if (s.width() != lastSize.width() || s.height() != lastSize.height()) {
            renderer.setSize(s);
            camera.aspect = float(s.width())/float(s.height());
            lastSize = s;
        }

        // vehicle selection (works in Menu & Playing)
        if (input.choose1) { ui.selectedVehicle = 0; swapVehicle(0); }
        if (input.choose2) { ui.selectedVehicle = 1; swapVehicle(1); }
        if (input.choose3) { ui.selectedVehicle = 2; swapVehicle(2); }

        switch (ui.state) {
            case GameState::Menu: {
                renderer.render(uiScene, *uiCam);
                if (input.keyEnter) { swapVehicle(ui.selectedVehicle); ui.state = GameState::Playing; }
                break;
            }
            case GameState::Playing: {
                if (input.keyEsc) { ui.state = GameState::Paused; break; }
                if (input.keyReset) { game->resetWorld(); }
                game->updateFrame();
                break;
            }
            case GameState::Paused: {
                renderer.render(uiScene, *uiCam);
                if (input.keyEnter) ui.state = GameState::Playing;
                if (input.keyReset) { game->resetWorld(); ui.state = GameState::Menu; }
                break;
            }
        }
    });

    return 0;
}