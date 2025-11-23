#include <iostream>
#include <cmath>
#include <functional>
#include <chrono>
#include <random>
#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

using namespace threepp;

// ------------------ Input listener ------------------
class DriveInput : public KeyListener {
public:
    bool forward{false};
    bool backward{false};
    bool left{false};
    bool right{false};
    bool view{false}; // V

    void onKeyPressed(KeyEvent evt) override {
        switch (evt.key) {
            case Key::W: forward = true; break;
            case Key::S: backward = true; break;
            case Key::A: left = true; break;
            case Key::D: right = true; break;
            case Key::V: view = true; break;
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
            default: break;
        }
    }
};

// camera helpers -------------------------------------------------
static void placeCameraThirdPerson(PerspectiveCamera& cam,
                                   const Vector3& carPos,
                                   float yawRadians,
                                   float distBack = 6.f,
                                   float height = 3.f) {
    float backX = std::sin(yawRadians) * distBack;
    float backZ = std::cos(yawRadians) * distBack;
    cam.position.set(carPos.x - backX, carPos.y + height, carPos.z - backZ);
    cam.lookAt(carPos);
}

static void placeCameraSideRight(PerspectiveCamera& cam,
                                 const Vector3& carPos,
                                 float yawRadians,
                                 float sideDist = 5.f,
                                 float height = 3.f) {
    float sideYaw = yawRadians - math::PI / 2.f;
    float sideX = std::sin(sideYaw) * sideDist;
    float sideZ = std::cos(sideYaw) * sideDist;
    cam.position.set(carPos.x - sideX, carPos.y + height, carPos.z - sideZ);
    cam.lookAt(carPos);
}

// find wheels by exact names -------------------------------------
static void collectWheels(Object3D* root, std::vector<Object3D*>& out) {
    const char* names[] = { "wheel_FL", "wheel_FR", "wheel_BL", "wheel_BR" };
    root->traverse([&](Object3D& obj){
        for (auto n : names) {
            if (obj.name == n) {
                out.push_back(&obj);
                break;
            }
        }
    });
}

// forward-declare Game so PowerUp can call back into it
class Game;

// ------------------ PowerUp class ------------------
class PowerUp {
public:
    enum class Type { Grow, Faster, Shrink };

    PowerUp(std::shared_ptr<Object3D> object, Type type)
        : object_(std::move(object)), type_(type) {}

    Object3D* object() const { return object_.get(); }
    bool isActive() const { return active_; }
    Type type() const { return type_; }

    void apply(Game& game);

    void deactivate() {
        active_ = false;
        if (object_) object_->visible = false;
    }

private:
    std::shared_ptr<Object3D> object_;
    Type type_;
    bool active_{true};
};

// ------------------ PowerUpManager ------------------
class PowerUpManager {
public:
    void add(std::shared_ptr<Object3D> obj, PowerUp::Type type) {
        powerups_.emplace_back(std::move(obj), type);
    }

    void update(Game& game, Object3D* car) {
        if (!car) return;
        const auto& p = car->position;
        for (auto& pu : powerups_) {
            if (!pu.isActive()) continue;
            auto* obj = pu.object();
            if (!obj) continue;
            float dx = p.x - obj->position.x;
            float dz = p.z - obj->position.z;
            if (dx*dx + dz*dz < 1.5f * 1.5f) {
                pu.apply(game);
                pu.deactivate();
            }
        }
    }
private:
    std::vector<PowerUp> powerups_;
};

// ------------------ Game ------------------
class Game {
public:
    Game(Canvas* canvas,
         GLRenderer* renderer,
         Scene* scene,
         PerspectiveCamera* camera,
         Object3D* player,
         DriveInput* input,
         PowerUpManager* powerups)
        : canvas_(canvas),
          renderer_(renderer),
          scene_(scene),
          camera_(camera),
          player_(player),
          input_(input),
          powerups_(powerups),
          start_(std::chrono::steady_clock::now()) {

        collectWheels(player_, wheels_);
        placeCameraThirdPerson(*camera_, player_->position, yaw_);
    }

    void update() {
        const float dt = 1.f / 60.f;

        // steering
        if (input_->left)  yaw_ += turnSpeed_ * dt;
        if (input_->right) yaw_ -= turnSpeed_ * dt;

        // accel/decel
        if (input_->forward)  velocity_ += accelForward_ * dt;
        if (input_->backward) velocity_ -= accelBackward_ * dt;

        // drag
        if (!input_->forward && !input_->backward) {
            if (velocity_ > 0.f) {
                velocity_ -= drag_ * dt;
                if (velocity_ < 0.f) velocity_ = 0.f;
            } else if (velocity_ < 0.f) {
                velocity_ += drag_ * dt;
                if (velocity_ > 0.f) velocity_ = 0.f;
            }
        }

        // clamp
        if (velocity_ > maxForward_)  velocity_ = maxForward_;
        if (velocity_ < maxBackward_) velocity_ = maxBackward_;

        // move
        float dirX = std::sin(yaw_);
        float dirZ = std::cos(yaw_);
        player_->position.x += dirX * velocity_ * dt;
        player_->position.z += dirZ * velocity_ * dt;

        // rotate car
        player_->rotation.y = yaw_;

        // spin wheels
        const float wheelRadius = 0.35f;
        if (wheelRadius > 0.f) {
            float angularDelta = (velocity_ / wheelRadius) * dt;
            for (auto* w : wheels_) {
                w->rotation.x += angularDelta;
            }
        }

        // let power-ups do their thing
        if (powerups_) {
            powerups_->update(*this, player_);
        }

        // camera toggle
        if (input_->view && !prevViewKey_) {
            sideView_ = !sideView_;
        }
        prevViewKey_ = input_->view;

        if (sideView_) {
            placeCameraSideRight(*camera_, player_->position, yaw_);
        } else {
            placeCameraThirdPerson(*camera_, player_->position, yaw_);
        }

        renderer_->render(*scene_, *camera_);
    }

    // ---- called by PowerUp::apply ----
    void growCar(float factor) {
        player_->scale.multiplyScalar(factor);
    }

    void makeFaster(float speedFactor, float accelFactor) {
        maxForward_  *= speedFactor;
        accelForward_ *= accelFactor;
    }

    /*void makeSlower(float speedFactor, float accelFactor) {
        maxForward_  *= speedFactor;
        accelForward_ *= accelFactor;
        if (velocity_ > maxForward_) velocity_ = maxForward_;
    }*/

private:
    Canvas* canvas_;
    GLRenderer* renderer_;
    Scene* scene_;
    PerspectiveCamera* camera_;
    Object3D* player_;
    DriveInput* input_;
    PowerUpManager* powerups_;

    float yaw_{0.f};
    float velocity_{0.f};

    // tunables
    float turnSpeed_{1.8f};
    float accelForward_{5.f};
    float accelBackward_{4.f};
    float drag_{2.5f};
    float maxForward_{8.f};
    float maxBackward_{-5.f};

    bool sideView_{false};
    bool prevViewKey_{false};

    std::vector<Object3D*> wheels_;
    std::chrono::steady_clock::time_point start_;
};

// implement PowerUp::apply now that Game is defined
void PowerUp::apply(Game& game) {
    switch (type_) {
        case Type::Grow:
            game.growCar(1.4f);
            break;
        case Type::Faster:
            game.makeFaster(1.35f, 1.3f);
            break;
        case Type::Shrink:
            game.growCar(0.7f);
            break;
    }
}

// ------------------ main ------------------
int main() {

    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::skyblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    // big ground
    auto planeGeo = PlaneGeometry::create(2000, 2000);
    auto planeMat = MeshPhongMaterial::create();
    planeMat->color = Color::forestgreen;
    auto plane = Mesh::create(planeGeo, planeMat);
    plane->rotation.x = -math::PI / 2.f;
    scene.add(plane);

    auto grid = GridHelper::create(2000, 2000, Color::black, Color::darkgray);
    grid->position.y = 0.01f;
    scene.add(grid);

    // lights
    auto hemi = HemisphereLight::create(Color::white, Color::gray, 1.0f);
    scene.add(hemi);
    auto dirLight = DirectionalLight::create(Color::white, 0.8f);
    dirLight->position.set(5, 10, 7);
    scene.add(dirLight);

    // loader
    AssimpLoader assimp;

    // car
    auto car = assimp.load("suv_model.glb");
    car->position.y = 0.f;
    scene.add(car);

    // power-ups (OO style)
    PowerUpManager puManager;

    std::vector<std::string> puFiles = { "power_up1.glb", "power_up2.glb", "power_up3.glb" };

    std::mt19937 rng(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<float> distXY(-950.f, 950.f); // near full 2000x2000 plane
    std::uniform_int_distribution<int>   distType(0, 2);

    const int powerupCount = 120;
    for (int i = 0; i < powerupCount; ++i) {
        int t = distType(rng);
        auto obj = assimp.load(puFiles[t]);
        obj->position.set(distXY(rng), 0.f, distXY(rng));
        scene.add(obj);

        PowerUp::Type type;
        if (t == 0) type = PowerUp::Type::Grow;
        else if (t == 1) type = PowerUp::Type::Faster;
        else type = PowerUp::Type::Shrink; // was Slower

        puManager.add(std::move(obj), type);
    }

    DriveInput input;
    canvas.addKeyListener(input);

    Game game(&canvas, &renderer, &scene, &camera, car.get(), &input, &puManager);

    canvas.animate([&] {
        game.update();
    });

    return 0;
}