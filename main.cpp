#include <iostream>
#include <cmath>
#include <functional>
#include <chrono>
#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

using namespace threepp;

class DriveInput : public KeyListener {
public:
    bool forward{false};
    bool backward{false};
    bool left{false};
    bool right{false};
    bool view{false};

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

static void collectWheels(Object3D* root, std::vector<Object3D*>& out) {
    const char* names[] = {
        "wheel_FL", "wheel_FR", "wheel_BL", "wheel_BR"
    };
    root->traverse([&](Object3D& obj){
        for (auto n : names) {
            if (obj.name == n) {
                out.push_back(&obj);
                break;
            }
        }
    });
}

class Game {
public:
    Game(Canvas* canvas,
         GLRenderer* renderer,
         Scene* scene,
         PerspectiveCamera* camera,
         Object3D* player,
         DriveInput* input)
        : canvas_(canvas),
          renderer_(renderer),
          scene_(scene),
          camera_(camera),
          player_(player),
          input_(input),
          start_(std::chrono::steady_clock::now()) {

        collectWheels(player_, wheels_);
        placeCameraThirdPerson(*camera_, player_->position, yaw_);
    }

    void update() {
        const float dt = 1.f / 60.f;

        const float turnSpeed = 1.8f;
        if (input_->left)  yaw_ += turnSpeed * dt;
        if (input_->right) yaw_ -= turnSpeed * dt;

        const float accelForward  = 5.f;
        const float accelBackward = 4.f;
        const float drag          = 2.5f;

        if (input_->forward)  velocity_ += accelForward * dt;
        if (input_->backward) velocity_ -= accelBackward * dt;

        if (!input_->forward && !input_->backward) {
            if (velocity_ > 0.f) {
                velocity_ -= drag * dt;
                if (velocity_ < 0.f) velocity_ = 0.f;
            } else if (velocity_ < 0.f) {
                velocity_ += drag * dt;
                if (velocity_ > 0.f) velocity_ = 0.f;
            }
        }

        const float maxForward  = 8.f;
        const float maxBackward = -5.f;
        if (velocity_ > maxForward)  velocity_ = maxForward;
        if (velocity_ < maxBackward) velocity_ = maxBackward;

        float dirX = std::sin(yaw_);
        float dirZ = std::cos(yaw_);
        player_->position.x += dirX * velocity_ * dt;
        player_->position.z += dirZ * velocity_ * dt;

        player_->rotation.y = yaw_;

        const float wheelRadius = 0.35f; // adjust for your model
        float angularDelta = 0.f;
        if (wheelRadius > 0.f) {
            angularDelta = (velocity_ / wheelRadius) * dt;
        }
        for (auto* w : wheels_) {
            w->rotation.x += angularDelta;
        }

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

private:
    Canvas* canvas_;
    GLRenderer* renderer_;
    Scene* scene_;
    PerspectiveCamera* camera_;
    Object3D* player_;
    DriveInput* input_;

    float yaw_{0.f};
    float velocity_{0.f};

    bool sideView_{false};
    bool prevViewKey_{false};

    std::vector<Object3D*> wheels_;
    std::chrono::steady_clock::time_point start_;
};

int main() {

    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::skyblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    auto planeGeo = PlaneGeometry::create(2000, 2000);  // was 200,200
    auto planeMat = MeshPhongMaterial::create();
    planeMat->color = Color::forestgreen;
    auto plane = Mesh::create(planeGeo, planeMat);
    plane->rotation.x = -math::PI / 2.f;
    scene.add(plane);

    auto grid = GridHelper::create(2000, 2000, Color::black, Color::darkgray);
    grid->position.y = 0.01f;
    scene.add(grid);

    auto hemi = HemisphereLight::create(Color::white, Color::gray, 1.0f);
    scene.add(hemi);
    auto dirLight = DirectionalLight::create(Color::white, 0.8f);
    dirLight->position.set(5, 10, 7);
    scene.add(dirLight);

    AssimpLoader assimp;
    auto car = assimp.load("suv_model.glb");
    car->position.y = 0.f;
    scene.add(car);

    DriveInput input;
    canvas.addKeyListener(input);

    Game game(&canvas, &renderer, &scene, &camera, car.get(), &input);

    canvas.animate([&] {
        game.update();
    });

    return 0;
}