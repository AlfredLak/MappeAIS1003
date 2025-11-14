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

    void onKeyPressed(KeyEvent evt) override {
        switch (evt.key) {
            case Key::W: forward = true; break;
            case Key::S: backward = true; break;
            case Key::A: left = true; break;
            case Key::D: right = true; break;
            default: break;
        }
    }

    void onKeyReleased(KeyEvent evt) override {
        switch (evt.key) {
            case Key::W: forward = false; break;
            case Key::S: backward = false; break;
            case Key::A: left = false; break;
            case Key::D: right = false; break;
            default: break;
        }
    }
};

static void placeCameraBehindBox(PerspectiveCamera& cam,
                                 const Vector3& boxPos,
                                 float yawRadians,
                                 float distBack = 6.f,
                                 float height = 3.f) {

    float backX = std::sin(yawRadians) * distBack;
    float backZ = std::cos(yawRadians) * distBack;

    cam.position.set(boxPos.x - backX, boxPos.y + height, boxPos.z - backZ);
    cam.lookAt(boxPos);
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

        placeCameraBehindBox(*camera_, player_->position, yaw_);
    }

    void update() {
        const float dt = 1.f / 60.f;

        const float turnSpeed = 1.8f;
        if (input_->left)  yaw_ += turnSpeed * dt;
        if (input_->right) yaw_ -= turnSpeed * dt;

        const float accelForward  = 5.f;
        const float accelBackward = 4.f;
        const float drag          = 2.5f;

        if (input_->forward) {
            velocity_ += accelForward * dt;
        }
        if (input_->backward) {
            velocity_ -= accelBackward * dt;
        }

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
        const float maxBackward = -5.f; // negative
        if (velocity_ > maxForward)  velocity_ = maxForward;
        if (velocity_ < maxBackward) velocity_ = maxBackward;

        float dirX = std::sin(yaw_);
        float dirZ = std::cos(yaw_);
        player_->position.x += dirX * velocity_ * dt;
        player_->position.z += dirZ * velocity_ * dt;

        player_->rotation.y = yaw_;

        placeCameraBehindBox(*camera_, player_->position, yaw_);

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
    std::chrono::steady_clock::time_point start_;
};

int main() {

    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::skyblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    auto planeGeo = PlaneGeometry::create(200, 200);
    auto planeMat = MeshPhongMaterial::create();
    planeMat->color = Color::forestgreen;
    auto plane = Mesh::create(planeGeo, planeMat);
    plane->rotation.x = -math::PI / 2.f;
    scene.add(plane);

    auto grid = GridHelper::create(200, 200, Color::black, Color::darkgray);
    grid->position.y = 0.01f;
    scene.add(grid);

    auto hemi = HemisphereLight::create(Color::white, Color::gray, 1.0f);
    scene.add(hemi);
    auto dirLight = DirectionalLight::create(Color::white, 0.8f);
    dirLight->position.set(5, 10, 7);
    scene.add(dirLight);

    AssimpLoader assimp;
    auto car = assimp.load("suv_model.glb");
    car->position.y = 0.0f;
    scene.add(car);


    DriveInput input;
    canvas.addKeyListener(input);

    Game game(&canvas, &renderer, &scene, &camera, car.get(), &input);

    canvas.animate([&] {
    game.update();
    }
    );

    return 0;
}