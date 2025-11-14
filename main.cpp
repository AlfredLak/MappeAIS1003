#include <iostream>
#include <cmath>
#include <functional>
#include <chrono>
#include <threepp/threepp.hpp>

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

    // camera position a bit behind the box, in direction of current yaw
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

        // initial camera placement
        placeCameraBehindBox(*camera_, player_->position, yaw_);
    }

    void update() {
        float t = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - start_).count();

        (void)t;

        float dt = 1.f / 60.f;

        // rotate left/right
        const float turnSpeed = 1.8f; // radians/sec
        if (input_->left)  yaw_ += turnSpeed * dt;
        if (input_->right) yaw_ -= turnSpeed * dt;

        // forward/backward
        const float moveSpeed = 4.f; // units/sec
        float forwardDirX = std::sin(yaw_);
        float forwardDirZ = std::cos(yaw_);

        if (input_->forward) {
            player_->position.x += forwardDirX * moveSpeed * dt;
            player_->position.z += forwardDirZ * moveSpeed * dt;
        }
        if (input_->backward) {
            player_->position.x -= forwardDirX * moveSpeed * dt;
            player_->position.z -= forwardDirZ * moveSpeed * dt;
        }

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
    std::chrono::steady_clock::time_point start_;
};

int main() {

    Canvas canvas("Bilsimulator");
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::skyblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);


    auto planeGeo = PlaneGeometry::create(200, 200);
    auto planeMat = MeshPhongMaterial::create();
    planeMat->color = Color::forestgreen;
    auto plane = Mesh::create(planeGeo, planeMat);
    plane->rotation.x = -math::PI / 2.f; // lay it flat
    scene.add(plane);

    auto hemi = HemisphereLight::create(Color::white, Color::gray, 1.0f);
    scene.add(hemi);
    auto dirLight = DirectionalLight::create(Color::white, 0.8f);
    dirLight->position.set(5, 10, 7);
    scene.add(dirLight);

    auto boxGeo = BoxGeometry::create(1, 1, 2); // a “car-ish” box
    auto boxMat = MeshPhongMaterial::create();
    boxMat->color = Color::red;
    auto player = Mesh::create(boxGeo, boxMat);
    player->position.y = 0.5f; // sit on plane
    scene.add(player);

    DriveInput input;
    canvas.addKeyListener(input);

    Game game(&canvas, &renderer, &scene, &camera, player.get(), &input);

    canvas.animate([&] {
           game.update();
       });
    return 0;
}