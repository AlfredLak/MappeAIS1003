#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

#include <chrono>
#include <random>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

#include "BilSimulator/InputState.hpp"
#include "BilSimulator/PowerUps.hpp"
#include "BilSimulator/PowerUpManager.hpp"
#include "BilSimulator/Trees.hpp"
#include "BilSimulator/Game.hpp"
#include "BilSimulator/VehicleSpecs.hpp"

using namespace threepp;
using namespace minbil;

#ifndef ASSETS_DIR
#  define ASSETS_DIR "./assets"
#endif

static std::string pathJoin(std::initializer_list<std::string> parts) {
    std::string out;
    for (auto& p : parts) {
        if (out.empty()) { out = p; continue; }
        char back = out.back();
        if (back != '/' && back != '\\') out += '/';
        out += p;
    }
#ifdef _WIN32
#endif
    return out;
}
static std::string model(const std::string& f)   { return pathJoin({ASSETS_DIR, "models",   f}); }
static std::string texture(const std::string& f) { return pathJoin({ASSETS_DIR, "textures", f}); }
static std::string uiTex(const std::string& f)   { return pathJoin({ASSETS_DIR, "ui",       f}); }

static void camThird(PerspectiveCamera& cam, const Vector3& p, float yaw, float back=6.f, float h=3.f) {
    cam.position.set(p.x - std::sin(yaw)*back, p.y + h, p.z - std::cos(yaw)*back);
    cam.lookAt(p);
}
static void camFront(PerspectiveCamera& cam, const Vector3& p, float yaw, float front=6.f, float h=3.f) {
    cam.position.set(p.x + std::sin(yaw)*front, p.y + h, p.z + std::cos(yaw)*front);
    cam.lookAt(p);
}
static void camSide(PerspectiveCamera& cam, const Vector3& p, float yaw, float side=5.f, float h=3.f) {
    const float sy = yaw - math::PI/2.f;
    cam.position.set(p.x - std::sin(sy)*side, p.y + h, p.z - std::cos(sy)*side);
    cam.lookAt(p);
}

static void findWheels(Object3D* root, std::vector<Object3D*>& out) {
    const char* names[] = { "wheel_FL", "wheel_FR", "wheel_BL", "wheel_BR" };
    root->traverse([&](Object3D& o){
        for (auto n : names) { if (o.name == n) { out.push_back(&o); break; } }
    });
}

struct CarRig {
    std::shared_ptr<Object3D> root;
    std::shared_ptr<Object3D> chassis;
    std::vector<Object3D*> wheels;
};

static CarRig buildCar(AssimpLoader& loader, const std::string& file, Scene& scene) {
    CarRig rig;
    auto modelNode = loader.load(file);
    rig.root     = Object3D::create();
    rig.chassis  = Object3D::create();
    rig.chassis->add(modelNode);     // visuals offset/tilt live here
    rig.root->add(rig.chassis);      // root carries world transform
    rig.root->position.y = 0.f;
    scene.add(rig.root);
    findWheels(rig.chassis.get(), rig.wheels);
    return rig;
}

enum class AppState { Menu, Playing, Paused };

int main() {
    Canvas canvas("BilSimulator");
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::skyblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    constexpr float kPlaneSize = 2048.f;
    auto planeGeo = PlaneGeometry::create(kPlaneSize, kPlaneSize);
    planeGeo->rotateX(-math::PI/2.f);

    auto mapTex = TextureLoader().load(texture("driving_map.png"));
    auto planeMat = MeshPhongMaterial::create();
    planeMat->map = mapTex;

    auto ground = Mesh::create(planeGeo, planeMat);
    scene.add(ground);

    auto grid = GridHelper::create((int)kPlaneSize, 200, Color::black, Color::darkgray);
    grid->position.y = 0.01f;
    scene.add(grid);

    auto hemi = HemisphereLight::create(Color::white, Color::gray, 1.0f);
    scene.add(hemi);
    auto dir = DirectionalLight::create(Color::white, 0.8f);
    dir->position.set(5, 10, 7);
    scene.add(dir);

    Scene uiScene;
    auto uiCam = OrthographicCamera::create(-1, 1, 1, -1, 0.0f, 10.0f);
    auto startTex = TextureLoader().load(uiTex("startscreen_test.png"));
    auto uiMat  = MeshBasicMaterial::create();
    uiMat->map  = startTex;
    uiMat->transparent = true;
    auto uiQuad = Mesh::create(PlaneGeometry::create(2,2), uiMat);
    uiQuad->position.z = -1.f;
    uiScene.add(uiQuad);

    AssimpLoader loader;

    std::vector<std::string> carFiles = {
        model("suv_model.glb"),
        model("sedan_model.glb"),
        model("tractor_model.glb")
    };

    int currentCarIndex = 0;
    CarRig rig = buildCar(loader, carFiles[currentCarIndex], scene);

    PowerUpManager powerUps;
    {
        std::vector puFiles = {
            model("power_up1.glb"),
            model("power_up2.glb"),
            model("power_up3.glb")
        };
        std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> dxy(-(kPlaneSize/2.f - 50.f), (kPlaneSize/2.f - 50.f));
        std::uniform_int_distribution<int>   pick(0, 2);
        for (int i=0; i<120; ++i) {
            int t = pick(rng);
            auto obj = loader.load(puFiles[t]);
            obj->position.set(dxy(rng), 0.f, dxy(rng));
            scene.add(obj);

            PowerUp::Type type = (t==0)? PowerUp::Type::Grow
                                 : (t==1)? PowerUp::Type::Faster
                                         : PowerUp::Type::Shrink;
            powerUps.add(std::move(obj), type);
        }
    }

    Trees trees;
    {
        std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<float> dxy(-(kPlaneSize/2.f - 50.f), (kPlaneSize/2.f - 50.f));
        for (int i=0; i<150; ++i) {
            auto t = loader.load(model("tree_model.glb"));
            t->position.set(dxy(rng), 0.f, dxy(rng));
            scene.add(t);
            trees.add(std::move(t));
        }
    }

    InputState input;
    canvas.addKeyListener(input);

    std::unique_ptr<Game> game = std::make_unique<Game>(
        &canvas, &renderer, &scene, &camera,
        rig.root.get(), rig.chassis.get(), rig.wheels,
        &input, &powerUps, &trees
    );

    {
        const auto tune = tuningFor(carFiles[currentCarIndex]);
        game->applyVehicleTuning(tune.accelF, tune.accelB, tune.maxF, tune.maxB);
    }

    trees.setCollisionSink(game.get());

    camThird(camera, rig.root->position, 0.f);

    AppState appState = AppState::Menu;
    int selectedIndex = 0;
    WindowSize lastSize = canvas.size();

    auto swapVehicle = [&](int newIndex){
        if (newIndex < 0 || newIndex >= (int)carFiles.size()) return;
        if (newIndex == currentCarIndex) return;

        if (rig.root) scene.remove(*rig.root);
        rig = buildCar(loader, carFiles[newIndex], scene);
        currentCarIndex = newIndex;

        game = std::make_unique<Game>(&canvas,&renderer,&scene,&camera,
                                      rig.root.get(), rig.chassis.get(), rig.wheels,
                                      &input, &powerUps, &trees);

        trees.setCollisionSink(game.get());
    };

    canvas.animate([&] {
        auto s = canvas.size();
        if (s.width() != lastSize.width() || s.height() != lastSize.height()) {
            renderer.setSize(s);
            camera.aspect = float(s.width())/float(s.height());
            lastSize = s;
        }

        if (input.select1) { selectedIndex = 0; swapVehicle(0); }
        if (input.select2) { selectedIndex = 1; swapVehicle(1); }
        if (input.select3) { selectedIndex = 2; swapVehicle(2); }

        switch (appState) {
            case AppState::Menu: {
                renderer.render(uiScene, *uiCam);
                if (input.enter) { swapVehicle(selectedIndex); appState = AppState::Playing; }
                break;
            }
            case AppState::Playing: {
                if (input.escape) { appState = AppState::Paused; break; }
                if (input.reset)  { game->reset(); }
                game->updateFrame();
                break;
            }
            case AppState::Paused: {
                renderer.render(uiScene, *uiCam);
                if (input.enter) appState = AppState::Playing;
                if (input.reset) { game->reset(); appState = AppState::Menu; }
                break;
            }
        }
    });

    return 0;
}