#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

#include <chrono>
#include <random>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "BilSimulator/InputState.hpp"
#include "BilSimulator/PowerUps.hpp"
#include "BilSimulator/PowerUpManager.hpp"
#include "BilSimulator/Trees.hpp"
#include "BilSimulator/Game.hpp"
#include "BilSimulator/VehicleSpecs.hpp"
#include "BilSimulator/VehicleFactory.hpp"

using namespace threepp;
using namespace MyCar;

#ifndef ASSETS_DIR
#  define ASSETS_DIR "./assets"
#endif

// Path helper
static std::string pathJoin(std::initializer_list<std::string> parts) {
    std::string out;
    for (auto& p : parts) {
        if (out.empty()) { out = p; continue; }
        const char back = out.back();
        if (back != '/' && back != '\\') out += '/';
        out += p;
    }
    return out;
}

static std::string model(const std::string& f)   { return pathJoin({ASSETS_DIR, "models",   f}); }
static std::string texture(const std::string& f) { return pathJoin({ASSETS_DIR, "textures", f}); }
static std::string uiTex(const std::string& f)   { return pathJoin({ASSETS_DIR, "ui",       f}); }

// ---------- quick camera placements (initial view only; runtime cam by Game) ----------
static void camThird(PerspectiveCamera& cam, const Vector3& p, float yaw, float back=6.f, float h=3.f) {
    cam.position.set(p.x - std::sin(yaw)*back, p.y + h, p.z - std::cos(yaw)*back);
    cam.lookAt(p);
}

enum class AppState { Menu, Playing, Paused };

int main() {
    Canvas canvas("BilSimulator");
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::skyblue;

    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    // ----- ground + map -----
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

    // ----- light -----
    scene.add(HemisphereLight::create(Color::white, Color::gray, 1.0f));
    auto dir = DirectionalLight::create(Color::white, 0.8f);
    dir->position.set(5, 10, 7);
    scene.add(dir);

    // ----- simple UI scene (menu/pause) -----
    Scene uiScene;
    auto uiCam = OrthographicCamera::create(-1, 1, 1, -1, 0.0f, 10.0f);
    auto startTex = TextureLoader().load(uiTex("startscreen_test.png"));
    auto uiMat  = MeshBasicMaterial::create();
    uiMat->map  = startTex;
    uiMat->transparent = true;
    auto uiQuad = Mesh::create(PlaneGeometry::create(2,2), uiMat);
    uiQuad->position.z = -1.f;
    uiScene.add(uiQuad);

    // ----- asset loader -----
    AssimpLoader loader;

    // ----- vehicle models -----
    std::vector<std::string> carFiles = {
        model("suv_model.glb"),
        model("sedan_model.glb"),
        model("tractor_model.glb")
    };

    int currentCarIndex = 0;
    CarRig rig = VehicleFactory::buildCar(loader, carFiles[currentCarIndex], scene);

    // ----- power-ups -----
    PowerUpManager powerUps;
    {
        std::vector<std::string> puFiles = {
            model("muffin.glb"),   // Grow
            model("soda.glb"),     // Faster
            model("mushroom.glb")  // Shrink
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

    // ----- trees -----
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

    // ----- input -----
    InputState input;
    canvas.addKeyListener(input);

    // ----- game wrapper -----
    std::unique_ptr<Game> game = std::make_unique<Game>(
        &canvas, &renderer, &scene, &camera,
        rig.root.get(), rig.chassis.get(), rig.wheels,
        &input, &powerUps, &trees
    );

    // per-vehicle tuning (accel/max speed) from VehicleSpecs
    {
        const auto tune = tuningFor(carFiles[currentCarIndex]);
        game->applyVehicleTuning(tune.accelF, tune.accelB, tune.maxF, tune.maxB);
    }

    // trees -> Game collision callback (Game implements CollisionSink in your codebase)
    trees.setCollisionSink(static_cast<CollisionSink*>(game.get()));

    // initial camera placement; runtime follow handled inside Game
    camThird(camera, rig.root->position, 0.f);

    // ----- app state -----
    AppState appState = AppState::Menu;
    int selectedIndex = 0;
    WindowSize lastSize = canvas.size();

    // swap vehicle helper
    auto swapVehicle = [&](int newIndex){
        if (newIndex < 0 || newIndex >= (int)carFiles.size()) return;
        if (newIndex == currentCarIndex) return;

        if (rig.root) scene.remove(*rig.root);
        rig = VehicleFactory::buildCar(loader, carFiles[newIndex], scene);
        currentCarIndex = newIndex;

        game = std::make_unique<Game>(&canvas,&renderer,&scene,&camera,
                                      rig.root.get(), rig.chassis.get(), rig.wheels,
                                      &input, &powerUps, &trees);

        // apply tuning for new model
        const auto tune = tuningFor(carFiles[currentCarIndex]);
        game->applyVehicleTuning(tune.accelF, tune.accelB, tune.maxF, tune.maxB);

        // reattach sink after recreating Game
        trees.setCollisionSink(static_cast<CollisionSink*>(game.get()));
    };

    // ----- main loop -----
    canvas.animate([&] {
        auto s = canvas.size();
        if (s.width() != lastSize.width() || s.height() != lastSize.height()) {
            renderer.setSize(s);
            camera.aspect = float(s.width())/float(s.height());
            lastSize = s;
        }

        // top-row digits -> pick car immediately (works in menu & playing)
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
                if (input.pause) { appState = AppState::Paused; break; }
                if (input.reset) { game->reset(); }
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