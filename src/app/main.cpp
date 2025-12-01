#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

#include "BilSimulator/InputState.hpp"
#include "BilSimulator/CameraController.hpp"
#include "BilSimulator/VehiclePhysics.hpp"
#include "BilSimulator/VehicleVisuals.hpp"
#include "BilSimulator/PowerUps.hpp"
#include "BilSimulator/Trees.hpp"
#include "BilSimulator/VehicleRig.hpp"
#include "BilSimulator/Game.hpp"

#include <chrono>
#include <random>
#include <vector>
#include <string>

using namespace threepp;
using namespace minbil;

struct VehicleSpec {
    std::string file;
    float accelF, accelB, maxF, maxB;
};

int main(){
    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());
    Scene scene; scene.background = Color::skyblue;
    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    const float GroundSize = 2048.f;
    const std::string texturesDir = std::string(ASSETS_DIR) + "/textures/";
    auto drivingMap = TextureLoader().load(texturesDir + "driving_map.png");
    auto groundMat = MeshPhongMaterial::create();
    groundMat->color = Color::white;
    groundMat->map = drivingMap;
    auto ground = Mesh::create(PlaneGeometry::create(GroundSize, GroundSize), groundMat);
    ground->rotation.x = -math::PI/2.f; scene.add(ground);

    // Lights
    scene.add(HemisphereLight::create(Color::white, Color::gray,1.0f));
    auto sun=DirectionalLight::create(Color::white,0.8f); sun->position.set(5,10,7); scene.add(sun);

    // Start/Pause UI
    Scene uiScene;
    auto uiCam = OrthographicCamera::create(-1, 1, 1, -1, 0.0f, 10.0f);
    const std::string uiDir = std::string(ASSETS_DIR) + "/ui/";
    auto startTex = TextureLoader().load(uiDir + "startscreen_test.png");
    auto uiMat  = MeshBasicMaterial::create(); uiMat->map=startTex; uiMat->transparent=true;
    auto uiQuad = Mesh::create(PlaneGeometry::create(2,2), uiMat);
    uiQuad->position.z = -1.f; uiScene.add(uiQuad);

    AssimpLoader loader;
    const std::string modelsDir = std::string(ASSETS_DIR) + "/models/";

    std::vector<VehicleSpec> vehicles = {
        {modelsDir + "suv_model.glb",     20.f, 18.f, 30.f, -25.f},
        {modelsDir + "sedan_model.glb",   24.f, 20.f, 36.f, -28.f},
        {modelsDir + "tractor_model.glb", 12.f, 10.f, 18.f, -14.f}
    };

    int currentVehicleIndex = 0;
    auto rig = buildVehicle(loader, vehicles[currentVehicleIndex].file, scene);

    // Power-ups
    PowerUpManager powerUps;
    std::vector powerUpFiles={
        modelsDir + "power_up1.glb",
        modelsDir + "power_up2.glb",
        modelsDir + "power_up3.glb"
    };

    std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> randPos(-(GroundSize/2.f-50.f),(GroundSize/2.f-50.f));
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
        auto t=loader.load(modelsDir + "tree_model.glb");
        t->position.set(randPos(rng),0,randPos(rng));
        scene.add(t);
        trees.add(std::move(t));
    }

    // Input + Game
    InputState input; canvas.addKeyListener(input);
    std::unique_ptr<Game> game = std::make_unique<Game>(
        &canvas, &renderer, &scene, &camera,
        rig.root.get(), rig.chassis.get(), rig.wheels,
        &input, &powerUps, &trees);

    // Apply initial vehicle tuning
    {
        const auto& v = vehicles[currentVehicleIndex];
        game->applyVehicleTuning(v.accelF, v.accelB, v.maxF, v.maxB);
    }

    GameState gameState = GameState::Menu;
    int selectedIndex = 0;
    WindowSize lastSize=canvas.size();

    auto swapVehicle = [&](int newIndex){
        if (newIndex < 0 || newIndex >= (int)vehicles.size()) return;
        if (newIndex == currentVehicleIndex) return;

        if (rig.root) scene.remove(*rig.root);
        rig = buildVehicle(loader, vehicles[newIndex].file, scene);
        currentVehicleIndex = newIndex;

        game = std::make_unique<Game>(&canvas,&renderer,&scene,&camera,
                                      rig.root.get(), rig.chassis.get(), rig.wheels,
                                      &input, &powerUps, &trees);

        const auto& v = vehicles[currentVehicleIndex];
        game->applyVehicleTuning(v.accelF, v.accelB, v.maxF, v.maxB);
    };

    canvas.animate([&]{
        auto s=canvas.size();
        if(s.width()!=lastSize.width()||s.height()!=lastSize.height()){
            renderer.setSize(s);
            camera.aspect=float(s.width())/float(s.height());
            lastSize=s;
        }

        // top-row vehicle selection (works anytime)
        if (input.select1) selectedIndex = 0, swapVehicle(0);
        if (input.select2) selectedIndex = 1, swapVehicle(1);
        if (input.select3) selectedIndex = 2, swapVehicle(2);

        switch (gameState) {
            case GameState::Menu: {
                renderer.render(uiScene, *uiCam);
                if (input.enter) { swapVehicle(selectedIndex); gameState = GameState::Playing; }
                break;
            }
            case GameState::Playing: {
                if (input.escape) { gameState = GameState::Paused; break; }
                if (input.reset)  { game->reset(); }
                game->updateFrame();
                break;
            }
            case GameState::Paused: {
                renderer.render(uiScene, *uiCam);
                if (input.enter) gameState = GameState::Playing;
                if (input.reset) { game->reset(); gameState = GameState::Menu; }
                break;
            }
        }
    });

    return 0;
}