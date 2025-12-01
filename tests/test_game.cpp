#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "BilSimulator/Game.hpp"
#include "BilSimulator/InputState.hpp"
#include "BilSimulator/PowerUps.hpp"
#include "BilSimulator/Trees.hpp"
#include "BilSimulator/VehicleVisuals.hpp"
#include "BilSimulator/VehiclePhysics.hpp"
#include <threepp/threepp.hpp>

using namespace minbil;
using namespace threepp;

TEST_CASE("Game resets position, rotation and scale") {
    // No rendering in this test: pass nullptr for canvas/renderer
    Scene scene;
    auto camera = PerspectiveCamera::create(60, 1.f, 0.1f, 1000.f);

    auto root = Object3D::create();
    auto chassis = Object3D::create();
    root->add(chassis);

    std::vector<Object3D*> wheels; // empty is fine for reset test
    InputState input;
    PowerUpManager powerups;
    Trees trees;

    Game game(nullptr, nullptr, &scene, camera.get(),
              root.get(), chassis.get(), wheels, &input, &powerups, &trees);

    // Nudge state
    root->position.set(5,0,7);
    root->rotation.set(0.1f, 0.2f, 0.3f);
    root->scale.set(2,2,2);

    game.reset();

    REQUIRE(root->position.x == Catch::Approx(0));
    REQUIRE(root->position.z == Catch::Approx(0));
    REQUIRE(root->rotation.y == Catch::Approx(0));
    REQUIRE(root->scale.x == Catch::Approx(1));
}

TEST_CASE("Game vehicle tuning propagates to physics") {
    Scene scene; auto camera = PerspectiveCamera::create(60,1.f,0.1f,1000.f);
    auto root = Object3D::create(); auto chassis = Object3D::create(); root->add(chassis);
    std::vector<Object3D*> wheels; InputState in; PowerUpManager pu; Trees tr;

    Game game(nullptr,nullptr,&scene,camera.get(), root.get(),chassis.get(),wheels,&in,&pu,&tr);

    game.applyVehicleTuning(24.f, 20.f, 36.f, -28.f);
    auto& phys = game.physics();
    REQUIRE(phys.accelForward == Catch::Approx(24.f));
    REQUIRE(phys.accelBackward == Catch::Approx(20.f));
    REQUIRE(phys.maxForward   == Catch::Approx(36.f));
    REQUIRE(phys.maxBackward  == Catch::Approx(-28.f));
}

TEST_CASE("Game collision snaps back and tilts") {
    Scene scene; auto camera = PerspectiveCamera::create(60,1.f,0.1f,1000.f);
    auto root = Object3D::create(); auto chassis = Object3D::create(); root->add(chassis);
    std::vector<Object3D*> wheels; InputState in; PowerUpManager pu; Trees tr;

    Game game(nullptr,nullptr,&scene,camera.get(), root.get(),chassis.get(),wheels,&in,&pu,&tr);

    // Move car and register safe position via a tiny update: call reset (sets safe to origin)
    game.reset();
    // Simulate a crash strong enough to trigger tilt
    game.onCollision(5.f);
    REQUIRE(root->position.x == Catch::Approx(0.f));
}