#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "BilSimulator/Game.hpp"
#include "BilSimulator/InputState.hpp"
#include "BilSimulator/PowerUps.hpp"
#include "BilSimulator/Trees.hpp"
#include <threepp/threepp.hpp>

using namespace threepp;
using namespace minbil;
using Catch::Approx;

TEST_CASE("Game: vehicle tuning is applied to physics") {
    auto root    = Object3D::create();
    auto chassis = Object3D::create();
    std::vector<Object3D*> wheels;

    InputState input;
    PowerUpManager pums;
    Trees trees;

    Game game(nullptr, nullptr, nullptr, nullptr,
              root.get(), chassis.get(), wheels, &input, &pums, &trees);

    game.applyVehicleTuning(24.f, 20.f, 36.f, -28.f);

    auto& phys = game.physics();
    REQUIRE(phys.accelForward == Approx(24.f));
    REQUIRE(phys.accelBackward == Approx(20.f));
    REQUIRE(phys.maxForward == Approx(36.f));
    REQUIRE(phys.maxBackward == Approx(-28.f));
}

TEST_CASE("Game: onCollision does not crash and resets safely") {
    auto root    = Object3D::create();
    auto chassis = Object3D::create();
    std::vector<Object3D*> wheels;

    InputState input;
    PowerUpManager pums;
    Trees trees;

    Game game(nullptr, nullptr, nullptr, nullptr,
              root.get(), chassis.get(), wheels, &input, &pums, &trees);

    // Put car somewhere non-zero then collide
    root->position.set(5.f, 0.f, 5.f);
    game.reset(); // establishes safe state
    game.onCollision(3.0f); // should not throw or crash

    SUCCEED();
}