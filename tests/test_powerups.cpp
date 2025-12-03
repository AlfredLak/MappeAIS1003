#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "BilSimulator/Game.hpp"
#include "BilSimulator/InputState.hpp"
#include "BilSimulator/PowerUps.hpp"
#include "BilSimulator/PowerUpManager.hpp"

#include <threepp/threepp.hpp>

using namespace threepp;
using namespace MyCar;
using Catch::Approx;

TEST_CASE("PowerUps apply correct effects and deactivate") {
    auto root    = Object3D::create();
    auto chassis = Object3D::create();
    std::vector<Object3D*> wheels;

    InputState input;
    PowerUpManager mgr;
    Trees trees;

    // Real Game instance; we never render, so nullptrs are safe here
    Game game(nullptr, nullptr, nullptr, nullptr,
              root.get(), chassis.get(), wheels, &input, &mgr, &trees);

    auto growObj = Object3D::create();
    growObj->position.set(0, 0, 0);
    PowerUp growPU(growObj, PowerUp::Type::Grow);

    const float sx0 = root->scale.x;
    growPU.apply(game);
    REQUIRE(root->scale.x == Approx(sx0 * 1.4f));
    growPU.hide();
    REQUIRE_FALSE(growPU.active());

    auto fastObj = Object3D::create();
    PowerUp fastPU(fastObj, PowerUp::Type::Faster);

    const float maxF0  = game.physics().maxForward;
    const float accel0 = game.physics().accelForward;

    fastPU.apply(game); // multiplies tunables
    REQUIRE(game.physics().maxForward   == Approx(maxF0  * 1.35f));
    REQUIRE(game.physics().accelForward == Approx(accel0 * 1.3f));
    fastPU.hide();
    REQUIRE_FALSE(fastPU.active());

    auto shrinkObj = Object3D::create();
    PowerUp shrinkPU(shrinkObj, PowerUp::Type::Shrink);

    const float sx1 = root->scale.x;
    shrinkPU.apply(game);
    REQUIRE(root->scale.x == Approx(sx1 * 0.7f));
    shrinkPU.hide();
    REQUIRE_FALSE(shrinkPU.active());
}