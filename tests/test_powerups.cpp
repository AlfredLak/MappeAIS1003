#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/PowerUps.hpp"
#include "BilSimulator/Game.hpp"   // for signatures only
#include <threepp/threepp.hpp>
#include <memory>

using namespace minbil;
using namespace threepp;

namespace {
    struct FakeGame {
        float scale{1.f};
        float maxF{30.f}, accelF{20.f};
        void growCar(float f) { scale *= f; }
        void faster(float speedMax, float speedAcc) { maxF *= speedMax; accelF *= speedAcc; }
    };
}

TEST_CASE("PowerUps apply correct effects and deactivate") {
    FakeGame g;

    auto objGrow = Object3D::create();
    auto objFast = Object3D::create();
    auto objShrink = Object3D::create();

    PowerUp grow(objGrow, PowerUp::Type::Grow);
    PowerUp fast(objFast, PowerUp::Type::Faster);
    PowerUp shrink(objShrink, PowerUp::Type::Shrink);

    // Directly call apply through a tiny adapter to FakeGame signatures
    struct Adapter : Game {
        using Game::Game; // never constructed
        static void applyGrow(PowerUp& p, FakeGame& g){ p.apply(reinterpret_cast<Game&>(g)); }
    };

    // Manually emulate calls:
    grow.apply(reinterpret_cast<Game&>(g));
    REQUIRE(g.scale > 1.f);

    fast.apply(reinterpret_cast<Game&>(g));
    REQUIRE(g.maxF > 30.f);
    REQUIRE(g.accelF > 20.f);

    float before = g.scale;
    shrink.apply(reinterpret_cast<Game&>(g));
    REQUIRE(g.scale < before);

    // Deactivate hides object
    grow.hide();
    REQUIRE_FALSE(grow.active());
    REQUIRE_FALSE(objGrow->visible);
}