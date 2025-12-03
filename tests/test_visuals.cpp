#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <threepp/threepp.hpp>
#include <memory>

#include "BilSimulator/VehicleVisuals.hpp"

using namespace threepp;
using Catch::Approx;

TEST_CASE("VehicleVisuals spins wheels") {
    auto chassis = Object3D::create();

    std::vector<std::shared_ptr<Object3D>> owned;
    std::vector<Object3D*> wheelPtrs;
    for (int i = 0; i < 4; ++i) {
        owned.push_back(Object3D::create());
        wheelPtrs.push_back(owned.back().get());
    }

    MyCar::VehicleVisuals visuals(chassis.get(), wheelPtrs);

    MyCar::CarPose pose{};
    pose.wheelSpinDelta = 0.5f;

    visuals.apply(1.f / 60.f, pose);

    REQUIRE(wheelPtrs[0]->rotation.x != Approx(0.f));
}