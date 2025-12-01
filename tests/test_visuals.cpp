#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/VehicleVisuals.hpp"
#include "BilSimulator/VehiclePhysics.hpp"
#include <threepp/threepp.hpp>

using namespace minbil;
using namespace threepp;

TEST_CASE("VehicleVisuals spins wheels and adds slip pose") {
    auto chassis = Object3D::create();
    auto w1 = Object3D::create(), w2 = Object3D::create();
    std::vector<Object3D*> wheels{ w1.get(), w2.get() };

    VehicleVisuals vis(chassis.get(), wheels);

    CarPose pose{};
    pose.wheelSpinDelta = 0.5f;
    pose.forwardVel = 10.f;
    pose.lateralVel = 2.f;
    pose.reversing = false;
    pose.drifting = false;
    pose.slipping = true;

    vis.apply(1.f/60.f, pose);

    REQUIRE(w1->rotation.x != 0.f);
    REQUIRE(w2->rotation.x != 0.f);
    // Expect some yaw offset applied to chassis
    REQUIRE(chassis->rotation.y != 0.f);
}