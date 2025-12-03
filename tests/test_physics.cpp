#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "BilSimulator/VehiclePhysics.hpp"

using namespace MyCar;

static void stepFor(VehiclePhysics& phys, const VehiclePhysics::InputLite& in, float seconds, float dt=1.f/120.f) {
    int steps = static_cast<int>(seconds / dt);
    for (int i=0;i<steps;i++) (void)phys.step(in, dt);
}

TEST_CASE("VehiclePhysics sits still with no input") {
    VehiclePhysics phys;
    VehiclePhysics::InputLite in{};
    auto p0 = phys.position();
    stepFor(phys, in, 1.0f);
    auto p1 = phys.position();
    REQUIRE(p0.x == Catch::Approx(p1.x));
    REQUIRE(p0.z == Catch::Approx(p1.z));
}

TEST_CASE("VehiclePhysics moves forward with forward input") {
    VehiclePhysics phys;
    VehiclePhysics::InputLite in{};
    in.forward = true;
    stepFor(phys, in, 1.0f);
    auto pose = phys.step(in, 0.f);
    REQUIRE(pose.forwardVel > 0.f);
    REQUIRE(phys.position().z > 0.f);
}

TEST_CASE("VehiclePhysics reverses with backward input") {
    VehiclePhysics phys;
    VehiclePhysics::InputLite in{};
    in.backward = true;
    stepFor(phys, in, 1.0f);
    auto pose = phys.step(in, 0.f);
    REQUIRE(pose.forwardVel < 0.f);
    REQUIRE(phys.position().z < 0.f);
}

TEST_CASE("Drift increases lateral velocity") {
    VehiclePhysics phys;
    VehiclePhysics::InputLite in{};
    in.forward = true;
    stepFor(phys, in, 1.0f); // get speed
    in.forward = false;

    in.left = true;
    auto poseNoDrift = phys.step(in, 1.f/60.f);

    in.drift = true;
    auto poseDrift = phys.step(in, 1.f/60.f);

    REQUIRE(std::abs(poseDrift.lateralVelocity) > std::abs(poseNoDrift.lateralVelocity));
}