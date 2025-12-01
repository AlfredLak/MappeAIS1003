#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/VehicleRig.hpp"
#include <threepp/threepp.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include "catch2/matchers/catch_matchers.hpp"

using namespace minbil;
using namespace threepp;

TEST_CASE("findWheels finds four wheel nodes by name") {
    auto root = Object3D::create();
    auto ch   = Object3D::create();
    root->add(ch);

    auto wheelFL = Object3D::create(); wheelFL->name = "wheel_FL"; ch->add(wheelFL);
    auto wheelFR = Object3D::create(); wheelFR->name = "wheel_FR"; ch->add(wheelFR);
    auto wheelBL = Object3D::create(); wheelBL->name = "wheel_BL"; ch->add(wheelBL);
    auto wheelBR = Object3D::create(); wheelBR->name = "wheel_BR"; ch->add(wheelBR);

    std::vector<Object3D*> wheels;
    findWheels(ch.get(), wheels);

    REQUIRE(wheels.size() == 4);
    REQUIRE_THAT(wheels[0]->name + wheels[1]->name + wheels[2]->name + wheels[3]->name,
                 Catch::Matchers::ContainsSubstring("wheel_"));
}