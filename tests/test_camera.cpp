#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/CameraController.hpp"

using namespace MyCar;

TEST_CASE("CameraController cycles modes") {
    CameraController cc;
    REQUIRE(cc.mode() == CameraMode::Behind);

    cc.cycle(); REQUIRE(cc.mode() == CameraMode::Side);
    cc.cycle(); REQUIRE(cc.mode() == CameraMode::Front);
    cc.cycle(); REQUIRE(cc.mode() == CameraMode::Behind);
}

TEST_CASE("CameraController update tolerates null camera") {
    CameraController cc;
    cc.update({0,0,0}, 0.f);
    SUCCEED();
}