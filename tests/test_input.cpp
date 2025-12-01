#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/InputState.hpp"
#include <threepp/threepp.hpp>

using namespace minbil;
using namespace threepp;

TEST_CASE("InputState toggles on press/release") {
    InputState in;
    in.onKeyPressed({Key::W});
    REQUIRE(in.forward);

    in.onKeyReleased({Key::W});
    REQUIRE_FALSE(in.forward);

    in.onKeyPressed({Key::SPACE});
    REQUIRE(in.drift);
    in.onKeyReleased({Key::SPACE});
    REQUIRE_FALSE(in.drift);

    in.onKeyPressed({Key::NUM_2});
    REQUIRE(in.select2);
    in.onKeyReleased({Key::NUM_2});
    REQUIRE_FALSE(in.select2);
}