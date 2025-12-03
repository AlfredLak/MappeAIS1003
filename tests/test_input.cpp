#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/InputState.hpp"
#include <threepp/threepp.hpp>

using namespace MyCar;
using namespace threepp;

static KeyEvent makeKey(Key key) {
    return KeyEvent{key, 0, 0};
}

TEST_CASE("InputState toggles on press/release") {
    InputState in;

    in.onKeyPressed(makeKey(Key::W));
    REQUIRE(in.forward);
    in.onKeyReleased(makeKey(Key::W));
    REQUIRE_FALSE(in.forward);

    in.onKeyPressed(makeKey(Key::SPACE));
    REQUIRE(in.drift);
    in.onKeyReleased(makeKey(Key::SPACE));
    REQUIRE_FALSE(in.drift);

    in.onKeyPressed(makeKey(Key::NUM_2));
    REQUIRE(in.select2);
    in.onKeyReleased(makeKey(Key::NUM_2));
    REQUIRE_FALSE(in.select2);
}