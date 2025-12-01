#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/Trees.hpp"
#include <threepp/threepp.hpp>

using namespace threepp;
using namespace minbil;
// Minimal fake Game recording collisions
struct FakeGame {
    int collisions{0};
    float lastSpeed{0.f};

    // must match signature used by Trees::update
    void onCollision(float speed) {
        ++collisions;
        lastSpeed = speed;
    }
};

TEST_CASE("Trees: no collision when car is outside radius") {
    Trees trees;

    // Place one tree far away
    auto farTree = Object3D::create();
    farTree->position.set(10.f, 0.f, 10.f);
    trees.add(farTree);

    // Car at origin
    auto car = Object3D::create();
    car->position.set(0.f, 0.f, 0.f);

    FakeGame g;
    // Call update (reinterpret cast to satisfy Trees::update(Game&))
    trees.update(reinterpret_cast<class Game&>(g), car.get(), /*speed*/5.f);

    REQUIRE(g.collisions == 0);
}

TEST_CASE("Trees: collision triggers exactly once when inside radius") {
    Trees trees;

    // Default radius in Trees is ~1.8 -> place within that
    auto nearTree = Object3D::create();
    nearTree->position.set(1.0f, 0.f, 0.0f);
    trees.add(nearTree);

    auto car = Object3D::create();
    car->position.set(0.f, 0.f, 0.f);

    FakeGame g;
    trees.update(reinterpret_cast<class Game&>(g), car.get(), /*speed*/7.5f);

    REQUIRE(g.collisions == 1);
    REQUIRE(g.lastSpeed == 7.5f);
}

TEST_CASE("Trees: multiple trees near car still call onCollision only once") {
    Trees trees;

    auto t1 = Object3D::create(); t1->position.set(1.0f, 0.f, 0.0f);
    auto t2 = Object3D::create(); t2->position.set(0.0f, 0.f, 1.0f);
    trees.add(t1);
    trees.add(t2);

    auto car = Object3D::create();
    car->position.set(0.f, 0.f, 0.f);

    FakeGame g;
    trees.update(reinterpret_cast<class Game&>(g), car.get(), 3.0f);

    REQUIRE(g.collisions == 1);   // Trees::update breaks after first hit
}