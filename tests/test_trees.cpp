#include <catch2/catch_test_macros.hpp>
#include "BilSimulator/Trees.hpp"
#include "BilSimulator/CollisionSink.hpp"
#include <threepp/threepp.hpp>

using namespace threepp;
using namespace minbil;

struct FakeSink : public CollisionSink {
    int hits{0};
    float lastSpeed{0.f};
    void onTreeCollision(float impactSpeed) override {
        ++hits;
        lastSpeed = impactSpeed;
    }
};

TEST_CASE("Trees: no collision when car is outside radius") {
    Trees trees;
    FakeSink sink;
    trees.setCollisionSink(&sink);

    auto farTree = Object3D::create();
    farTree->position.set(10.f, 0.f, 10.f);
    trees.add(farTree);

    auto car = Object3D::create();
    car->position.set(0.f, 0.f, 0.f);

    trees.update(car.get(), /*speed*/5.f);

    REQUIRE(sink.hits == 0);
}

TEST_CASE("Trees: collision triggers exactly once when inside radius") {
    Trees trees;
    FakeSink sink;
    trees.setCollisionSink(&sink);

    auto nearTree = Object3D::create();
    nearTree->position.set(1.0f, 0.f, 0.0f);
    trees.add(nearTree);

    auto car = Object3D::create();
    car->position.set(0.f, 0.f, 0.f);

    trees.update(car.get(), /*speed*/7.5f);

    REQUIRE(sink.hits == 1);
    REQUIRE(sink.lastSpeed == 7.5f);
}

TEST_CASE("Trees: multiple nearby trees still notify only once per update") {
    Trees trees;
    FakeSink sink;
    trees.setCollisionSink(&sink);

    auto t1 = Object3D::create(); t1->position.set(1.0f, 0.f, 0.0f);
    auto t2 = Object3D::create(); t2->position.set(0.0f, 0.f, 1.0f);
    trees.add(t1);
    trees.add(t2);

    auto car = Object3D::create();
    car->position.set(0.f, 0.f, 0.f);

    trees.update(car.get(), 3.0f);

    REQUIRE(sink.hits == 1);
}