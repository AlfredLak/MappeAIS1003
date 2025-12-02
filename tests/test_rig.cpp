#include <catch2/catch_test_macros.hpp>
#include <threepp/threepp.hpp>
#include <memory>

#include "BilSimulator/VehicleRig.hpp"  // or the header where you expose findWheels

using namespace threepp;

TEST_CASE("findWheels finds four wheels") {
    auto root = Object3D::create();

    std::vector<std::shared_ptr<Object3D>> owned; // keep alive
    const char* names[] = {"wheel_FL","wheel_FR","wheel_BL","wheel_BR"};
    for (auto n : names) {
        owned.push_back(Object3D::create());
        owned.back()->name = n;
        root->add(owned.back());
    }

    std::vector<Object3D*> out;
    minbil::findWheels(root.get(), out); // adjust to your helper’s namespace
    REQUIRE(out.size() == 4);
}