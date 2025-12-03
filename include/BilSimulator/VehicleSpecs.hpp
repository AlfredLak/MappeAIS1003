#pragma once
namespace MyCar {
    struct VehicleTuning { float accelF, accelB, maxF, maxB; };
    inline VehicleTuning tuningFor(const std::string& modelPath) {
        if (modelPath.find("tractor") != std::string::npos) return {14.f, 12.f, 18.f, -12.f};
        if (modelPath.find("sedan")   != std::string::npos) return {22.f, 20.f, 36.f, -28.f};
        /* default */                                       return {20.f, 18.f, 30.f, -25.f};
    }
}