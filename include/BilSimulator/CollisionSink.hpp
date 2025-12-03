#pragma once

namespace MyCar {

    struct CollisionSink {
        virtual ~CollisionSink() = default;
        virtual void onTreeCollision(float impactSpeed) = 0;
    };

}