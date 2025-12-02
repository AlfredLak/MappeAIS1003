#pragma once

namespace minbil {

    struct CollisionSink {
        virtual ~CollisionSink() = default;
        virtual void onTreeCollision(float impactSpeed) = 0;
    };

}