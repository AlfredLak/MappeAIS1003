#pragma once

namespace minbil {

    class CollisionSink {
    public:
        virtual ~CollisionSink() = default;

        virtual void onTreeCollision(float impactSpeed) = 0;
    };

}