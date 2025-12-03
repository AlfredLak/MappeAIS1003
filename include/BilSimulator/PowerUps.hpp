#pragma once
#include <threepp/threepp.hpp>
#include <memory>

namespace MyCar {

    class Game;

    class PowerUp {
    public:
        enum class Type { Grow, Faster, Shrink };

        explicit PowerUp(std::shared_ptr<threepp::Object3D> object, Type type)
            : object_(std::move(object)), type_(type) {}

        threepp::Object3D* object() { return object_.get(); }
        bool active() const { return active_; }
        Type type() const { return type_; }

        void hide() { active_ = false; if (object_) object_->visible = false; }

        void apply(Game& game) const;

    private:
        std::shared_ptr<threepp::Object3D> object_;
        Type  type_{Type::Grow};
        bool  active_{true};
    };

}