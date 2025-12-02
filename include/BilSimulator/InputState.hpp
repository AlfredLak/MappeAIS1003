#pragma once
#include <threepp/threepp.hpp>

namespace minbil {

    class InputState : public threepp::KeyListener {
    public:
        bool forward{false}, backward{false}, left{false}, right{false}, drift{false};
        bool enter{false}, pause{false}, reset{false};
        bool camToggle{false};
        bool select1{false}, select2{false}, select3{false};

        void onKeyPressed(threepp::KeyEvent e) override {
            using threepp::Key;
            switch (e.key) {
                case Key::W: forward = true; break;
                case Key::S: backward = true; break;
                case Key::A: left = true; break;
                case Key::D: right = true; break;
                case Key::SPACE: drift = true; break;

                case Key::V: camToggle = true; break;

                case Key::ENTER: enter = true; break;
                case Key::P: pause = true; break;
                case Key::R: reset = true; break;

                case Key::NUM_1: select1 = true; break;
                case Key::NUM_2: select2 = true; break;
                case Key::NUM_3: select3 = true; break;
                default: break;
            }
        }

        void onKeyReleased(threepp::KeyEvent e) override {
            using threepp::Key;
            switch (e.key) {
                case Key::W: forward = false; break;
                case Key::S: backward = false; break;
                case Key::A: left = false; break;
                case Key::D: right = false; break;
                case Key::SPACE: drift = false; break;

                case Key::V: camToggle = false; break;

                case Key::ENTER: enter = false; break;
                case Key::P: pause = false; break;
                case Key::R: reset = false; break;

                case Key::NUM_1: select1 = false; break;
                case Key::NUM_2: select2 = false; break;
                case Key::NUM_3: select3 = false; break;
                default: break;
            }
        }
    };

}