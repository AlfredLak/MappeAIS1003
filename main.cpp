#include <threepp/threepp.hpp>
#include <threepp/loaders/AssimpLoader.hpp>
#include <chrono>
#include <random>
#include <memory>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace threepp;

// ===== input =====
class DriveInput : public KeyListener {
public:
    bool w{false}, s{false}, a{false}, d{false}, v{false}, space{false};
    bool enter{false}, esc{false}, one{false}, two{false}, three{false}, r{false};
    void onKeyPressed(KeyEvent e) override {
        if (e.key==Key::W) w=true; if (e.key==Key::S) s=true;
        if (e.key==Key::A) a=true; if (e.key==Key::D) d=true;
        if (e.key==Key::V) v=true; if (e.key==Key::SPACE) space=true;
        if (e.key==Key::ENTER) enter=true; if (e.key==Key::ESCAPE) esc=true;
        if (e.key==Key::KP_1) one=true; if (e.key==Key::KP_2) two=true; if (e.key==Key::KP_3) three=true;
        if (e.key==Key::R) r=true;
    }
    void onKeyReleased(KeyEvent e) override {
        if (e.key==Key::W) w=false; if (e.key==Key::S) s=false;
        if (e.key==Key::A) a=false; if (e.key==Key::D) d=false;
        if (e.key==Key::V) v=false; if (e.key==Key::SPACE) space=false;
        if (e.key==Key::ENTER) enter=false; if (e.key==Key::ESCAPE) esc=false;
        if (e.key==Key::KP_1) one=false; if (e.key==Key::KP_2) two=false; if (e.key==Key::KP_3) three=false;
        if (e.key==Key::R) r=false;
    }
};

// ===== helpers =====
static void camThird(PerspectiveCamera& cam,const Vector3& p,float yaw,float back=6,float h=3){
    cam.position.set(p.x-std::sin(yaw)*back, p.y+h, p.z-std::cos(yaw)*back); cam.lookAt(p);
}
static void camFront(PerspectiveCamera& cam,const Vector3& p,float yaw,float front=6,float h=3){
    cam.position.set(p.x+std::sin(yaw)*front, p.y+h, p.z+std::cos(yaw)*front); cam.lookAt(p);
}
static void camSide(PerspectiveCamera& cam,const Vector3& p,float yaw,float side=5,float h=3){
    float sy=yaw-math::PI/2.f; cam.position.set(p.x-std::sin(sy)*side, p.y+h, p.z-std::cos(sy)*side); cam.lookAt(p);
}
static void findWheels(Object3D* root, std::vector<Object3D*>& out){
    const char* names[]={"wheel_FL","wheel_FR","wheel_BL","wheel_BR"};
    root->traverse([&](Object3D& o){ for(auto n:names) if(o.name==n){ out.push_back(&o); break; }});
}

// ===== pose =====
struct CarPose {
    Vector3 pos{};
    float yaw{0};
    float vf{0}, vr{0};
    float yawRate{0};
    bool reversing{false};
    bool drift{false};
    bool normalSlip{false};
    float wheelSpinDelta{0};
};

class Game; // fwd

// ===== trees =====
class Trees {
public:
    void add(std::shared_ptr<Object3D> t){ trees_.push_back(std::move(t)); }
    void update(Game& g, Object3D* car, float currentSpeed);
private:
    std::vector<std::shared_ptr<Object3D>> trees_;
    float radius_{1.8f};
};

// ===== physics =====
class CarPhysics {
public:
    float accelF=20, accelB=18, brake=30, drag=2.5f, vMaxF=30, vMaxB=-25;
    float turnMax=1.05f, turnAcc=3.8f, turnFric=5.2f;
    float revScale=0.30f, revAcc=2.0f, revFric=6.5f, revClamp=0.30f, revGrip=24.f;
    float gripN=12.f, gripD=1.5f, driftKick=80.f, driftMinV=2.f, driftGripSpeed=0.08f, gripDMin=0.8f;
    float normGripFactor=0.45f, normKick=18.f;
    float airDrag=0.05f, wheelR=0.35f;

    CarPose step(const DriveInput& in, float dt){
        CarPose P;
        float fx=std::sin(yaw_), fz=std::cos(yaw_), rx=std::cos(yaw_), rz=-std::sin(yaw_);
        float vf = vx_*fx + vz_*fz;
        float vr = vx_*rx + vz_*rz;

        float inputTurn = (in.a?+turnMax:0.f) + (in.d?-turnMax:0.f);
        bool onlyS = in.s && !in.w && !in.a && !in.d;
        bool reversing = (vf < -0.05f);
        float targetTurn = reversing ? (revScale*inputTurn) : inputTurn;
        if (onlyS) targetTurn = 0.f;

        float resp = (std::abs(targetTurn)>0)? (reversing?revAcc:turnAcc) : (reversing?revFric:turnFric);
        float alpha = 1.f-std::exp(-resp*dt);
        yawRate_ += (targetTurn - yawRate_) * alpha;
        if (reversing){ if (yawRate_> revClamp) yawRate_= revClamp; if (yawRate_<-revClamp) yawRate_=-revClamp; }

        bool braking = in.s && (vf>0.05f);
        if (in.w) vf += accelF*dt;
        if (in.s){
            if (vf>0){ vf -= brake*dt; if (vf<0) vf=0; }
            else { vf -= accelB*dt; }
        }

        if (!in.w && !in.s){
            if (vf>0){ vf -= drag*dt; if (vf<0) vf=0; }
            else if (vf<0){ vf += drag*dt; if (vf>0) vf=0; }
        }
        vf = std::clamp(vf, vMaxB, vMaxF);

        bool allowDrift = in.space && (vf>0.05f) && !onlyS;
        bool normalSlip = !allowDrift && (vf>0.05f) && (std::abs(yawRate_)>1e-4f) && !reversing;

        float grip = gripN;
        if (allowDrift){
            float sf = std::clamp(std::abs(vf)*driftGripSpeed,0.f,3.f);
            grip = std::max(gripDMin, gripD/(1.f+sf));
            float sgn = (yawRate_>=0)? 1.f : -1.f;
            if (vf>driftMinV){
                float g = 0.3f + 0.7f*std::min(std::abs(vf)/20.f,1.f);
                vr += sgn * driftKick * std::abs(yawRate_) * g * dt;
            }
        } else if (normalSlip){
            float sp = std::clamp(std::abs(vf)/15.f,0.f,1.5f);
            grip = gripN / (1.f + normGripFactor*sp);
            float sgn = (yawRate_>=0)? 1.f : -1.f;
            vr += sgn * normKick * std::abs(yawRate_) * (0.5f+0.5f*sp) * dt;
        }

        if (reversing && !(in.a||in.d)) grip = revGrip;
        if (braking){ grip = std::max(grip, 40.f); yawRate_ *= (1.f - std::min(1.f, 8.f*dt)); }

        float latA = 1.f-std::exp(-grip*dt);
        vr += (0.f - vr) * latA;

        float air = std::max(0.f, 1.f - airDrag*dt);
        vf*=air; vr*=air;

        vx_ = fx*vf + rx*vr;
        vz_ = fz*vf + rz*vr;
        pos_.x += vx_*dt; pos_.z += vz_*dt;

        bool moving = (vx_*vx_+vz_*vz_ > 1e-6f);
        bool revNoTurn = reversing && !(in.a||in.d);
        bool freezeAlign = !moving || revNoTurn;
        float desiredYaw = moving ? std::atan2(vx_, vz_) : yaw_;
        float alignRate = (reversing? 0.f : (allowDrift? 1.0f : 6.0f));
        float aY = freezeAlign? 0.f : (1.f - std::exp(-alignRate*dt));

        yaw_ += yawRate_*dt;
        if (!freezeAlign){
            float d = desiredYaw - yaw_;
            while (d> math::PI) d -= 2*math::PI;
            while (d<-math::PI) d += 2*math::PI;
            yaw_ += d * aY;
        }

        P.pos = pos_; P.yaw = yaw_; P.vf=vf; P.vr=vr; P.yawRate=yawRate_;
        P.reversing = reversing; P.drift = allowDrift; P.normalSlip = normalSlip;
        P.wheelSpinDelta = (wheelR>0)? (vf/wheelR)*dt : 0.f;
        return P;
    }

    void boostMax(float s){ vMaxF*=s; }
    void boostAccel(float s){ accelF*=s; }
    void hardStop(){ vx_=0; vz_=0; }
    const Vector3& pos() const { return pos_; }
    void setPos(const Vector3& p){ pos_=p; }
    float speed() const { return std::sqrt(vx_*vx_ + vz_*vz_); }

private:
    Vector3 pos_{0,0,0};
    float vx_{0}, vz_{0};
    float yaw_{0}, yawRate_{0};
};

// ===== visuals =====
class CarVisual {
public:
    CarVisual(Object3D* chassis, const std::vector<Object3D*>& wheels)
        : ch_(chassis), wheels_(wheels) {}

    float yawDrift=0.6f, offDrift=0.6f;
    float yawNorm =0.35f, offNorm =0.35f;
    float followYaw=10.f, followOff=12.f;

    void crashTilt(){ tiltTimer_ = tiltDur_; }

    void apply(float dt, const CarPose& P){
        for (auto* w:wheels_) w->rotation.x += P.wheelSpinDelta;

        float yawVisTarget=0.f, offVisTarget=0.f;
        if (!P.reversing){
            float safe = (std::abs(P.vf)>1e-3f)? P.vf : (P.vf>=0? 1e-3f:-1e-3f);
            float slip = std::atan2(P.vr, safe);
            if (P.drift){
                float boost = 1.f + 0.8f*std::min(std::abs(P.vf)/15.f, 1.5f);
                slip *= boost;
                yawVisTarget =  yawDrift * slip;
                offVisTarget = -offDrift * std::clamp(P.vr/10.f,-1.f,1.f);
            } else if (P.normalSlip){
                float boost = 1.f + 0.55f*std::min(std::abs(P.vf)/15.f, 1.25f);
                slip *= boost;
                yawVisTarget =  yawNorm * slip;
                offVisTarget = -offNorm * std::clamp(P.vr/10.f,-1.f,1.f);
            }
        }
        float aY = 1.f-std::exp(-followYaw*dt);
        yawVis_ += (yawVisTarget - yawVis_) * aY;
        ch_->rotation.y = yawVis_;

        float aO = 1.f-std::exp(-followOff*dt);
        offVis_ += (offVisTarget - offVis_) * aO;
        ch_->position.set(offVis_, 0, 0);

        if (tiltTimer_ > 0.f){
            tiltTimer_ -= dt;
            float t = std::max(0.f, tiltTimer_ / tiltDur_);
            float angle = tiltMax_ * t * (2.f - t);
            ch_->rotation.x = angle;
        } else {
            ch_->rotation.x *= std::exp(-10.f*dt);
        }
    }

private:
    Object3D* ch_{};
    std::vector<Object3D*> wheels_;
    float yawVis_{0}, offVis_{0};

    float tiltTimer_{0.f};
    float tiltDur_{0.15f};
    float tiltMax_{0.10f};
};

// ===== power-ups =====
class PowerUp {
public:
    enum class Type { Grow, Faster, Shrink };
    PowerUp(std::shared_ptr<Object3D> obj, Type t): obj_(std::move(obj)), t_(t) {}
    Object3D* obj(){ return obj_.get(); }
    bool active() const { return on_; }
    void apply(class Game& g);
    void hide(){ on_=false; if(obj_) obj_->visible=false; }
private:
    std::shared_ptr<Object3D> obj_;
    Type t_;
    bool on_{true};
};
class PowerUps {
public:
    void add(std::shared_ptr<Object3D> o, PowerUp::Type t){ v_.emplace_back(std::move(o),t); }
    void update(Game& g, Object3D* car){
        auto p=car->position;
        for(auto& pu:v_){
            if(!pu.active()) continue;
            auto* o=pu.obj(); if(!o) continue;
            float dx=p.x-o->position.x, dz=p.z-o->position.z;
            if(dx*dx+dz*dz < 2.25f){ pu.apply(g); pu.hide(); }
        }
    }
private:
    std::vector<PowerUp> v_;
};

// ===== game =====
class Game {
public:
    Game(Canvas* canv, GLRenderer* rend, Scene* scn, PerspectiveCamera* cam,
         Object3D* carRoot, Object3D* chassis, const std::vector<Object3D*>& wheels,
         DriveInput* in, PowerUps* pus, Trees* trees)
        : canvas_(canv), renderer_(rend), scene_(scn), camera_(cam),
          carRoot_(carRoot), input_(in), pwr_(pus), trees_(trees), vis_(chassis,wheels) {
        camThird(*camera_, carRoot_->position, 0.f);
        last_ = clk_.getElapsedTime();
        lastSafe_ = carRoot_->position;
    }

    void update(){
        double now=clk_.getElapsedTime();
        float dt = float(now-last_); last_=now; if(dt>0.033f) dt=0.033f;

        lastSafe_ = carRoot_->position;

        CarPose pose = phys_.step(*input_, dt);

        carRoot_->position.copy(pose.pos);
        carRoot_->rotation.y = pose.yaw;

        vis_.apply(dt, pose);

        if (pwr_)   pwr_->update(*this, carRoot_);
        if (trees_) trees_->update(*this, carRoot_, phys_.speed());

        if (input_->v && !prevV_) side_=!side_;
        prevV_=input_->v;

        if (side_) camSide(*camera_, carRoot_->position, pose.yaw);
        else if (pose.reversing) camFront(*camera_, carRoot_->position, pose.yaw);
        else camThird(*camera_, carRoot_->position, pose.yaw);

        renderer_->render(*scene_, *camera_);
    }

    void growCar(float f){ carRoot_->scale.multiplyScalar(f); }
    void faster(float sMax,float sAcc){ phys_.boostMax(sMax); phys_.boostAccel(sAcc); }

    void onCollision(float impactSpeed){
        const double now = clk_.getElapsedTime();
        if (impactSpeed < crashSpeedThreshold_) {
            phys_.hardStop(); phys_.setPos(lastSafe_); carRoot_->position.copy(lastSafe_);
            return;
        }
        if (now - lastCrashTime_ < crashCooldown_) {
            phys_.hardStop(); phys_.setPos(lastSafe_); carRoot_->position.copy(lastSafe_);
            return;
        }
        lastCrashTime_ = now;
        phys_.hardStop(); phys_.setPos(lastSafe_); carRoot_->position.copy(lastSafe_);
        vis_.crashTilt();
    }

private:
    Canvas* canvas_; GLRenderer* renderer_; Scene* scene_; PerspectiveCamera* camera_;
    Object3D* carRoot_; DriveInput* input_; PowerUps* pwr_; Trees* trees_;
    CarPhysics phys_; CarVisual vis_;
    Clock clk_; double last_{0.0};
    bool side_{false}, prevV_{false};
    Vector3 lastSafe_;
    double lastCrashTime_{-1.0};
    float crashCooldown_{0.25f};
    float crashSpeedThreshold_{2.5f};
};

// power-up behavior
void PowerUp::apply(Game& g){
    switch(t_){
        case Type::Grow:   g.growCar(1.4f); break;
        case Type::Faster: g.faster(1.35f,1.3f); break;
        case Type::Shrink: g.growCar(0.7f); break;
    }
}

// Trees::update after Game is complete
void Trees::update(Game& g, Object3D* car, float currentSpeed){
    auto p=car->position;
    for (auto& t: trees_){
        auto* o=t.get(); if(!o) continue;
        float dx=p.x-o->position.x, dz=p.z-o->position.z;
        if (dx*dx+dz*dz < radius_*radius_){
            g.onCollision(currentSpeed);
            break;
        }
    }
}

// ===== simple UI state (Phase 1) =====
enum class GameState { Menu, VehicleSelect, Playing, Paused };
struct GameUI { GameState state{GameState::Menu}; int vehicleIndex{0}; };

// ===== main =====
int main(){
    Canvas canvas("threepp driving car");
    GLRenderer renderer(canvas.size());
    Scene scene; scene.background = Color::skyblue;
    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 1000.f);

    auto plane = Mesh::create(PlaneGeometry::create(2000,2000), MeshPhongMaterial::create());
    plane->rotation.x = -math::PI/2.f; scene.add(plane);
    auto grid = GridHelper::create(2000,2000, Color::black, Color::darkgray);
    grid->position.y=0.01f; scene.add(grid);
    scene.add(HemisphereLight::create(Color::white, Color::gray,1.0f));
    auto dir=DirectionalLight::create(Color::white,0.8f); dir->position.set(5,10,7); scene.add(dir);

    AssimpLoader loader;

    // car rig
    auto model = loader.load("suv_model.glb");
    auto carRoot = Object3D::create();
    auto chassis = Object3D::create();
    chassis->add(model);
    scene.add(carRoot);
    carRoot->add(chassis);
    carRoot->position.y=0;

    std::vector<Object3D*> wheels; findWheels(chassis.get(), wheels);

    // power-ups
    class PowerUps pums;
    std::vector<std::string> files={"power_up1.glb","power_up2.glb","power_up3.glb"};
    std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<float> dxy(-950.f,950.f);
    std::uniform_int_distribution<int> dti(0,2);
    for(int i=0;i<120;i++){
        int t=dti(rng);
        auto o=loader.load(files[t]);
        o->position.set(dxy(rng),0,dxy(rng));
        scene.add(o);
        PowerUp::Type ty = (t==0)?PowerUp::Type::Grow:(t==1)?PowerUp::Type::Faster:PowerUp::Type::Shrink;
        pums.add(std::move(o), ty);
    }

    // trees
    Trees trees;
    for(int i=0;i<150;i++){
        auto t=loader.load("tree_model.glb");
        t->position.set(dxy(rng),0,dxy(rng));
        scene.add(t);
        trees.add(std::move(t));
    }

    DriveInput input; canvas.addKeyListener(input);
    Game game(&canvas,&renderer,&scene,&camera, carRoot.get(), chassis.get(), wheels, &input, &pums, &trees);

    // Phase 1 UI state
    GameUI ui;

    WindowSize last=canvas.size();
    canvas.animate([&]{
        auto s=canvas.size();
        if(s.width()!=last.width()||s.height()!=last.height()){
            renderer.setSize(s);
            camera.aspect=float(s.width())/float(s.height());
            last=s;
        }

        switch (ui.state) {
            case GameState::Menu: {
                // “menu”: show scene, wait for keys
                renderer.render(scene, camera);
                // ENTER: start
                if (input.enter) ui.state = GameState::Playing;
                else if (input.two) ui.state = GameState::VehicleSelect;
                break;
            }
            case GameState::VehicleSelect: {
                renderer.render(scene, camera);
                if (input.one)  ui.vehicleIndex = 0;
                if (input.two)  ui.vehicleIndex = 1;
                if (input.three)ui.vehicleIndex = 2;
                if (input.enter) {
                    ui.state = GameState::Playing;
                } else if (input.esc) {
                    ui.state = GameState::Menu;
                }
                break;
            }
            case GameState::Playing: {
                game.update();
                if (input.esc) ui.state = GameState::Paused;
                break;
            }
            case GameState::Paused: {
                renderer.render(scene, camera);
                if (input.enter) ui.state = GameState::Playing;
                if (input.esc)   ui.state = GameState::Menu;
                break;
            }
        }
    });

    return 0;
}