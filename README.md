Car Simulator for candidate number 10079
===============

What it does and what it can do
-------------------------------
- Three different car models with individual tunings and max accelerations
- Keyboard controls using WASD for directions and SPACE for drifting
- A simple interface for the start and pause-screen
- Camera controller allowing the user to see the car from the back, from the side, and from the front, with smooth transitioning between perspectives
- A 3D environment
    - 2048x2048 textured ground with a track
    - Randomly scattered trees the car can collide with, circular collision radius that triggers a subtle crash animation
    - Three collectible power-ups with fitting models (grow, shrink, speed up)
- Unit tests for core components using Catch2


How it is used
--------------
In the interface:

Enter to start


W: accelerate forwards

S: decelerate/reverse

A: Turn left

D: Turn right


V: Change perspective


P: Pause

R: Reset

Top row keys 1-3 to switch between car models

Reflection
---------
Perks

I'm quite happy to have achieved a clear separation between the visual components and the physics components, allowing for easy testing of each component individually.
Another major perk of this is that visual slip when drifting could be derived from lateral velocity and yaw rate which gives a great feel to the drifting without having to do any physics
on a rigid body object. I also implemented a collision sink which helps for fully separating the game from the collision function; the collision work independently from the simulation itself.
which is also a perk when testing.
Additionally, I managed to get each update quite safe so the program can be run without fear of crashing. To achieve this I used clamped time steps and a cooldown on collision to avoid
repeated triggers, preventing the program from exploding and keeping the flow of events nice and tidy without a negative visual effect.
I was also quite happy with being able to implement testing across all core modules, and to get the tests working even after making changes to the structure or function of the project without
any major issues.
As for CMake, I was able to make a nice framework for copying all necessary DLLs into the current build, making sure there are no DLL errors when running the program with
either debug or release builds.

Improvements

The absolute main problem with my project is that I initially made it entirely on .hpp files in the include folder, with
main.cpp and .cpp files only in tests. When I later tried to split it properly into .cpp and .hpp, everything unraveled,
and even reverting to earlier commits didn’t fully save me. Only-headers is fine for tiny header-only libs, but for an 
app like this it’s dumb: it bloats compile times, hides link/ODR errors until too late, encourages circular includes,
and makes dependencies between translation units impossible to manage cleanly. The fix is simple next time—every class 
gets a small header (only interface + minimal forward declarations) and a .cpp that implements it, with private includes
staying in the .cpp.

The functionality of the collision could be much better. Right now it’s a circular radius per tree that hard-stops the 
car and triggers a tilt animation. That works with my current assets, but the hitbox will be wrong for larger or oddly 
shaped props. Worse, the car is checked against every tree each frame (O(n)), which is fine for a small map but scales 
poorly. A clear upgrade is a spatial partition where we only check for an object it is possible to collide with in a
limited area around the car, which would drastically reduce the number of checks per frame.

Another thing that should be improved is that main.cpp has way too many responsibilities: it sets up cameras,
builds vehicles, wires power-ups/trees, and owns the game loop. Better would be small components: a CameraController
that hides all view logic (already moving there), a VehicleFactory (done) that returns a rig, and a simple App class
that owns the loop and state, so main.cpp just wires and runs.

When swapping between car models while running, the simulation currently reconstructs on each swap. A nicer approach
would cache assets (meshes/materials) and swap the chassis node while keeping physics/input/camera intact; even a tiny
object pool for rigs would drop hitches.

Today I mix raw pointers with smart pointers. Next pass I would decide ownership once and document it: things the scene
truly owns live in smart pointers (use unique_ptr for a single clear owner; use shared_ptr only when multiple subsystems
must co-own), while other fields are non-owning raw pointers or references. Pass by reference when required,
use pointers only for optional args, consider weak_ptr for callbacks (e.g., Trees → CollisionSink),
avoid reinterpret_cast hacks in tests, and keep math values (like CarPose) as copies. This keeps lifetimes obvious,
prevents dangling use after swaps, and makes tests cleaner.

UML class diagram
----------------
![UML](mermaid_uml.png)

References
-----------
https://github.com/markaren/threepp

https://github.com/assimp/assimp

https://github.com/catchorg/Catch2


I have not copied the code of any other repository in this project. I have however used ChatGPT 5.0 as a helper throughout
the entire project for bugspotting and suggesting fixes, as well as commenting on code.