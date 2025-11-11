#include <iostream>
#include <chrono>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <memory>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "raylib.h"

#include "core.hpp"

#include "rigidbody.hpp"
#include "forcegenerator.hpp"
#include "world.hpp"

/*
#include "particle.hpp"
#include "ballistic.hpp"
#include "fireworks.hpp"
#include "particleforcegenerator.hpp"
#include "particleworld.hpp"
*/

int main(void)
{
    const Vector3 origin = {0,0,0};
    const int screenWidth = 1280;
    const int screenHeight = 720;

    #ifdef __APPLE__
    SetConfigFlags(FLAG_WINDOW_HIGHDPI);
    #endif
    
    std::string pauseButtonText = "#132#";

    InitWindow(screenWidth, screenHeight, "IPhysicsEngine");

    bool cameraState = true;
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 30.0f, 30.0f, 30.0f }; // Camera position
    camera.target = origin;                             // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    const IPhysicsEngine::real duration = 1.0L / 60.0L;
    bool showMessageBox = false;
    SetTargetFPS(60);

    bool physicsState = true;

    IPhysicsEngine::Vector3 high(0,10.0f,0);

    IPhysicsEngine::World world;

    IPhysicsEngine::World::Rigidbodies& rigidbodies = world.GetRigidBodies();

    IPhysicsEngine::Quaternion* quaternion = new IPhysicsEngine::Quaternion();
    quaternion->r = 1.0f;

    IPhysicsEngine::Matrix3* inverseInertiaTensor = new IPhysicsEngine::Matrix3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.1f);

    IPhysicsEngine::RigidBody* rigidbody = new IPhysicsEngine::RigidBody(high, *quaternion, 1.0f, 0.99f, 0.98f, *inverseInertiaTensor);
    IPhysicsEngine::Gravity* gravity = new IPhysicsEngine::Gravity(IPhysicsEngine::GravityEarth);

    world.GetRigidBodies().emplace_back(rigidbody);
    world.GetParticleForceRegistry().Add(rigidbody, gravity);

    /*
    IPhysicsEngine::ParticleWorld particleWorld(100,10);
    IPhysicsEngine::Particle* particle = new IPhysicsEngine::Particle(high, 0.5f, 1.0f);
    particleWorld.GetParticles().push_back(particle);
    IPhysicsEngine::ParticleGravity* particleGravity = new IPhysicsEngine::ParticleGravity(IPhysicsEngine::Gravity);

    particleWorld.GetParticleForceRegistry().Add(particle, particleGravity);
    IPhysicsEngine::ParticleGroundContactGenerator* particleGroundContactGenerator = new IPhysicsEngine::ParticleGroundContactGenerator();
    particleGroundContactGenerator->Init(&particleWorld.GetParticles(), 0.8f);
    particleWorld.GetParticleContactGenerator().push_back(particleGroundContactGenerator);
    */


    // Main loop
    while (!WindowShouldClose())
    {
        world.StartFrame();

        if (IsKeyPressed(KEY_SPACE)) {
            physicsState = !physicsState;
        }

        if (IsKeyPressed('Z')) {
            if(cameraState){
                DisableCursor();             
                cameraState = false;
            }
            else{
                cameraState = true;
                EnableCursor();             
            }
        }

        if(cameraState){
            UpdateCamera(&camera, CAMERA_PERSPECTIVE);
        }
        else{
            UpdateCamera(&camera, CAMERA_FREE);
        }
        if(physicsState){
            // Particles
            world.RunPhysics(duration);
        }

        BeginDrawing();

            ClearBackground(RAYWHITE);
           IPhysicsEngine::Vector3 iPosition;

            BeginMode3D(camera);
                
                IPhysicsEngine::World::Rigidbodies::iterator iterator = rigidbodies.begin();
                while (iterator != rigidbodies.end()){
                    iPosition =  (*iterator)->GetPosition();
                    Vector3 position = {iPosition.GetX(), iPosition.GetY(), iPosition.GetZ()};
                    DrawSphere(position, 1.0f, RED);
                    ++iterator;
                }
                
                /*
                for (IPhysicsEngine::Firework* firework = fireworks; firework < fireworks + IPhysicsEngine::FireworkManager::GetMaxFireworks(); firework++){
                    if (firework->GetType() == 0){
                        continue;
                    }
                    IPhysicsEngine::Vector3 iPosition = firework->GetPosition();
                    IPhysicsEngine::Vector3 iVelocity = firework->GetVelocity();
                    position = {iPosition.GetX(), iPosition.GetY(), iPosition.GetZ()};
                    velocity = {iVelocity.GetX(), iVelocity.GetY(), iVelocity.GetZ()};
                    age = firework->GetAge();
                    damping = firework->GetDamping();

                    DrawSphere(position, 0.25f, RED);
                    
                    count++;
                }
                */
               
                DrawGrid(100, 1.0f);


            EndMode3D();

            DrawText(std::to_string(iPosition.GetX()).c_str(), 300, 200, 10, BLACK);
            DrawText(std::to_string(iPosition.GetY()).c_str(), 400, 200, 10, BLACK);
            DrawText(std::to_string(iPosition.GetZ()).c_str(), 500, 200, 10, BLACK);

            // Pause Button
            if (GuiButton((Rectangle){ 340, 10, 30, 30 }, pauseButtonText.c_str())){
                if (physicsState){
                    pauseButtonText = "#131#";
                    physicsState = false;
                }
                else if (!physicsState){
                    pauseButtonText = "#132#";
                    physicsState = true;
                }
            }
            
            // Add Button
            if (GuiButton((Rectangle){ 380, 10, 30, 30 }, "#80#")) showMessageBox = true;

            if (showMessageBox)
            {
                Rectangle box = {10, 113, 400, 300}; // Position and size of the popup
                GuiGroupBox(box, "");
                DrawRectangle(box.x, box.y, box.width , box.height, Fade(GRAY, 0.5f));
                DrawRectangleLines(box.x, box.y, box.width , box.height, DARKGRAY);
                GuiLabel((Rectangle){ box.x + 10, box.y , 90, 30 }, "Add an Object");
                // Example widgets inside the box
                static bool toggle = false;
                static float volume = 0.5f;
                GuiCheckBox((Rectangle){box.x + 20, box.y + 40, 20, 20}, "Enable Feature", &toggle);
                volume = GuiSlider((Rectangle){box.x + 20, box.y + 80, 200, 20}, "Volume", TextFormat("%.2f", volume), &volume, 0.0f, 1.0f);

                // Close button
                if (GuiButton((Rectangle){box.x + box.width - 40, box.y + 10, 30, 30}, "#113#")) {
                    showMessageBox = false;
                }
            }



            DrawRectangle( 10, 10, 320, 93, Fade(GRAY, 0.5f));
            DrawRectangleLines( 10, 10, 320, 93, DARKGRAY);

            DrawText("Press Z to toggle camera control on and off", 20, 20, 10, BLACK);
            DrawText("Press Spacebar to toggle physics simulation on and off", 20, 40, 10, DARKGRAY);
            DrawText("- Mouse Wheel Pressed to Pan", 20, 60, 10, DARKGRAY);
            DrawText("- Z to zoom to (0, 0, 0)", 20, 80, 10, DARKGRAY);




        EndDrawing();
    }

    CloseWindow();

    return 0;
}

