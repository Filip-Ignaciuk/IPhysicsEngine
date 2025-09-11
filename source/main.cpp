#include <iostream>
#include <chrono>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <memory>

#include "core.hpp"
#include "particle.hpp"
#include "ballistic.hpp"
#include "raylib.h"



int main(void)
{
    const Vector3 origin = {0,0,0};
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "IPhysicsEngine");

    bool cameraState = true;
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f }; // Camera position
    camera.target = origin;                             // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type


    SetTargetFPS(60);

    bool physicsState = true;
    




    IPhysicsEngine::Vector3 high(0,20,0);
    IPhysicsEngine::Vector3 projectileVelocity(10.0f, 0, 0);
    IPhysicsEngine::Vector3 down(0,-9.81f,0);
    std::vector<IPhysicsEngine::Particle*> particles;

    IPhysicsEngine::BallisticParticle* particle2 = new IPhysicsEngine::BallisticParticle(high, 0.1f, 1);
    particle2->SetVelocity(projectileVelocity);
    particle2->SetAcceleration(down);
    particles.push_back(particle2);
    IPhysicsEngine::BallisticParticle* particle3 = new IPhysicsEngine::BallisticParticle(high, 0.5f, 1);
    particle2->SetVelocity(projectileVelocity);
    particle3->SetAcceleration(down);
    particles.push_back(particle3);
    
 

    const IPhysicsEngine::real duration = 1.0L / 60.0L;

    // Main game loop
    while (!WindowShouldClose())
    {
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
            particles.erase(std::remove_if(particles.begin(), particles.end(), 
            [duration](const auto& element) {
                return !element->Integrate(duration); // your condition function
            }), particles.end());
        }
        
      
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                
                for (int i = 0; i < particles.size(); i++)
                {
                    
                    IPhysicsEngine::Vector3 iPosition = particles[i]->GetPosition();
                    Vector3 position = {iPosition.GetX(), iPosition.GetY(), iPosition.GetZ()};

                    DrawCube(position, 2.0f, 2.0f, 2.0f, RED);
                    DrawCubeWires(position, 2.0f, 2.0f, 2.0f, MAROON);

                }

                DrawGrid(10, 1.0f);

            EndMode3D();

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
