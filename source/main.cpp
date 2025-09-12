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
#include "fireworks.hpp"



int main(void)
{
    const Vector3 origin = {0,0,0};
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "IPhysicsEngine");

    bool cameraState = true;
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 20.0f, 20.0f, 20.0f }; // Camera position
    camera.target = origin;                             // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type


    SetTargetFPS(60);

    bool physicsState = true;
    
    IPhysicsEngine::RandomStore::Initialise();

        IPhysicsEngine::Vector3 high(0,20,0);
    IPhysicsEngine::Vector3 projectileVelocity(10.0f, 0, 0);
    IPhysicsEngine::Vector3 down(0,-9.81f,0);


    IPhysicsEngine::FireworkManager::Initialise();
    IPhysicsEngine::Firework* fireworks = IPhysicsEngine::FireworkManager::GetFireworks();
    IPhysicsEngine::FireworkManager::Create(8,1, NULL);


    std::vector<IPhysicsEngine::Particle*> particles;

    /*
    IPhysicsEngine::BallisticParticle* particle2 = new IPhysicsEngine::BallisticParticle(high, 0.1f, 1);
    particle2->SetVelocity(projectileVelocity);
    particles.push_back(particle2);
    IPhysicsEngine::BallisticParticle* particle3 = new IPhysicsEngine::BallisticParticle(high, 0.5f, 1);
    particle2->SetVelocity(projectileVelocity);
    particle3->SetAcceleration(down);
    particles.push_back(particle3);
    */

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
            // Particles
            particles.erase(std::remove_if(particles.begin(), particles.end(), 
            [duration](const auto& element) {
                return !element->Integrate(duration);
            }), particles.end());
            // Fireworks
            IPhysicsEngine::FireworkManager::Update(duration);
        }
        
        Vector3 velocity;
        Vector3 position;
        IPhysicsEngine::real age; 
        IPhysicsEngine::real damping; 

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

                for (IPhysicsEngine::Firework* firework = fireworks; firework < fireworks + IPhysicsEngine::FireworkManager::GetMaxFireworks(); firework++){
                    if(firework->GetType() == 0){
                        continue;
                    }
                    IPhysicsEngine::Vector3 iPosition = firework->GetPosition();
                    IPhysicsEngine::Vector3 iVelocity = firework->GetVelocity();
                    position = {iPosition.GetX(), iPosition.GetY(), iPosition.GetZ()};
                    velocity = {iVelocity.GetX(), iVelocity.GetY(), iVelocity.GetZ()};
                    age = firework->GetAge();
                    damping = firework->GetDamping();

                    DrawCube(position, 2.0f, 2.0f, 2.0f, RED);

                }
                
                DrawGrid(100, 1.0f);

               

            EndMode3D();

            DrawText(std::to_string(velocity.x).c_str(), 100, 100, 20, BLACK);
            DrawText(std::to_string(velocity.y).c_str(), 300, 100, 20, BLACK);
            DrawText(std::to_string(velocity.z).c_str(), 500, 100, 20, BLACK);
            DrawText(std::to_string(age).c_str(), 700, 100, 20, BLACK);
            DrawText(std::to_string(damping).c_str(), 900, 100, 20, BLACK);
            DrawText(std::to_string(position.x).c_str(), 100, 200, 20, BLACK);
            DrawText(std::to_string(position.y).c_str(), 300, 200, 20, BLACK);
            DrawText(std::to_string(position.z).c_str(), 500, 200, 20, BLACK);



        EndDrawing();
    }

    CloseWindow();

    return 0;
}
