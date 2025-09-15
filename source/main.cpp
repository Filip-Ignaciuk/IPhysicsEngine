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
#include "particleforcegenerator.hpp"



int main(void)
{
    const Vector3 origin = {0,0,0};
    const int screenWidth = 1280;
    const int screenHeight = 720;
    #ifdef __APPLE__
    SetConfigFlags(FLAG_WINDOW_HIGHDPI);
    #endif
    
    InitWindow(screenWidth, screenHeight, "IPhysicsEngine");

    bool cameraState = true;
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 50.0f, 50.0f, 50.0f }; // Camera position
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
    IPhysicsEngine::Vector3 five(5,5,5);

    /*
    IPhysicsEngine::FireworkManager::Initialise();
    IPhysicsEngine::Firework* fireworks = IPhysicsEngine::FireworkManager::GetFireworks();
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    */
    std::vector<IPhysicsEngine::Particle*> particles;

    IPhysicsEngine::Particle* particle = new IPhysicsEngine::Particle(high, 1.0f, 1.0f);
    IPhysicsEngine::ParticleForceRegistry particleForceRegistry;
    IPhysicsEngine::ParticleGravity* particleGravity = new IPhysicsEngine::ParticleGravity(IPhysicsEngine::Gravity);
    particleForceRegistry.Add(particle, particleGravity);
    particles.emplace_back(particle);

    const IPhysicsEngine::real duration = 1.0L / 60.0L;
    int pay = 0;
    // Main game loop
    while (!WindowShouldClose())
    {
        int count = 0;
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

            particleForceRegistry.UpdateForces(duration);
            particles.erase(std::remove_if(particles.begin(), particles.end(), 
            [duration](const auto& element) {
                return !element->Integrate(duration);
            }), particles.end());
            // Fireworks
            if (pay == 0){
                pay = IPhysicsEngine::FireworkManager::Update(duration);

            }
            else{
                IPhysicsEngine::FireworkManager::Update(duration);
            }
            //fireworkPointer->Integrate(duration);
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

            DrawText(std::to_string(count).c_str(), 100, 100, 20, BLACK);




        EndDrawing();
    }

    CloseWindow();

    return 0;
}
