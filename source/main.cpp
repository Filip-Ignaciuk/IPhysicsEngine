#include <iostream>
#include <chrono>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <memory>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

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
    
    std::string pauseButtonText = "#132#";

    InitWindow(screenWidth, screenHeight, "IPhysicsEngine");

    bool cameraState = true;
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 30.0f, 30.0f, 30.0f }; // Camera position
    camera.target = origin;                             // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    bool showMessageBox = false;
    SetTargetFPS(60);

    bool physicsState = true;
    
    IPhysicsEngine::RandomStore::Initialise();
    IPhysicsEngine::FireworkManager::Initialise();
    IPhysicsEngine::ParticleForceRegistry particleForceRegistry;

    // Particle Force Generators
    IPhysicsEngine::ParticleGravity* particleGravity = new IPhysicsEngine::ParticleGravity(IPhysicsEngine::Gravity);
    IPhysicsEngine::ParticleDrag* particleDrag = new IPhysicsEngine::ParticleDrag(0.6f, 0.5f);
    IPhysicsEngine::ParticleRealGravity* particleRealGravity = new IPhysicsEngine::ParticleRealGravity(particleForceRegistry.GetRegistrations(), 0.000000000066743015f);
    

    IPhysicsEngine::Vector3 high(0,10,0);
    IPhysicsEngine::Vector3 left(0,1,0);
    IPhysicsEngine::Vector3 right(0,-1,0);
    IPhysicsEngine::Vector3 projectileVelocity(10.0f, 0, 0);
    IPhysicsEngine::Vector3 down(0,-9.81f,0);
    IPhysicsEngine::Vector3 five(5,5,5);
    IPhysicsEngine::Vector3 ten(15,10,4);

    /*
    IPhysicsEngine::Firework* fireworks = IPhysicsEngine::FireworkManager::GetFireworks();
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    IPhysicsEngine::FireworkManager::Create(5,nullptr);
    */
    std::vector<IPhysicsEngine::Particle*> particles;

    IPhysicsEngine::Particle* particle1 = new IPhysicsEngine::Particle(left, 0.8f, 1.0f);
    IPhysicsEngine::Particle* particle2 = new IPhysicsEngine::Particle(five, 0.8f, 1.0f);
    IPhysicsEngine::Particle* particle3 = new IPhysicsEngine::Particle(ten, 0.8f, 1.0f);


    IPhysicsEngine::ParticleSpring* particleSpring1 = new IPhysicsEngine::ParticleSpring(particle2, 2.0f, 2.0f);
    IPhysicsEngine::ParticleSpring* particleSpring2 = new IPhysicsEngine::ParticleSpring(particle1, 2.0f, 2.0f);

    IPhysicsEngine::ParticleAnchoredBungee* particleanchoredSpring1 = new IPhysicsEngine::ParticleAnchoredBungee(high, 2.0f, 2.0f);

    //particleForceRegistry.Add(particle1, particleSpring1);
    //particleForceRegistry.Add(particle2, particleSpring2);
    particleForceRegistry.Add(particle3, particleanchoredSpring1);
    particleForceRegistry.Add(particle3, particleGravity);

    //particles.emplace_back(particle1);
    //particles.emplace_back(particle2);
    particles.emplace_back(particle3);



    const IPhysicsEngine::real duration = 1.0L / 60.0L;
    // Main game loop
    while (!WindowShouldClose())
    {
        int count = 0;
        int x1 = 0;
        int y1 = 0;
        int z1 = 0;
        int x2 = 0;
        int y2 = 0;
        int z2 = 0;

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
            //fireworkPointer->Integrate(duration);
            count = particles.size();
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
                    Vector3 high = {0,10,0};
                    if (i == 0){
                        x1 = iPosition.GetX();
                        y1 = iPosition.GetY();
                        z1 = iPosition.GetZ();
                    }
                    if (i == 1){
                        x2 = iPosition.GetX();
                        y2 = iPosition.GetY();
                        z2 = iPosition.GetZ();
                    }
                    DrawSphere(position, 1.0f, RED);
                    DrawLine3D(high, position, BLUE);
                    
                    

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
