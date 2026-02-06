#include <iostream>
#include <chrono>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <memory>
#include "raylib.h"
#include "raymath.h"

#define RAYGUI_ICONS
#define RAYGUI_IMPLEMENTATION

#include <raylib.h>
#include <raygui.h>

#include "core.hpp"

#include "rigidbody/rigidbody.hpp"
#include "rigidbody/forcegenerator.hpp"
#include "rigidbody/world.hpp"

#include "object.hpp"
#include "component.hpp"
#include "geometry.hpp"

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
    const int screenWidth = 1272;
    const int screenHeight = 720;

    #ifdef __APPLE__
        SetConfigFlags(FLAG_WINDOW_HIGHDPI);

    #endif
    
    std::string pauseButtonText = "#132#";

    InitWindow(screenWidth, screenHeight, "IPhysicsEngine");
    GuiEnable();
    GuiLoadStyleDefault();
    
    bool cameraState = true;
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 30.0f, 30.0f, 30.0f }; // Camera position
    camera.target = origin;                             // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type



    SetTargetFPS(60);


    IPhysicsEngine::World world;

    

    
    /*
    IPhysicsEngine::ParticleWorld particleWorld(100,10);
    IPhysicsEngine::Particle* particle = new IPhysicsEngine::Particle(high, 0.5f, 1.0f);
    particleWorld.GetParticles().push_back(particle);
    IPhysicsEngine::ParticleGravity* particleGravity = new IPhysicsEngine::ParticleGravity(IPhysicsEngine::GravityEarth);
    IPhysicsEngine::Firework* firework = new IPhysicsEngine::Firework();
    IPhysicsEngine::FireworkManager* fireworkManager = new IPhysicsEngine::FireworkManager();
       

    particleWorld.GetParticleForceRegistry().Add(particle, particleGravity);
    IPhysicsEngine::ParticleGroundContactGenerator* particleGroundContactGenerator = new IPhysicsEngine::ParticleGroundContactGenerator();
    particleGroundContactGenerator->Init(&particleWorld.GetParticles(), 0.8f);
    particleWorld.GetParticleContactGenerator().push_back(particleGroundContactGenerator);
    */

    // GUI state
    bool showAddObjectBox = false;
    bool showAddMeshBox = false;
    bool showPreviewBox = false;
    bool showHelpBox = false;

    // Information associated with add object window
    IPhysicsEngine::Vector3* position;
    IPhysicsEngine::Quaternion* orientation;
    IPhysicsEngine::real mass;
    IPhysicsEngine::real linearDamping;
    IPhysicsEngine::real angularDamping;
    IPhysicsEngine::Matrix3* inverseInertiaTensor;
    Mesh mesh;
    Color color;
    IPhysicsEngine::real scale;

    // Main loop
    while (!WindowShouldClose())
    {
        //particleWorld.StartFrame();
        world.StartFrame();

        if (IsKeyPressed(KEY_SPACE)) {
            world.SetPhysicsState(!world.GetPhysicsState());
            if(world.GetPhysicsState()){
                pauseButtonText = "#131#";
            }
            else{
                pauseButtonText = "#132#";
            }
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
        if(world.GetPhysicsState()){
            // Particles
            //particleWorld.RunPhysics(duration);
            // RigidBodies
            world.RunPhysics();
        }

        BeginDrawing();

            ClearBackground(RAYWHITE);
            IPhysicsEngine::Vector3 iPosition;
            IPhysicsEngine::Quaternion iQuaternion;
            IPhysicsEngine::real iMass;
            IPhysicsEngine::Vector3 iVelocity;
            BeginMode3D(camera);
                /*
                IPhysicsEngine::ParticleWorld::Particles::iterator iterator = particleWorld.GetParticles().begin();
                while (iterator != particleWorld.GetParticles().end()){
                    iPosition =  (*iterator)->GetPosition();
                    Vector3 position = {iPosition.GetX(), iPosition.GetY(), iPosition.GetZ()};
                    DrawSphere(position, 1.0f, RED);
                    ++iterator;
                }
                */
                
                IPhysicsEngine::World::Objects::iterator iterator = world.GetObjects().begin();
                while (iterator != world.GetObjects().end()){
                    IPhysicsEngine::Object* object = *iterator;
                    IPhysicsEngine::RigidBody* rigidbody = object->GetComponent<IPhysicsEngine::RigidBody>();
                    IPhysicsEngine::Geometry* geometry = object->GetComponent<IPhysicsEngine::Geometry>();
                    iPosition =  rigidbody->GetPosition();
                    iQuaternion =  rigidbody->GetOrientation();
                    iMass = rigidbody->GetMass();
                    iVelocity = rigidbody->GetVelocity();
                    Matrix position = MatrixTranslate(iPosition.GetX(), iPosition.GetY(), iPosition.GetZ());
                    Vector4 quaternion{iQuaternion.i, iQuaternion.j, iQuaternion.k, iQuaternion.r};
                    Matrix rotation = QuaternionToMatrix(quaternion);




                    Model model = LoadModelFromMesh(*geometry->GetMesh());
                    model.transform = MatrixMultiply(rotation, position);
                    DrawModel(model, (Vector3){0,0,0}, geometry->GetScale(), geometry->GetColor());
                   
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



            
            
            // Add Button
            if (GuiButton((Rectangle){ 24, 24, 24, 24 }, "#80#")) {
                showAddObjectBox = true;
            }

            // Add Mesh Button
            if (GuiButton((Rectangle){ 72, 24, 24, 24 }, "#162#")) {

            }

            // Pause Button
            if (GuiButton((Rectangle){ 120, 24, 24, 24 }, pauseButtonText.c_str())){
                if (world.GetPhysicsState()){
                    pauseButtonText = "#131#";
                    world.SetPhysicsState(false);
                }
                else{
                    pauseButtonText = "#132#";
                    world.SetPhysicsState(true);
                }
            }

            // Help Button 193
            if (GuiButton((Rectangle){ 168, 24, 24, 24 },"#193#")){
                showHelpBox = !showHelpBox;
            }

            if (showAddObjectBox)
            {
                Rectangle box = {10, 113, 400, 300}; // Position and size of the popup
                GuiGroupBox(box, "");
                DrawRectangle(box.x, box.y, box.width , box.height, Fade(GRAY, 0.5f));
                DrawRectangleLines(box.x, box.y, box.width , box.height, DARKGRAY);
                GuiLabel((Rectangle){ box.x + 10, box.y , 90, 30 }, "Add an Object");

                /*I
                if(GuiTextBox((Rectangle){ box.x + 10, box.y , 90, 30}), "x"){

                }
                */
                // Close button
                if (GuiButton((Rectangle){box.x + box.width - 40, box.y + 10, 30, 30}, "#113#")) {
                    showAddObjectBox = false;
                }

                if (GuiButton((Rectangle){box.x + box.width - 40, box.y + 10, 30, 30}, "#113#")) {


                    // Creating the object
                    IPhysicsEngine::Object* object = new IPhysicsEngine::Object();
                    IPhysicsEngine::RigidBody* rigidbody = object->AddComponent<IPhysicsEngine::RigidBody>();
                    IPhysicsEngine::Geometry* geometry = object->AddComponent<IPhysicsEngine::Geometry>();
                    rigidbody->SetPosition(*position);
                    rigidbody->SetOrientation(*orientation);
                    rigidbody->SetMass(mass);
                    rigidbody->SetLinearDamping(linearDamping);
                    rigidbody->SetAngularDamping(angularDamping);
                    rigidbody->SetInverseInertiaTensor(*inverseInertiaTensor);

                    Mesh cubeMesh = GenMeshCube(1.0f, 1.0f, 1.0f);
                    geometry->SetMesh(&cubeMesh);
                    geometry->SetScale(1.0f);
                    geometry->SetColor(RED);
                    
                    world.AddObject(object);
                    //world.AddForceRegistry(object, gravity);

                }
            }
            else if (showAddMeshBox){

            }

            if(showPreviewBox){

            }

            if(showHelpBox){
                DrawRectangle( 936, 600, 312, 96, Fade(GRAY, 0.5f));
                DrawRectangleLines( 936, 600, 312, 96, DARKGRAY);

                DrawText("Press Z to toggle camera control on and off", 946, 610, 10, BLACK);
                DrawText("Press Spacebar to toggle physics simulation on and off", 946, 630, 10, DARKGRAY);
                DrawText("Add objects using the Add button", 946, 650, 10, DARKGRAY);
                DrawText("- Z to zoom to (0, 0, 0)", 946, 670, 10, DARKGRAY);
            }

            
            
            



        EndDrawing();
    }

    CloseWindow();

    return 0;
}

