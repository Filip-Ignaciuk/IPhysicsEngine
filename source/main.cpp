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

#include "components/rigidbody.hpp"
#include "forcegenerator.hpp"
#include "world.hpp"

#include "collidebroad.hpp"
#include "collidenarrow.hpp"
#include "object.hpp"
#include "components/component.hpp"
#include "components/geometry.hpp"
#include "languagemanager.hpp"
#include "meshmanager.hpp"

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
    std::string fileDir = "resources/en-gb.json";
    IPhysicsEngine::LanguageManager::LoadLanguage(fileDir);
    IPhysicsEngine::MeshManager::LoadDefaults();

    IPhysicsEngine::Object* object = new IPhysicsEngine::Object();
    IPhysicsEngine::RigidBody* rigidbody = object->AddComponent<IPhysicsEngine::RigidBody>();
    IPhysicsEngine::Geometry* geometry = object->AddComponent<IPhysicsEngine::Geometry>();
    geometry->SetMesh(IPhysicsEngine::MeshManager::GetMesh("Cube"));
    geometry->SetColor(Color(RED));
    world.AddObject(object);

    
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

    // Adding Objects Window State
    static bool isValidData = false;

    static bool isBufferXCoordinateEdited = false;
    static bool isBufferYCoordinateEdited = false;
    static bool isBufferZCoordinateEdited = false;
    
    static bool isBufferXOrientationEdited = false;
    static bool isBufferYOrientationEdited = false;
    static bool isBufferZOrientationEdited = false;

    static bool isBufferMassEdited = false;

    static bool isBufferLinearDampingEdited = false;
    static bool isBufferAngularDampingEdited = false;

    static bool isMeshDropDownActive = false;


    char textBufferXCoordinate[64] = "";
    char textBufferYCoordinate[64] = "";
    char textBufferZCoordinate[64] = "";
    char textBufferXOrientation[64] = "";
    char textBufferYOrientation[64] = "";
    char textBufferZOrientation[64] = "";
    char textBufferMass[64] = "";
    char textBufferLinearDamping[64] = "";
    char textBufferAngularDamping[64] = "";

    char textBuffer1InverseInertiaTensor[64] = "";
    char textBuffer2InverseInertiaTensor[64] = "";
    char textBuffer3InverseInertiaTensor[64] = "";
    char textBuffer4InverseInertiaTensor[64] = "";
    char textBuffer5InverseInertiaTensor[64] = "";
    char textBuffer6InverseInertiaTensor[64] = "";
    char textBuffer7InverseInertiaTensor[64] = "";
    char textBuffer8InverseInertiaTensor[64] = "";
    char textBuffer9InverseInertiaTensor[64] = "";

    int dropDownSelectedMesh = 0;

    // Information associated with add object window

    IPhysicsEngine::Vector3 position;
    IPhysicsEngine::Quaternion orientation;
    IPhysicsEngine::real mass;
    IPhysicsEngine::real linearDamping;
    IPhysicsEngine::real angularDamping;
    IPhysicsEngine::Matrix3 inverseInertiaTensor;
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
                    Matrix position = MatrixTranslate(iPosition.x, iPosition.y, iPosition.z);
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
                showAddMeshBox = true;
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
                Rectangle box = {24, 72, 408, 408}; // Position and size of the popup
                showAddObjectBox = !GuiWindowBox(box, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.title").c_str());

                GuiSetStyle(LABEL, TEXT_ALIGNMENT_VERTICAL, TEXT_ALIGN_MIDDLE);

                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

                GuiLabel((Rectangle){ box.x + 24, box.y + 24, 96, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.positiontitle").c_str());

                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_CENTER);


                GuiLabel((Rectangle){ box.x + 24, box.y + 48, 24, 24 }, "X");
                if(GuiTextBox((Rectangle){ box.x + 48, box.y + 48, 96, 24 }, textBufferXCoordinate, 64, isBufferXCoordinateEdited)){
                    isBufferXCoordinateEdited = !isBufferXCoordinateEdited;
                }
                
                GuiLabel((Rectangle){ box.x + 144, box.y + 48, 24, 24 }, "Y");
                if(GuiTextBox((Rectangle){ box.x + 168, box.y + 48, 96, 24 }, textBufferYCoordinate, 64, isBufferYCoordinateEdited)){
                    isBufferYCoordinateEdited = !isBufferYCoordinateEdited;
                }

                GuiLabel((Rectangle){ box.x + 264, box.y + 48, 24, 24 }, "Z");
                if(GuiTextBox((Rectangle){ box.x + 288, box.y + 48, 96, 24 }, textBufferZCoordinate, 64, isBufferZCoordinateEdited)){
                    isBufferZCoordinateEdited = !isBufferZCoordinateEdited;
                }
                
                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

                GuiLabel((Rectangle){ box.x + 24, box.y + 72, 96, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.orientationtitle").c_str());

                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_CENTER);

                GuiLabel((Rectangle){ box.x + 24, box.y + 96, 24, 24 }, "X");
                if(GuiTextBox((Rectangle){ box.x + 48, box.y + 96, 96, 24 }, textBufferXOrientation, 64, isBufferXOrientationEdited)){
                    isBufferXOrientationEdited = !isBufferXOrientationEdited;
                }
                                
                GuiLabel((Rectangle){ box.x + 144, box.y + 96, 24, 24 }, "Y");
                if(GuiTextBox((Rectangle){ box.x + 168, box.y + 96, 96, 24 }, textBufferYOrientation, 64, isBufferYOrientationEdited)){
                    isBufferYOrientationEdited = !isBufferYOrientationEdited;
                }

                GuiLabel((Rectangle){ box.x + 264, box.y + 96, 24, 24 }, "Z");
                if(GuiTextBox((Rectangle){ box.x + 288, box.y + 96, 96, 24 }, textBufferZOrientation, 64, isBufferZOrientationEdited)){
                    isBufferZOrientationEdited = !isBufferZOrientationEdited;
                }

                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

                GuiLabel((Rectangle){ box.x + 24, box.y + 120, 168, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.masstitle").c_str());


                if(GuiTextBox((Rectangle){ box.x + 24, box.y + 144, 168, 24 }, textBufferMass, 64, isBufferMassEdited)){
                    isBufferMassEdited = !isBufferMassEdited;
                }
                
                GuiLabel((Rectangle){ box.x + 216, box.y + 120, 168, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.lineardampingtitle").c_str());

                if(GuiTextBox((Rectangle){ box.x + 216, box.y + 144, 168, 24 }, textBufferLinearDamping, 64, isBufferLinearDampingEdited)){
                    isBufferLinearDampingEdited = !isBufferLinearDampingEdited;
                }

                GuiLabel((Rectangle){ box.x + 24, box.y + 168, 168, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.angulardampingtitle").c_str());

                if(GuiTextBox((Rectangle){ box.x + 24, box.y + 192, 168, 24 }, textBufferAngularDamping, 64, isBufferAngularDampingEdited)){
                    isBufferAngularDampingEdited = !isBufferAngularDampingEdited;
                }

                std::string dropDownSelection;
                std::vector<std::string> meshStrings = IPhysicsEngine::MeshManager::GetMeshStrings();
                for (size_t i = 0; i < meshStrings.size() - 1; i++)
                {
                    dropDownSelection = dropDownSelection + meshStrings[i] + ";";
                }

                dropDownSelection = dropDownSelection + meshStrings[meshStrings.size() - 1];

                GuiLabel((Rectangle){ box.x + 216, box.y + 168, 168, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.meshtitle").c_str());

                if (GuiDropdownBox({box.x + 216, box.y + 192, 168, 24}, dropDownSelection.c_str(), &dropDownSelectedMesh, isMeshDropDownActive)){
                    isMeshDropDownActive = !isMeshDropDownActive;
                }

                GuiLabel((Rectangle){ box.x + 216, box.y + 216, 168, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.angulardampingtitle").c_str());

                if(GuiTextBox((Rectangle){ box.x + 216, box.y + 240, 168, 24 }, textBufferAngularDamping, 64, isBufferAngularDampingEdited)){
                    isBufferAngularDampingEdited = !isBufferAngularDampingEdited;
                }
                
                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_CENTER);

                /*

                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer1InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer2InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer3InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer4InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer5InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer6InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer7InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer8InverseInertiaTensor, 64, isBeingEdited);
                GuiTextBox((Rectangle){ 100, 100, 48, 24 }, textBuffer9InverseInertiaTensor, 64, isBeingEdited);


                
                if(GuiTextBox((Rectangle){ box.x + 10, box.y , 90, 30}), "x"){

                }
                */
               GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

                // Add Button
                if (GuiButton((Rectangle){box.x + box.width - 72, box.y + box.height - 48, 48, 24}, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.addbutton").c_str())) {
                    // Assuming data is valid
                    isValidData = true;

                    // Converting the char arrays to real values.
                    IPhysicsEngine::CharBufferResultStore* xCoordinate = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);
                    IPhysicsEngine::CharBufferResultStore* yCoordinate = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);
                    IPhysicsEngine::CharBufferResultStore* zCoordinate = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);

                    if(!xCoordinate->isValid){
                        isValidData = false;
                    }

                    if(!yCoordinate->isValid){
                        isValidData = false;
                    }

                    if(!zCoordinate->isValid){
                        isValidData = false;
                    }

                    
                    position.x = (xCoordinate->result);
                    position.y = (yCoordinate->result);
                    position.z = (zCoordinate->result);

                    IPhysicsEngine::CharBufferResultStore* xOrientation = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);
                    IPhysicsEngine::CharBufferResultStore* yOrientation = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);
                    IPhysicsEngine::CharBufferResultStore* zOrientation = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);

                    if(!xOrientation->isValid){
                        isValidData = false;
                    }

                    if(!yOrientation->isValid){
                        isValidData = false;
                    }

                    if(!zOrientation->isValid){
                        isValidData = false;
                    }

                    orientation.SetFromEuler(xOrientation->result, yOrientation->result, zOrientation->result);

                    IPhysicsEngine::CharBufferResultStore* massResult = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);

                    if(!massResult->isValid){
                        isValidData = false;
                    }

                    mass = massResult->result;

                    IPhysicsEngine::CharBufferResultStore* linearDampingResult = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);

                    if(!linearDampingResult->isValid){
                        isValidData = false;
                    }

                    linearDamping = linearDampingResult->result;

                    IPhysicsEngine::CharBufferResultStore* angularDampingResult = IPhysicsEngine::CharBufferToReal(textBufferXCoordinate);

                    if(!angularDampingResult->isValid){
                        isValidData = false;
                    }

                    angularDamping = angularDampingResult->result;

                    if(isValidData){
                        // Creating the object
                        IPhysicsEngine::Object* object = new IPhysicsEngine::Object();
                        IPhysicsEngine::RigidBody* rigidbody = object->AddComponent<IPhysicsEngine::RigidBody>();
                        IPhysicsEngine::Geometry* geometry = object->AddComponent<IPhysicsEngine::Geometry>();
                        rigidbody->SetPosition(position);
                        rigidbody->SetOrientation(orientation);
                        rigidbody->SetMass(mass);
                        rigidbody->SetLinearDamping(linearDamping);
                        rigidbody->SetAngularDamping(angularDamping);
                        rigidbody->SetInverseInertiaTensor(inverseInertiaTensor);

                        Mesh cubeMesh = GenMeshCube(1.0f, 1.0f, 1.0f);
                        geometry->SetMesh(&cubeMesh);
                        geometry->SetScale(1.0f);
                        geometry->SetColor(RED);
                        
                        world.AddObject(object);
                        //world.AddForceRegistry(object, gravity);
                    }
                    else{

                    }

                    

                }
            }
            else if (showAddMeshBox){
                Rectangle box = {24, 72, 408, 408};
                showAddMeshBox = !GuiWindowBox(box, IPhysicsEngine::LanguageManager::GetText("addmeshmenu.title").c_str());

                GuiLabel((Rectangle){ box.x + 24, box.y + 216, 168, 24 }, IPhysicsEngine::LanguageManager::GetText("addobjectmenu.colortitle").c_str());

                GuiColorPicker({box.x + 24, box.y + 240, 144, 144}, "Pick a color", &color);







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

