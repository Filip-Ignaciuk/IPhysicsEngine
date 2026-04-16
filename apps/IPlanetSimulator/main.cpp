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
#include "object.hpp"
#include "components/component.hpp"
#include "components/geometry.hpp"
#include "languagemanager.hpp"
#include "meshmanager.hpp"


int main(void)
{
    const Vector3 origin = {0,0,0};
    const int screenWidth = 1272;
    const int screenHeight = 720;

    #ifdef __APPLE__
        SetConfigFlags(FLAG_WINDOW_HIGHDPI);
    #endif
    
    std::string pauseButtonText = "#132#";

    InitWindow(screenWidth, screenHeight, "IPlanetSimulator");
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


    IPhysics::World world;
    std::string fileDir = "resources/en-gb.json";
    IPhysics::LanguageManager::LoadLanguage(fileDir);
    IPhysics::MeshManager::LoadDefaults();


    IPhysics::Object* object1 = new IPhysics::Object();
    IPhysics::Geometry* geometry1 = object1->AddComponent<IPhysics::Geometry>();
    IPhysics::RigidBody* rigidbody1 = object1->AddComponent<IPhysics::RigidBody>();
    IPhysics::Vector3 position1(10.0, 10.0, 10.0);
    IPhysics::real mass1 = 7.5 * pow(10, 12);
    IPhysics::Quaternion quaternion1(0, 0, 0, 1);
    rigidbody1->SetPosition(position1);
    rigidbody1->SetMass(mass1);
    rigidbody1->SetOrientation(quaternion1);
    geometry1->SetMesh(IPhysics::MeshManager::GetMesh("Sphere"));
    geometry1->SetColor(Color(RED));
    geometry1->SetScale(1.0);

    IPhysics::Object* object2 = new IPhysics::Object();
    IPhysics::Geometry* geometry2 = object2->AddComponent<IPhysics::Geometry>();
    IPhysics::RigidBody* rigidbody2 = object2->AddComponent<IPhysics::RigidBody>();
    IPhysics::Vector3 position2(-10.0, -10.0, -10.0);
    IPhysics::real mass2 = 7.5 * pow(10, 12);
    IPhysics::Quaternion quaternion2(0, 0, 0, 1);
    rigidbody2->SetPosition(position2);
    rigidbody2->SetMass(mass2);
    rigidbody2->SetOrientation(quaternion2);
    geometry2->SetMesh(IPhysics::MeshManager::GetMesh("Sphere"));
    geometry2->SetColor(Color(BLUE));
    geometry2->SetScale(1.0);

    IPhysics::RealGravity* realGravity = new IPhysics::RealGravity(6.674 * pow(10, -11));
    realGravity->AddObject(object1);
    realGravity->AddObject(object2);
    world.AddForceRegistry(object1, realGravity);
    world.AddForceRegistry(object2, realGravity);
    world.AddObject(object1);
    world.AddObject(object2);


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

    IPhysics::Vector3 position;
    IPhysics::Quaternion orientation;
    IPhysics::real mass;
    IPhysics::real linearDamping;
    IPhysics::real angularDamping;
    IPhysics::Matrix3 inverseInertiaTensor;
    Mesh mesh;
    Color color;
    IPhysics::real scale;

    // Main loop
    while (!WindowShouldClose())
    {
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
            world.RunPhysics();
        }

        BeginDrawing();
            ClearBackground(CLITERAL(Color){39, 38, 40});
            IPhysics::Vector3 iPosition;
            IPhysics::Quaternion iQuaternion;
            IPhysics::real iMass;
            IPhysics::Vector3 iVelocity;
            BeginMode3D(camera);              
                IPhysics::World::Objects::iterator iterator = world.GetObjects().begin();
                while (iterator != world.GetObjects().end()){
                    IPhysics::Object* object = *iterator;
                    IPhysics::RigidBody* rigidbody = object->GetComponent<IPhysics::RigidBody>();
                    IPhysics::Geometry* geometry = object->GetComponent<IPhysics::Geometry>();
                    iPosition =  rigidbody->GetPosition();
                    iQuaternion =  rigidbody->GetOrientation();
                    iMass = rigidbody->GetMass();
                    iVelocity = rigidbody->GetVelocity();
                    Matrix position = MatrixTranslate(iPosition.x, iPosition.y, iPosition.z);
                    Vector4 quaternion{(float)iQuaternion.i, (float)iQuaternion.j, (float)iQuaternion.k, (float)iQuaternion.r};
                    Matrix rotation = QuaternionToMatrix(quaternion);



                    
                    //Model model = LoadModelFromMesh(*geometry->GetMesh());
                    Mesh* mesh = new Mesh(GenMeshSphere(1.0f, 32, 64));

                    Model model = LoadModelFromMesh(*mesh);
                    model.transform = MatrixMultiply(rotation, position);
                    DrawModel(model, (Vector3){0,0,0}, geometry->GetScale(), geometry->GetColor());

                    ++iterator;
                }
               
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
                showAddObjectBox = !GuiWindowBox(box, IPhysics::LanguageManager::GetText("addobjectmenu.title").c_str());

                GuiSetStyle(LABEL, TEXT_ALIGNMENT_VERTICAL, TEXT_ALIGN_MIDDLE);

                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

                GuiLabel((Rectangle){ box.x + 24, box.y + 24, 96, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.positiontitle").c_str());

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

                GuiLabel((Rectangle){ box.x + 24, box.y + 72, 96, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.orientationtitle").c_str());

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

                GuiLabel((Rectangle){ box.x + 24, box.y + 120, 168, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.masstitle").c_str());


                if(GuiTextBox((Rectangle){ box.x + 24, box.y + 144, 168, 24 }, textBufferMass, 64, isBufferMassEdited)){
                    isBufferMassEdited = !isBufferMassEdited;
                }
                
                GuiLabel((Rectangle){ box.x + 216, box.y + 120, 168, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.lineardampingtitle").c_str());

                if(GuiTextBox((Rectangle){ box.x + 216, box.y + 144, 168, 24 }, textBufferLinearDamping, 64, isBufferLinearDampingEdited)){
                    isBufferLinearDampingEdited = !isBufferLinearDampingEdited;
                }

                GuiLabel((Rectangle){ box.x + 24, box.y + 168, 168, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.angulardampingtitle").c_str());

                if(GuiTextBox((Rectangle){ box.x + 24, box.y + 192, 168, 24 }, textBufferAngularDamping, 64, isBufferAngularDampingEdited)){
                    isBufferAngularDampingEdited = !isBufferAngularDampingEdited;
                }

                std::string dropDownSelection;
                std::vector<std::string> meshStrings = IPhysics::MeshManager::GetMeshStrings();
                for (size_t i = 0; i < meshStrings.size() - 1; i++)
                {
                    dropDownSelection = dropDownSelection + meshStrings[i] + ";";
                }

                dropDownSelection = dropDownSelection + meshStrings[meshStrings.size() - 1];

                GuiLabel((Rectangle){ box.x + 216, box.y + 168, 168, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.meshtitle").c_str());

                if (GuiDropdownBox({box.x + 216, box.y + 192, 168, 24}, dropDownSelection.c_str(), &dropDownSelectedMesh, isMeshDropDownActive)){
                    isMeshDropDownActive = !isMeshDropDownActive;
                }

                GuiLabel((Rectangle){ box.x + 216, box.y + 216, 168, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.angulardampingtitle").c_str());

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
                if (GuiButton((Rectangle){box.x + box.width - 72, box.y + box.height - 48, 48, 24}, IPhysics::LanguageManager::GetText("addobjectmenu.addbutton").c_str())) {
                    // Assuming data is valid
                    isValidData = true;

                    // Converting the char arrays to real values.
                    IPhysics::CharBufferResultStore* xCoordinate = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* yCoordinate = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* zCoordinate = IPhysics::CharBufferToReal(textBufferXCoordinate);

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

                    IPhysics::CharBufferResultStore* xOrientation = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* yOrientation = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* zOrientation = IPhysics::CharBufferToReal(textBufferXCoordinate);

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

                    IPhysics::CharBufferResultStore* massResult = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!massResult->isValid){
                        isValidData = false;
                    }

                    mass = massResult->result;

                    IPhysics::CharBufferResultStore* linearDampingResult = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!linearDampingResult->isValid){
                        isValidData = false;
                    }

                    linearDamping = linearDampingResult->result;

                    IPhysics::CharBufferResultStore* angularDampingResult = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!angularDampingResult->isValid){
                        isValidData = false;
                    }

                    angularDamping = angularDampingResult->result;

                    if(isValidData){
                        // Creating the object
                        IPhysics::Object* object = new IPhysics::Object();
                        IPhysics::RigidBody* rigidbody = object->AddComponent<IPhysics::RigidBody>();
                        IPhysics::Geometry* geometry = object->AddComponent<IPhysics::Geometry>();
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
                    }
                    else{

                    }

                    

                }
            }
            else if (showAddMeshBox){
                Rectangle box = {24, 72, 408, 408};
                showAddMeshBox = !GuiWindowBox(box, IPhysics::LanguageManager::GetText("addmeshmenu.title").c_str());

                GuiLabel((Rectangle){ box.x + 24, box.y + 216, 168, 24 }, IPhysics::LanguageManager::GetText("addobjectmenu.colortitle").c_str());

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

