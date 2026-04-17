#include <iostream>
#include <chrono>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <memory>
#include <unordered_map>
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
#include "components/information.hpp"
#include "languagemanager.hpp"
#include "meshmanager.hpp"
#include "errormanager.hpp"


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

    
    std::unordered_map<IPhysics::Object*, Model*> Map; 

    IPhysics::World world;
    std::string fileDir = "resources/en-gb.json";
    IApp::LanguageManager::LoadLanguage(fileDir);
    IApp::MeshManager::LoadDefaults();

    IPhysics::Matrix3 tensor(
    2.5e-13, 0,       0,
    0,       2.5e-13, 0,
    0,       0,       2.5e-13
    );
    IPhysics::Vector3 velocity(0, 3.2, 0);
    IPhysics::Object* object1 = new IPhysics::Object();
    IPhysics::Geometry* geometry1 = object1->AddComponent<IPhysics::Geometry>();
    IPhysics::RigidBody* rigidbody1 = object1->AddComponent<IPhysics::RigidBody>();
    IPhysics::Information* information1 = object1->AddComponent<IPhysics::Information>();
    IPhysics::Vector3 position1(10.0, 10.0, 10.0);
    IPhysics::real mass1 = 7.5 * pow(10, 12);
    IPhysics::Quaternion quaternion1(0, 0, 0, 1);
    IPhysics::real damping = 1;
    rigidbody1->SetPosition(position1);
    rigidbody1->SetMass(mass1);
    rigidbody1->SetOrientation(quaternion1);
    rigidbody1->SetLinearDamping(damping);
    rigidbody1->SetAngularDamping(damping);
    rigidbody1->SetInverseInertiaTensor(tensor);
    rigidbody1->AddVelocity(velocity);
    geometry1->SetMesh(IApp::MeshManager::GetMesh("Box"));
    geometry1->SetColor(Color(RED));
    geometry1->SetScale(1.0);
    std::string name1 = "One";
    information1->SetName(name1);
    Model model1 = LoadModelFromMesh(*geometry1->GetMesh());
    Map.emplace(object1, &model1);

    IPhysics::Object* object2 = new IPhysics::Object();
    IPhysics::Geometry* geometry2 = object2->AddComponent<IPhysics::Geometry>();
    IPhysics::RigidBody* rigidbody2 = object2->AddComponent<IPhysics::RigidBody>();
    IPhysics::Information* information2 = object2->AddComponent<IPhysics::Information>();
    IPhysics::Vector3 position2(0.0, 0.0, 0.0);
    IPhysics::real mass2 = 7.5 * pow(10, 12);
    IPhysics::Quaternion quaternion2(0, 0, 0, 1);
    rigidbody2->SetPosition(position2);
    rigidbody2->SetMass(mass2);
    rigidbody2->SetOrientation(quaternion2);
    rigidbody2->SetLinearDamping(damping);
    rigidbody2->SetAngularDamping(damping);
    rigidbody2->SetInverseInertiaTensor(tensor);
    //rigidbody2->AddVelocity(velocity);
    geometry2->SetMesh(IApp::MeshManager::GetMesh("Box"));
    geometry2->SetColor(Color(BLUE));
    geometry2->SetScale(1.0);
    std::string name2 = "Two";
    information2->SetName(name1);
    Model model2 = LoadModelFromMesh(*geometry2->GetMesh());
    Map.emplace(object2, &model2);

    IPhysics::RealGravity* realGravity = new IPhysics::RealGravity(6.674 * pow(10, -11));
    realGravity->AddObject(object1);
    realGravity->AddObject(object2);
    world.AddForceRegistry(object1, realGravity);
    world.AddForceRegistry(object2, realGravity);
    world.AddObject(object1);
    world.AddObject(object2);


    // GUI state
    bool showListObjectBox = false;
    bool showAddObjectBox = false;
    bool showAddMeshBox = false;
    bool showPreviewBox = false;
    bool showHelpBox = false;
    bool showSettingsBox = false;

    // List Objects Window State
    static Vector2 scroll = { 0, 0 };
    Rectangle view = { 0 };

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


    static char textBufferXCoordinate[64] = "";
    static char textBufferYCoordinate[64] = "";
    static char textBufferZCoordinate[64] = "";
    static char textBufferXOrientation[64] = "";
    static char textBufferYOrientation[64] = "";
    static char textBufferZOrientation[64] = "";
    static char textBufferMass[64] = "";
    static char textBufferLinearDamping[64] = "";
    static char textBufferAngularDamping[64] = "";

    static char textBuffer1InverseInertiaTensor[64] = "";
    static char textBuffer2InverseInertiaTensor[64] = "";
    static char textBuffer3InverseInertiaTensor[64] = "";
    static char textBuffer4InverseInertiaTensor[64] = "";
    static char textBuffer5InverseInertiaTensor[64] = "";
    static char textBuffer6InverseInertiaTensor[64] = "";
    static char textBuffer7InverseInertiaTensor[64] = "";
    static char textBuffer8InverseInertiaTensor[64] = "";
    static char textBuffer9InverseInertiaTensor[64] = "";

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

    // Information associated with error handling
    std::vector<IApp::Error> errors;

    // Main loop
    while (!WindowShouldClose())
    {
        // Handling Errors
        if(IApp::ErrorManager::IsQueueNotEmpty() && errors.size() != 5){
            errors.emplace_back(IApp::ErrorManager::GetNextError());
        }
        Vector2 startingPosition = { 960, 648 };
        for (auto iterator = errors.begin(); iterator != errors.end();){
            IApp::Error error = *iterator;
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::FatalError){
                break;
            }
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::NormalError){
                DrawRectangle( startingPosition.x, startingPosition.y, 288, 48, RED);
                DrawRectangleLines( startingPosition.x, startingPosition.y, 288, 48, MAROON);

                DrawText(error.GetErrorTitle().c_str(), startingPosition.x, startingPosition.y + 8, 10, WHITE);
                DrawText(error.GetErrorMessage().c_str(), startingPosition.x, startingPosition.y + 24, 10, LIGHTGRAY);
                if(GuiButton({startingPosition.x + 256, startingPosition.y + 8, 24, 24}, "#113#")){
                    iterator = errors.erase(iterator);
                    break;
                }
            }
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::Warning){
                DrawRectangle( startingPosition.x, startingPosition.y, 288, 48, YELLOW);
                DrawRectangleLines( startingPosition.x, startingPosition.y, 288, 48, BROWN);

                DrawText(error.GetErrorTitle().c_str(), startingPosition.x, startingPosition.y + 8, 10, WHITE);
                DrawText(error.GetErrorMessage().c_str(), startingPosition.x, startingPosition.y + 24, 10, LIGHTGRAY);
                if(GuiButton({startingPosition.x + 256, startingPosition.y + 8, 24, 24}, "#113#")){
                    iterator = errors.erase(iterator);
                    break;
                }
                
            }
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::Information){
                DrawRectangle( startingPosition.x, startingPosition.y, 288, 48, LIGHTGRAY);
                DrawRectangleLines( startingPosition.x, startingPosition.y, 288, 48, DARKGRAY);

                DrawText(error.GetErrorTitle().c_str(), startingPosition.x, startingPosition.y + 8, 10, WHITE);
                DrawText(error.GetErrorMessage().c_str(), startingPosition.x, startingPosition.y + 24, 10, LIGHTGRAY);
                if(GuiButton({startingPosition.x + 256, startingPosition.y + 8, 24, 24}, "#113#")){
                    iterator = errors.erase(iterator);
                    break;
                }
                
            }

            ++iterator;
            startingPosition.y -= 72;

        }




        world.StartFrame();

        if (IsKeyPressed(KEY_SPACE)) {
            world.SetPhysicsState(!world.GetPhysicsState());
            if(world.GetPhysicsState()){
                pauseButtonText = "#132#";
            }
            else{
                pauseButtonText = "#131#";
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



                    Model* model = Map[object];
                    model->transform = MatrixMultiply(rotation, position);
                    DrawModel(*model, (Vector3){0,0,0}, geometry->GetScale(), geometry->GetColor());

                    ++iterator;
                }
               
                DrawGrid(100, 1.0f);


            EndMode3D();

            IPhysics::RigidBody* rigidbody = object2->GetComponent<IPhysics::RigidBody>();
            iPosition = rigidbody->GetPosition();
            std::string text = std::to_string(iPosition.x) + " " + std::to_string(iPosition.y) + " " + std::to_string(iPosition.z);
            GuiLabel((Rectangle){ 256, 256, 256, 256 }, text.c_str());
            
            // List Button
            if (GuiButton((Rectangle){ 24, 24, 24, 24 }, "#214#")) {
                showListObjectBox = !showListObjectBox;
            }

            // Add Button
            if (GuiButton((Rectangle){ 72, 24, 24, 24 }, "#80#")) {
                showAddObjectBox = !showAddObjectBox;
            }

            // Add Mesh Button
            if (GuiButton((Rectangle){ 120, 24, 24, 24 }, "#162#")) {
                showAddMeshBox = !showAddMeshBox;
            }

            // Pause Button
            if (GuiButton((Rectangle){ 216, 24, 24, 24 }, pauseButtonText.c_str())){
                if (world.GetPhysicsState()){
                    pauseButtonText = "#131#";
                    world.SetPhysicsState(false);
                }
                else{
                    pauseButtonText = "#132#";
                    world.SetPhysicsState(true);
                }
            }

            // Help Button
            if (GuiButton((Rectangle){ 168, 24, 24, 24 },"#193#")){
                showHelpBox = !showHelpBox;
            }

            // Settings Button
            if (GuiButton((Rectangle){ 1224, 24, 24, 24 },"#142#")){
                showSettingsBox = !showSettingsBox;
            }

            if(showListObjectBox){
                Rectangle box = {24, 72, 408, 408};
                Rectangle scrollPanel = { 24, 96, 408, 408 };
                Rectangle content = { 0, 0, 290, 290 };
                Rectangle view;
                



                showListObjectBox = !GuiWindowBox(box, "List");

                GuiScrollPanel(scrollPanel, NULL, content, &scroll, &view);
                Vector2 initialObjectPanelPosition = {scrollPanel.x, scrollPanel.y};

                for(IPhysics::Object* object : world.GetObjects()){
                    IPhysics::Information* information = object->GetComponent<IPhysics::Information>();
                    Rectangle objectPanel {initialObjectPanelPosition.x, initialObjectPanelPosition.y, 408, 40};
                    Rectangle objectNameRectangle {objectPanel.x + 8, objectPanel.y + 8, 232, 24};
                    Rectangle objectViewButtonRectangle {objectPanel.x + 296, objectPanel.y + 8, 24, 24};
                    Rectangle objectDeleteButtonRectangle {objectPanel.x + 344, objectPanel.y + 8, 24, 24};

                    DrawRectangle(objectPanel.x, objectPanel.y, objectPanel.width, objectPanel.height, LIGHTGRAY);
                    DrawRectangleLines(objectPanel.x, objectPanel.y, objectPanel.width, objectPanel.height, DARKGRAY);
                    GuiLabel(objectNameRectangle, information->GetName().c_str());

                    if(GuiButton(objectViewButtonRectangle, "#42#")){

                    }

                    if(GuiButton(objectDeleteButtonRectangle, "#143#")){
                        world.RemoveObject(object);
                    }
                    initialObjectPanelPosition.y += 40;
                }


                BeginScissorMode(view.x, view.y, view.width, view.height);

                

                EndScissorMode();
            }


            if (showAddObjectBox)
            {
                Rectangle box = {24, 72, 408, 408};
                showAddObjectBox = !GuiWindowBox(box, IApp::LanguageManager::GetText("addobjectmenu.title").c_str());

                GuiSetStyle(LABEL, TEXT_ALIGNMENT_VERTICAL, TEXT_ALIGN_MIDDLE);

                GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

                GuiLabel((Rectangle){ box.x + 24, box.y + 24, 96, 24 }, IApp::LanguageManager::GetText("addobjectmenu.positiontitle").c_str());

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

                GuiLabel((Rectangle){ box.x + 24, box.y + 72, 96, 24 }, IApp::LanguageManager::GetText("addobjectmenu.orientationtitle").c_str());

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

                GuiLabel((Rectangle){ box.x + 24, box.y + 120, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.masstitle").c_str());


                if(GuiTextBox((Rectangle){ box.x + 24, box.y + 144, 168, 24 }, textBufferMass, 64, isBufferMassEdited)){
                    isBufferMassEdited = !isBufferMassEdited;
                }
                
                GuiLabel((Rectangle){ box.x + 216, box.y + 120, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.lineardampingtitle").c_str());

                if(GuiTextBox((Rectangle){ box.x + 216, box.y + 144, 168, 24 }, textBufferLinearDamping, 64, isBufferLinearDampingEdited)){
                    isBufferLinearDampingEdited = !isBufferLinearDampingEdited;
                }

                GuiLabel((Rectangle){ box.x + 24, box.y + 168, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.angulardampingtitle").c_str());

                if(GuiTextBox((Rectangle){ box.x + 24, box.y + 192, 168, 24 }, textBufferAngularDamping, 64, isBufferAngularDampingEdited)){
                    isBufferAngularDampingEdited = !isBufferAngularDampingEdited;
                }

                std::string dropDownSelection;
                std::vector<std::string> meshStrings = IApp::MeshManager::GetMeshStrings();
                for (size_t i = 0; i < meshStrings.size() - 1; i++)
                {
                    dropDownSelection = dropDownSelection + meshStrings[i] + ";";
                }

                dropDownSelection = dropDownSelection + meshStrings[meshStrings.size() - 1];

                GuiLabel((Rectangle){ box.x + 216, box.y + 168, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.meshtitle").c_str());

                if (GuiDropdownBox({box.x + 216, box.y + 192, 168, 24}, dropDownSelection.c_str(), &dropDownSelectedMesh, isMeshDropDownActive)){
                    isMeshDropDownActive = !isMeshDropDownActive;
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
                if (GuiButton((Rectangle){box.x + box.width - 72, box.y + box.height - 48, 48, 24}, IApp::LanguageManager::GetText("addobjectmenu.addbutton").c_str())) {
                    // Assuming data is valid
                    isValidData = true;

                    // Converting the char arrays to real values.
                    IPhysics::CharBufferResultStore* xCoordinate = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* yCoordinate = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* zCoordinate = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!xCoordinate->isValid){
                        isValidData = false;
                        std::string title = "X coordinate invalid";
                        std::string message = "The X coordinate you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    if(!yCoordinate->isValid){
                        isValidData = false;
                        std::string title = "Y coordinate invalid";
                        std::string message = "The Y coordinate you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    if(!zCoordinate->isValid){
                        isValidData = false;
                        std::string title = "Z coordinate invalid";
                        std::string message = "The Z coordinate you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    
                    position.x = (xCoordinate->result);
                    position.y = (yCoordinate->result);
                    position.z = (zCoordinate->result);

                    IPhysics::CharBufferResultStore* xOrientation = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* yOrientation = IPhysics::CharBufferToReal(textBufferXCoordinate);
                    IPhysics::CharBufferResultStore* zOrientation = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!xOrientation->isValid){
                        isValidData = false;
                        std::string title = "X orientation invalid";
                        std::string message = "The X orientation you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    if(!yOrientation->isValid){
                        isValidData = false;
                        std::string title = "Y orientation invalid";
                        std::string message = "The Y orientation you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    if(!zOrientation->isValid){
                        isValidData = false;
                        std::string title = "Z orientation invalid";
                        std::string message = "The Z orientation you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    orientation.SetFromEuler(xOrientation->result, yOrientation->result, zOrientation->result);

                    IPhysics::CharBufferResultStore* massResult = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!massResult->isValid){
                        isValidData = false;
                        std::string title = "Mass invalid";
                        std::string message = "The mass you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    mass = massResult->result;

                    IPhysics::CharBufferResultStore* linearDampingResult = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!linearDampingResult->isValid){
                        isValidData = false;
                        std::string title = "Linear Damping invalid";
                        std::string message = "The linear damping you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    linearDamping = linearDampingResult->result;

                    IPhysics::CharBufferResultStore* angularDampingResult = IPhysics::CharBufferToReal(textBufferXCoordinate);

                    if(!angularDampingResult->isValid){
                        isValidData = false;
                        std::string title = "Angular Damping invalid";
                        std::string message = "The angular damping you have provided is invalid.";
                        IApp::Error error(IApp::ErrorSeverity::NormalError, title, message);
                        IApp::ErrorManager::AddError(error);
                    }

                    angularDamping = angularDampingResult->result;

                    if(isValidData){
                        // Creating the object
                        IPhysics::Object* object = new IPhysics::Object();
                        IPhysics::RigidBody* rigidbody = object->AddComponent<IPhysics::RigidBody>();
                        IPhysics::Geometry* geometry = object->AddComponent<IPhysics::Geometry>();
                        IPhysics::Information* information = object->AddComponent<IPhysics::Information>();
                        rigidbody->SetPosition(position);
                        rigidbody->SetOrientation(orientation);
                        rigidbody->SetMass(mass);
                        rigidbody->SetLinearDamping(linearDamping);
                        rigidbody->SetAngularDamping(angularDamping);
                        IPhysics::Matrix3 tensor(
                            2.5e-13, 0,       0,
                            0,       2.5e-13, 0,
                            0,       0,       2.5e-13
                            );
                        rigidbody->SetInverseInertiaTensor(tensor);

                        geometry->SetMesh(IApp::MeshManager::GetMesh("Box"));
                        geometry->SetScale(1.0f);
                        geometry->SetColor(RED);
                        std::string name = "Name";
                        information->SetName(name);
                        
                        world.AddObject(object);
                    }
                    else{

                    }

                    

                }
            }
            else if (showAddMeshBox){
                Rectangle box = {24, 72, 408, 408};
                showAddMeshBox = !GuiWindowBox(box, IApp::LanguageManager::GetText("addmeshmenu.title").c_str());

                GuiLabel((Rectangle){ box.x + 24, box.y + 216, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.colortitle").c_str());

                GuiColorPicker({box.x + 24, box.y + 240, 144, 144}, "Pick a color", &color);







            }

            if(showPreviewBox){

            }

            if(showHelpBox){
                DrawRectangle( 936, 600, 312, 96, GRAY);
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

