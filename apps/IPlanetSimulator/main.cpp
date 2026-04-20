#include <cmath>
#include <cstddef>
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

#ifdef __EMSCRIPTEN__
    #include <emscripten.h>
#endif

#include <raylib.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <filesystem>
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

#include <iostream>

// Const information
const static Vector3 origin = {0,0,0};
const static int screenWidth = 1272;
const static int screenHeight = 720;
const static int frameRate = 60;
const static IPhysics::real timeStep = (IPhysics::real)1 / (IPhysics::real)frameRate;

// Colours
const static Color darkBackgroundColour = CLITERAL(Color){39, 38, 40};
const static Color lightBackgroundColour = WHITE;
const static Color darkRectangleLinesColour = BLACK;
const static Color lightRectangleLinesColour = WHITE;


static Color backgroundColour = darkBackgroundColour;
static Color rectangleLinesColour = darkRectangleLinesColour;

static bool cameraState = true;
static Camera3D camera = { 0 };
static std::vector<IApp::Error> errors;
static IPhysics::World world;
static std::unordered_map<IPhysics::Object*, Model*> Map; 
const static IPhysics::Matrix3 standardTensor(2.5e-13, 0, 0, 0, 2.5e-13, 0, 0, 0, 2.5e-13);
static bool lightModeEnabled = false;
static bool forceIndicatorEnabled = false;
static bool accelerationIndicatorEnabled = false;
static bool velocityIndicatorEnabled = false;
// GUI state
const static Rectangle standardLeftBox = {24, 72, 408, 408};
const static Rectangle standardLeftViewbox = {24, 504, 408, 192};
const static Rectangle standardRightBox = {840, 72, 408, 408};

enum class LeftHandSideGuiState{
    None,
    ListObjectBox,
    AddObjectBox,
    HelpBox
};

enum class RightHandSideGuiState{
    None,
    SettingsBox
};

LeftHandSideGuiState leftHandGuiState = LeftHandSideGuiState::None;
RightHandSideGuiState rightHandGuiState = RightHandSideGuiState::None;

// List Objects Window State
static Vector2 scroll = { 0, 0 };
static Rectangle view = { 0 };

// Add Object Window State
static bool isValidData = false;

static bool isBufferXCoordinateEdited = false;
static bool isBufferYCoordinateEdited = false;
static bool isBufferZCoordinateEdited = false;

static bool isBufferXOrientationEdited = false;
static bool isBufferYOrientationEdited = false;
static bool isBufferZOrientationEdited = false;

static bool isBufferMassEdited = false;
static bool isBufferNameEdited = false;

static bool isBufferLinearDampingEdited = false;
static bool isBufferAngularDampingEdited = false;

static bool isMeshDropDownActive = false;
static bool isColourDropDownActive = false;
static bool isLanguageDropDownActive = false;

static bool isBuffer1InverseInertiaTensorEdited = false;
static bool isBuffer2InverseInertiaTensorEdited = false;
static bool isBuffer3InverseInertiaTensorEdited = false;
static bool isBuffer4InverseInertiaTensorEdited = false;
static bool isBuffer5InverseInertiaTensorEdited = false;
static bool isBuffer6InverseInertiaTensorEdited = false;
static bool isBuffer7InverseInertiaTensorEdited = false;
static bool isBuffer8InverseInertiaTensorEdited = false;
static bool isBuffer9InverseInertiaTensorEdited = false;

static bool wantsStandardInverseInertiaValue = false;

static char textBufferXCoordinate[64] = "";
static char textBufferYCoordinate[64] = "";
static char textBufferZCoordinate[64] = "";
static char textBufferXOrientation[64] = "";
static char textBufferYOrientation[64] = "";
static char textBufferZOrientation[64] = "";
static char textBufferMass[64] = "";
static char textBufferName[64] = "";
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

static int dropDownSelectedMesh = 0;
static int dropDownSelectedColour = 0;
static int dropDownSelectedLanguage = 0;

static std::string meshDropDownSelection;
static std::vector<std::string> meshStrings;

static std::string colourDropDownSelection;
static std::vector<std::string> colourStrings;

static std::string languageDropDownSelection;
static std::vector<std::string> languageStrings;

static IPhysics::RealGravity* realGravity = new IPhysics::RealGravity(6.674 * pow(10, -11));
static std::string pauseButtonText = "#132#";


// Information associated with add object window
static IPhysics::Object* addObject = nullptr;;
static IPhysics::Object* listObject = nullptr;;

static Camera3D addViewCamera = {0};
static Camera3D listViewCamera = {0};

static void UpdateColourScheme(){
    if(lightModeEnabled){
        backgroundColour = lightBackgroundColour;
    }
    else{
        backgroundColour = darkBackgroundColour;
    }
}

static void UpdateAddObject(){
    IPhysics::Geometry* geometry = addObject->GetComponent<IPhysics::Geometry>();
    geometry->SetMesh(IApp::MeshManager::GetMesh(meshStrings[dropDownSelectedMesh]));
    geometry->SetScale(1.0f);
    geometry->SetColor(IApp::MeshManager::GetColor(colourStrings[dropDownSelectedColour]));
}

static inline void ResetAddObject(){
    addObject = new IPhysics::Object();
    addObject->AddComponent<IPhysics::RigidBody>();
    addObject->AddComponent<IPhysics::Information>();
    addObject->AddComponent<IPhysics::Geometry>();
    // So far we only need to set the default mesh and color.
    UpdateAddObject();
}


static inline void ResetListObject(){
    if(listObject != nullptr){
        // Just delete the reference to the object not the object itself.
        listObject = nullptr;
    }
}

static inline void InitialiseGUI(){
    // Determine if we need highdpi for mac.
    #if defined(__APPLE__) && !defined(__EMSCRIPTEN__)       
    SetConfigFlags(FLAG_WINDOW_HIGHDPI);
    #endif
    InitWindow(screenWidth, screenHeight, "IPlanetSimulator");
    GuiEnable();
    GuiLoadStyleDefault();
    SetTargetFPS(frameRate);
    IApp::MeshManager::LoadDefaults();
    meshStrings = IApp::MeshManager::GetMeshStrings();
    colourStrings = IApp::MeshManager::GetColourStrings();
    languageStrings = IApp::LanguageManager::GetLanguageStrings();

    camera.position = (Vector3){ 30.0f, 30.0f, 30.0f };
    camera.target = origin;
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    addViewCamera.position = (Vector3){10.0f, 10.0f, 0.0f};
    addViewCamera.target = origin;
    addViewCamera.up = (Vector3){0.0f, 1.0f, 0.0f};
    addViewCamera.fovy = 45.0f;
    addViewCamera.projection = CAMERA_PERSPECTIVE;

    listViewCamera.up = (Vector3){0.0f, 1.0f, 0.0f};
    listViewCamera.fovy = 45.0f;
    listViewCamera.projection = CAMERA_PERSPECTIVE;

    for (size_t i = 0; i < meshStrings.size() - 1; i++)
    {
        meshDropDownSelection = meshDropDownSelection + meshStrings[i] + ";";
    }
    meshDropDownSelection = meshDropDownSelection + meshStrings[meshStrings.size() - 1];

    
    for (size_t i = 0; i < colourStrings.size() - 1; i++)
    {
        colourDropDownSelection = colourDropDownSelection + colourStrings[i] + ";";
    }
    colourDropDownSelection = colourDropDownSelection + colourStrings[colourStrings.size() - 1];

    for (size_t i = 0; i < languageStrings.size() - 1; i++)
    {
        languageDropDownSelection = languageDropDownSelection + languageStrings[i] + ";";
        if(languageStrings[i] == "English (UK)"){
            dropDownSelectedLanguage = i;
        }
    }
    languageDropDownSelection = languageDropDownSelection + languageStrings[languageStrings.size() - 1];
}

static inline void AddObjectMenu(){
    if(GuiWindowBox(standardLeftBox, IApp::LanguageManager::GetText("addobjectmenu.title").c_str())){
        leftHandGuiState = LeftHandSideGuiState::None;
    }
    else{
        leftHandGuiState = LeftHandSideGuiState::AddObjectBox;
    }

    GuiSetStyle(LABEL, TEXT_ALIGNMENT_VERTICAL, TEXT_ALIGN_MIDDLE);

    GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

    GuiLabel((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 24, 96, 24 }, IApp::LanguageManager::GetText("addobjectmenu.position.title").c_str());

    GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_CENTER);


    GuiLabel((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 48, 24, 24 }, "X");
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 48, standardLeftBox.y + 48, 96, 24 }, textBufferXCoordinate, 64, isBufferXCoordinateEdited)){
        isBufferXCoordinateEdited = !isBufferXCoordinateEdited;
    }
    
    GuiLabel((Rectangle){ standardLeftBox.x + 144, standardLeftBox.y + 48, 24, 24 }, "Y");
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 168, standardLeftBox.y + 48, 96, 24 }, textBufferYCoordinate, 64, isBufferYCoordinateEdited)){
        isBufferYCoordinateEdited = !isBufferYCoordinateEdited;
    }

    GuiLabel((Rectangle){ standardLeftBox.x + 264, standardLeftBox.y + 48, 24, 24 }, "Z");
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 288, standardLeftBox.y + 48, 96, 24 }, textBufferZCoordinate, 64, isBufferZCoordinateEdited)){
        isBufferZCoordinateEdited = !isBufferZCoordinateEdited;
    }
    
    GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

    GuiLabel((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 72, 96, 24 }, IApp::LanguageManager::GetText("addobjectmenu.orientation.title").c_str());

    GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_CENTER);

    GuiLabel((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 96, 24, 24 }, "X");
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 48, standardLeftBox.y + 96, 96, 24 }, textBufferXOrientation, 64, isBufferXOrientationEdited)){
        isBufferXOrientationEdited = !isBufferXOrientationEdited;
    }
                    
    GuiLabel((Rectangle){ standardLeftBox.x + 144, standardLeftBox.y + 96, 24, 24 }, "Y");
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 168, standardLeftBox.y + 96, 96, 24 }, textBufferYOrientation, 64, isBufferYOrientationEdited)){
        isBufferYOrientationEdited = !isBufferYOrientationEdited;
    }

    GuiLabel((Rectangle){ standardLeftBox.x + 264, standardLeftBox.y + 96, 24, 24 }, "Z");
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 288, standardLeftBox.y + 96, 96, 24 }, textBufferZOrientation, 64, isBufferZOrientationEdited)){
        isBufferZOrientationEdited = !isBufferZOrientationEdited;
    }

    GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

    GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
    GuiLabel((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 120, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.name.title").c_str());
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 144, 168, 24 }, textBufferName, 64, isBufferNameEdited)){
        isBufferNameEdited = !isBufferNameEdited;
    }

    GuiLabel((Rectangle){ standardLeftBox.x + 216, standardLeftBox.y + 120, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.mass.title").c_str());
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 216, standardLeftBox.y + 144, 168, 24 }, textBufferMass, 64, isBufferMassEdited)){
        isBufferMassEdited = !isBufferMassEdited;
    }

    GuiLabel((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 168, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.lineardamping.title").c_str());
    GuiLabel((Rectangle){ standardLeftBox.x + 216, standardLeftBox.y + 168, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.angulardamping.title").c_str());

    if(GuiTextBox((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 192, 168, 24 }, textBufferLinearDamping, 64, isBufferLinearDampingEdited)){
        isBufferLinearDampingEdited = !isBufferLinearDampingEdited;
    }
    if(GuiTextBox((Rectangle){ standardLeftBox.x + 216, standardLeftBox.y + 192, 168, 24 }, textBufferAngularDamping, 64, isBufferAngularDampingEdited)){
        isBufferAngularDampingEdited = !isBufferAngularDampingEdited;
    }

    

    GuiLabel((Rectangle){ standardLeftBox.x + 216, standardLeftBox.y + 312, 148, 24 }, IApp::LanguageManager::GetText("addobjectmenu.inverseinertiatensor.title").c_str());
    GuiCheckBox((Rectangle){ standardLeftBox.x + 364, standardLeftBox.y + 316, 16, 16 }, "", &wantsStandardInverseInertiaValue);

    GuiLabel((Rectangle){ standardLeftBox.x + 24, standardLeftBox.y + 216, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.inverseinertiatensor.title").c_str());
    if(wantsStandardInverseInertiaValue){
        GuiSetState(STATE_DISABLED);
    }

    GuiTextBox((Rectangle){ standardLeftBox.x + 24,  standardLeftBox.y + 240, 24, 24 }, textBuffer1InverseInertiaTensor, 64, isBuffer1InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 72,  standardLeftBox.y + 240, 24, 24 }, textBuffer2InverseInertiaTensor, 64, isBuffer2InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 120, standardLeftBox.y + 240, 24, 24 }, textBuffer3InverseInertiaTensor, 64, isBuffer3InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 24,  standardLeftBox.y + 288, 24, 24 }, textBuffer4InverseInertiaTensor, 64, isBuffer4InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 72,  standardLeftBox.y + 288, 24, 24 }, textBuffer5InverseInertiaTensor, 64, isBuffer5InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 120, standardLeftBox.y + 288, 24, 24 }, textBuffer6InverseInertiaTensor, 64, isBuffer6InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 24,  standardLeftBox.y + 336, 24, 24 }, textBuffer7InverseInertiaTensor, 64, isBuffer7InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 72,  standardLeftBox.y + 336, 24, 24 }, textBuffer8InverseInertiaTensor, 64, isBuffer8InverseInertiaTensorEdited);
    GuiTextBox((Rectangle){ standardLeftBox.x + 120, standardLeftBox.y + 336, 24, 24 }, textBuffer9InverseInertiaTensor, 64, isBuffer9InverseInertiaTensorEdited);

    if(wantsStandardInverseInertiaValue){
        GuiSetState(STATE_NORMAL);
    }

    GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);

    // Add Button
    if (GuiButton((Rectangle){standardLeftBox.x + 300, standardLeftBox.y + 360, 84, 24}, IApp::LanguageManager::GetText("addobjectmenu.addbutton.title").c_str()) && !isColourDropDownActive && !isMeshDropDownActive) {
        // Assuming data is valid
        isValidData = true;

        // Converting the char arrays to real values.
        IPhysics::CharBufferResultStore* xCoordinate = IPhysics::CharBufferToReal(textBufferXCoordinate);
        IPhysics::CharBufferResultStore* yCoordinate = IPhysics::CharBufferToReal(textBufferYCoordinate);
        IPhysics::CharBufferResultStore* zCoordinate = IPhysics::CharBufferToReal(textBufferZCoordinate);

        if(!xCoordinate->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.xcoordinate.title").c_str(), 
                IApp::LanguageManager::GetText("addobjectmenu.error.xcoordinate.message").c_str(), 
                IApp::ErrorSeverity::NormalError
            );
        }

        if(!yCoordinate->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.ycoordinate.title").c_str(), 
                IApp::LanguageManager::GetText("addobjectmenu.error.ycoordinate.message").c_str(), 
                IApp::ErrorSeverity::NormalError
            );
        }

        if(!zCoordinate->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.zcoordinate.title").c_str(), 
                IApp::LanguageManager::GetText("addobjectmenu.error.zcoordinate.message").c_str(), 
                IApp::ErrorSeverity::NormalError
            );
        }

        IPhysics::CharBufferResultStore* xOrientation = IPhysics::CharBufferToReal(textBufferXOrientation);
        IPhysics::CharBufferResultStore* yOrientation = IPhysics::CharBufferToReal(textBufferYOrientation);
        IPhysics::CharBufferResultStore* zOrientation = IPhysics::CharBufferToReal(textBufferZOrientation);

        if(!xOrientation->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.xorientation.title").c_str(),
                IApp::LanguageManager::GetText("addobjectmenu.error.xorientation.message").c_str(),
                IApp::ErrorSeverity::NormalError
            );
        }   

        if(!yOrientation->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.yorientation.title").c_str(),
                IApp::LanguageManager::GetText("addobjectmenu.error.yorientation.message").c_str(),
                IApp::ErrorSeverity::NormalError
            );
        }

        if(!zOrientation->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.zorientation.title").c_str(),
                IApp::LanguageManager::GetText("addobjectmenu.error.zorientation.message").c_str(),
                IApp::ErrorSeverity::NormalError
            );
        }

        std::string name(textBufferName);
        if(name.empty()){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.name.title").c_str(),
                IApp::LanguageManager::GetText("addobjectmenu.error.name.message").c_str(),
                IApp::ErrorSeverity::NormalError
            );
        }
        else{
            for(IPhysics::Object* object : world.GetObjects()){
                IPhysics::Information* information = object->GetComponent<IPhysics::Information>();
                if(information->GetName() == name){
                    isValidData = false;
                    IApp::ErrorManager::AddError(
                        IApp::LanguageManager::GetText("addobjectmenu.error.name.exists.title").c_str(),
                        IApp::LanguageManager::GetText("addobjectmenu.error.name.exists.message").c_str(),
                        IApp::ErrorSeverity::NormalError
                    );
                }
            }
        }

        IPhysics::CharBufferResultStore* massResult = IPhysics::CharBufferToReal(textBufferMass);
        if(!massResult->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.mass.title").c_str(),
                IApp::LanguageManager::GetText("addobjectmenu.error.mass.message").c_str(),
                IApp::ErrorSeverity::NormalError
            );
        }

        IPhysics::CharBufferResultStore* linearDampingResult = IPhysics::CharBufferToReal(textBufferLinearDamping);
        if(!linearDampingResult->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.lineardamping.title").c_str(),
                IApp::LanguageManager::GetText("addobjectmenu.error.lineardamping.message").c_str(),
                IApp::ErrorSeverity::NormalError
            );
        }

        IPhysics::CharBufferResultStore* angularDampingResult = IPhysics::CharBufferToReal(textBufferAngularDamping);
        if(!angularDampingResult->isValid){
            isValidData = false;
            IApp::ErrorManager::AddError(
                IApp::LanguageManager::GetText("addobjectmenu.error.angulardamping.title").c_str(),
                IApp::LanguageManager::GetText("addobjectmenu.error.angulardamping.message").c_str(),
                IApp::ErrorSeverity::NormalError
            );
        }

        IPhysics::CharBufferResultStore* inverseInertiaResult1 = IPhysics::CharBufferToReal(textBuffer1InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult2 = IPhysics::CharBufferToReal(textBuffer2InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult3 = IPhysics::CharBufferToReal(textBuffer3InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult4 = IPhysics::CharBufferToReal(textBuffer4InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult5 = IPhysics::CharBufferToReal(textBuffer5InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult6 = IPhysics::CharBufferToReal(textBuffer6InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult7 = IPhysics::CharBufferToReal(textBuffer7InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult8 = IPhysics::CharBufferToReal(textBuffer8InverseInertiaTensor);
        IPhysics::CharBufferResultStore* inverseInertiaResult9 = IPhysics::CharBufferToReal(textBuffer9InverseInertiaTensor);

        if(!wantsStandardInverseInertiaValue){
            if(!inverseInertiaResult1->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.topleft.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.topleft.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult2->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.topmiddle.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.topmiddle.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult3->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.topright.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.topright.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult4->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.centreleft.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.centreleft.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult5->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.centremiddle.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.centremiddle.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult6->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.centreright.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.centreright.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult7->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.bottomleft.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.bottomleft.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult8->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.bottommiddle.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.bottommiddle.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
            if(!inverseInertiaResult9->isValid){
                isValidData = false;
                IApp::ErrorManager::AddError(
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.bottomright.title").c_str(),
                    IApp::LanguageManager::GetText("addobjectmenu.error.tensor.bottomright.message").c_str(),
                    IApp::ErrorSeverity::NormalError
                );
            }
        }
            
        

        if(isValidData){
            // Creating the object
            IPhysics::Vector3 position(xCoordinate->result, yCoordinate->result, zCoordinate->result);
            IPhysics::Quaternion orientation;
            orientation.SetFromEuler(xOrientation->result, yOrientation->result, zOrientation->result);
            IPhysics::Matrix3 inverseInertiaTensor;

            if(wantsStandardInverseInertiaValue){
                inverseInertiaTensor = standardTensor;
                std::cout << "Using Standard" << std::endl;
            }
            else{
                inverseInertiaTensor = IPhysics::Matrix3(
                    inverseInertiaResult1->result, inverseInertiaResult2->result, inverseInertiaResult3->result,
                    inverseInertiaResult4->result, inverseInertiaResult5->result, inverseInertiaResult6->result,
                    inverseInertiaResult7->result, inverseInertiaResult8->result, inverseInertiaResult9->result
                );
            }

            IPhysics::RigidBody* rigidbody = addObject->GetComponent<IPhysics::RigidBody>();
            IPhysics::Geometry* geometry = addObject->GetComponent<IPhysics::Geometry>();
            IPhysics::Information* information = addObject->GetComponent<IPhysics::Information>();
            rigidbody->SetPosition(position);
            rigidbody->SetOrientation(orientation);
            rigidbody->SetMass(massResult->result);
            rigidbody->SetLinearDamping(linearDampingResult->result);
            rigidbody->SetAngularDamping(angularDampingResult->result);
            rigidbody->SetInverseInertiaTensor(inverseInertiaTensor);

            geometry->SetMesh(IApp::MeshManager::GetMesh(meshStrings[dropDownSelectedMesh]));
            geometry->SetScale(1.0f);
            geometry->SetColor(IApp::MeshManager::GetColor(colourStrings[dropDownSelectedColour]));
            information->SetName((name));
            
            Model* model = new Model(LoadModelFromMesh(*geometry->GetMesh()));
            Map.emplace(addObject, model);

            realGravity->AddObject(addObject);
            world.AddForceRegistry(addObject, realGravity);
            world.AddObject(addObject);

            // Reset everything.
            dropDownSelectedMesh = 0;
            dropDownSelectedColour = 0;

            ResetAddObject();
        }

    }

    GuiLabel((Rectangle){ standardLeftBox.x + 216, standardLeftBox.y + 264, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.color.title").c_str());
    // We need to check for if the other drop down is active as it can click on both with once click.
    if(GuiDropdownBox({standardLeftBox.x + 216, standardLeftBox.y + 288, 168, 24}, colourDropDownSelection.c_str(), &dropDownSelectedColour, isColourDropDownActive) && !isMeshDropDownActive){
        isColourDropDownActive = !isColourDropDownActive;
        UpdateAddObject();
    }

    GuiLabel((Rectangle){ standardLeftBox.x + 216, standardLeftBox.y + 216, 168, 24 }, IApp::LanguageManager::GetText("addobjectmenu.mesh.title").c_str());
    if(GuiDropdownBox({standardLeftBox.x + 216, standardLeftBox.y + 240, 168, 24}, meshDropDownSelection.c_str(), &dropDownSelectedMesh, isMeshDropDownActive)){
        isMeshDropDownActive = !isMeshDropDownActive;
        UpdateAddObject();
    }
    
}

static inline void ShowListMenu(){
    Rectangle scrollPanel = { 24, 96, 408, 384 };
    Rectangle content = { 0, 0, 408, (float)(40 * world.GetObjects().size()) };
    Rectangle view;

    if(GuiWindowBox(standardLeftBox, IApp::LanguageManager::GetText("showlistmenu.title").c_str())){
        leftHandGuiState = LeftHandSideGuiState::None;
    }
    else{
        leftHandGuiState = LeftHandSideGuiState::ListObjectBox;
    }

    GuiScrollPanel(scrollPanel, NULL, content, &scroll, &view);

    BeginScissorMode(view.x, view.y, view.width, view.height);
    Vector2 initialObjectPanelPosition = {scrollPanel.x, scrollPanel.y + scroll.y};

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
            ResetListObject();
            listObject = object;
        }

        if(GuiButton(objectDeleteButtonRectangle, "#143#")){
            world.RemoveObject(object);
            world.RemoveForceRegistry(object);
            realGravity->RemoveObject(object);
        }
        initialObjectPanelPosition.y += 40;

    }
    EndScissorMode();
}

static inline void ShowHelpMenu(){
    if(GuiWindowBox(standardLeftBox, IApp::LanguageManager::GetText("showhelpmenu.title").c_str())){
        leftHandGuiState = LeftHandSideGuiState::None;
    }
    else{
        leftHandGuiState = LeftHandSideGuiState::HelpBox;
    }
    DrawText(IApp::LanguageManager::GetText("showhelpmenu.tip.camera.toggle").c_str(), standardLeftBox.x + 8, standardLeftBox.y + 32, 10, BLACK);
    DrawText(IApp::LanguageManager::GetText("showhelpmenu.tip.camera.move").c_str(), standardLeftBox.x + 8, standardLeftBox.y + 52, 10, BLACK);
    DrawText(IApp::LanguageManager::GetText("showhelpmenu.tip.physics.toggle").c_str(), standardLeftBox.x + 8, standardLeftBox.y + 72, 10, BLACK);
    DrawText(IApp::LanguageManager::GetText("showhelpmenu.tip.addobject").c_str(), standardLeftBox.x + 8, standardLeftBox.y + 92, 10, BLACK);
    DrawText(IApp::LanguageManager::GetText("showhelpmenu.tip.removeobject").c_str(), standardLeftBox.x + 8, standardLeftBox.y + 112, 10, BLACK);
    DrawText(IApp::LanguageManager::GetText("showhelpmenu.tip.mass.suggestion").c_str(), standardLeftBox.x + 8, standardLeftBox.y + 132, 10, BLACK);
    DrawText(IApp::LanguageManager::GetText("showhelpmenu.tip.mass.value").c_str(), standardLeftBox.x + 8, standardLeftBox.y + 152, 10, BLACK);
}

static inline void ShowSettingsMenu(){
    if(GuiWindowBox(standardRightBox, IApp::LanguageManager::GetText("showsettingsmenu.title").c_str())){
        rightHandGuiState = RightHandSideGuiState::None;
    }
    else{
        rightHandGuiState = RightHandSideGuiState::SettingsBox;
    }

    

    GuiLabel((Rectangle){ standardRightBox.x + 24, standardRightBox.y + 72, 336, 24 }, IApp::LanguageManager::GetText("showsettingsmenu.lightmode.title").c_str());
    if(GuiCheckBox((Rectangle){ standardRightBox.x + 364, standardRightBox.y + 76, 16, 16 }, "", &lightModeEnabled)){
        UpdateColourScheme();
    }
    
    GuiLabel((Rectangle){ standardRightBox.x + 24, standardRightBox.y + 96, 336, 24 }, IApp::LanguageManager::GetText("showsettingsmenu.accelerationindicator.title").c_str());
    GuiCheckBox((Rectangle){ standardRightBox.x + 364, standardRightBox.y + 100, 16, 16 }, "", &accelerationIndicatorEnabled);

    GuiLabel((Rectangle){ standardRightBox.x + 24, standardRightBox.y + 120, 336, 24 }, IApp::LanguageManager::GetText("showsettingsmenu.velocityindicator.title").c_str());
    GuiCheckBox((Rectangle){ standardRightBox.x + 364, standardRightBox.y + 124, 16, 16 }, "", &velocityIndicatorEnabled);
    
    
    GuiLabel((Rectangle){ standardRightBox.x + 24, standardRightBox.y + 24, 168, 24 }, IApp::LanguageManager::GetText("showsettingsmenu.language.title").c_str());
    if(GuiDropdownBox({standardRightBox.x + 24, standardRightBox.y + 48, 168, 24}, languageDropDownSelection.c_str(), &dropDownSelectedLanguage, isLanguageDropDownActive)){
        isLanguageDropDownActive = !isLanguageDropDownActive;
        IApp::LanguageManager::LoadLanguage(languageStrings[dropDownSelectedLanguage]);
    }
}

static void ShowListView(){
    static RenderTexture2D target = LoadRenderTexture(standardLeftViewbox.width, standardLeftViewbox.height);
    DrawRectangle(standardLeftViewbox.x, standardLeftViewbox.y, standardLeftViewbox.width, standardLeftViewbox.height, Color(WHITE));

    if(listObject != nullptr){
        listViewCamera.position = (Vector3){10.0f, 10.0f, 0.0f};
        IPhysics::Vector3 cameraPositionTarget = listObject->GetComponent<IPhysics::RigidBody>()->GetPosition();
        Vector3 objectPosition = {(float)cameraPositionTarget.x, (float)cameraPositionTarget.y, (float)cameraPositionTarget.z};
        listViewCamera.target = objectPosition;
        listViewCamera.position = {objectPosition.x + 10, objectPosition.y + 10, objectPosition.z + 10};

        BeginTextureMode(target);
            ClearBackground(backgroundColour);
            BeginMode3D(listViewCamera);
                DrawGrid(100, 1.0f);
                IPhysics::World::Objects::iterator iterator = world.GetObjects().begin();
                while (iterator != world.GetObjects().end()){
                    IPhysics::Object* object = *iterator;
                    IPhysics::RigidBody* rigidbody = object->GetComponent<IPhysics::RigidBody>();
                    IPhysics::Geometry* geometry = object->GetComponent<IPhysics::Geometry>();
                    IPhysics::Information* information = object->GetComponent<IPhysics::Information>();
                    IPhysics::Vector3 position =  rigidbody->GetPosition();
                    IPhysics::Quaternion quaternion =  rigidbody->GetOrientation();
                    Matrix positionMatrix = MatrixTranslate(position.x, position.y, position.z);
                    Vector4 rayLibQuaternion{(float)quaternion.i, (float)quaternion.j, (float)quaternion.k, (float)quaternion.r};
                    Matrix rotation = QuaternionToMatrix(rayLibQuaternion);

                    Model* model = Map[object];
                    model->transform = MatrixMultiply(rotation, positionMatrix);
                    DrawModel(*model, (Vector3){0,0,0}, geometry->GetScale(), geometry->GetColor());

                    Vector3 rayPosition{(float)position.x, (float)position.y, (float)position.z};

                    if(accelerationIndicatorEnabled){
                        IPhysics::Vector3 acceleration = rigidbody->GetAcceleration();
                        Vector3 rayAcceleration = {(float)acceleration.x, (float)acceleration.y, (float)acceleration.z};
                        Vector3 finalRayPosition = rayPosition + rayAcceleration;

                        DrawLine3D(rayPosition, finalRayPosition, BLUE);
                    }
                    
                    if(velocityIndicatorEnabled){
                        IPhysics::Vector3 velocity = rigidbody->GetVelocity();
                        Vector3 rayVelocity = {(float)velocity.x, (float)velocity.y, (float)velocity.z};
                        Vector3 finalRayPosition = rayPosition + rayVelocity;

                        DrawLine3D(rayPosition, finalRayPosition, YELLOW);
                    }
                    ++iterator;
                }
            EndMode3D();
        EndTextureMode();

        DrawTextureRec(
            target.texture,
            {0, 0, (float)target.texture.width, -(float)target.texture.height},
            {standardLeftViewbox.x, standardLeftViewbox.y},
            WHITE
        );
    }
    else{
        // To achieve centred text.
        int textWidth = MeasureText(IApp::LanguageManager::GetText("showlistview.message").c_str(), 10);
        int textHeight = 15;

        float x = standardLeftViewbox.x + (standardLeftViewbox.width  - textWidth)  / 2.0f;
        float y = standardLeftViewbox.y + (standardLeftViewbox.height - textHeight) / 2.0f;

        DrawText(IApp::LanguageManager::GetText("showlistview.message").c_str(), (int)x, (int)y, 10, DARKGRAY);
    }
    DrawRectangleLines(standardLeftViewbox.x, standardLeftViewbox.y, standardLeftViewbox.width, standardLeftViewbox.height, darkRectangleLinesColour);
}

static void ShowAddView(){
    static RenderTexture2D target = LoadRenderTexture(standardLeftViewbox.width, standardLeftViewbox.height);
    static float angle = 0;

    DrawRectangle(standardLeftViewbox.x, standardLeftViewbox.y, standardLeftViewbox.width, standardLeftViewbox.height, Color(WHITE));


    addViewCamera.position = (Vector3){cosf(angle * PI / 180) * 2.0f,  2.0f, sinf(angle * PI / 180) * 2.0f};
    angle = angle + 0.75;

    // Reset angle to prevent integer overflow
    if(angle == 360){
        angle = 0;
    }

    BeginTextureMode(target);
        ClearBackground(backgroundColour);
        BeginMode3D(addViewCamera);
            DrawGrid(10, 1.0f);
            IPhysics::Geometry* geometry = addObject->GetComponent<IPhysics::Geometry>();
            Model* model = IApp::MeshManager::GetModel(meshStrings[dropDownSelectedMesh]);
            DrawModel(*model, (Vector3){0,0,0}, geometry->GetScale(), geometry->GetColor());
        EndMode3D();
    EndTextureMode();

    DrawTextureRec(
        target.texture,
        {0, 0, (float)target.texture.width, -(float)target.texture.height},
        {standardLeftViewbox.x, standardLeftViewbox.y},
        WHITE
    );

    DrawRectangleLines(standardLeftViewbox.x, standardLeftViewbox.y, standardLeftViewbox.width, standardLeftViewbox.height, darkRectangleLinesColour);

}

static void Update(){
    // Handling Errors
    if(IApp::ErrorManager::IsQueueNotEmpty() && errors.size() != 5){
        errors.emplace_back(IApp::ErrorManager::GetNextError());
    }

    world.StartFrame();

    if (IsKeyPressed('P')) {
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
        world.RunPhysics(timeStep);
    }


    BeginDrawing();
        ClearBackground(backgroundColour);
        BeginMode3D(camera);         
            DrawGrid(100, 1.0f);
    
            IPhysics::World::Objects::iterator iterator = world.GetObjects().begin();
            while (iterator != world.GetObjects().end()){
                IPhysics::Object* object = *iterator;
                IPhysics::RigidBody* rigidbody = object->GetComponent<IPhysics::RigidBody>();
                IPhysics::Geometry* geometry = object->GetComponent<IPhysics::Geometry>();
                IPhysics::Information* information = object->GetComponent<IPhysics::Information>();
                IPhysics::Vector3 position =  rigidbody->GetPosition();
                IPhysics::Quaternion quaternion =  rigidbody->GetOrientation();
                Matrix matrixPosition = MatrixTranslate(position.x, position.y, position.z);
                Vector4 rayQuaternion{(float)quaternion.i, (float)quaternion.j, (float)quaternion.k, (float)quaternion.r};
                Matrix rotation = QuaternionToMatrix(rayQuaternion);

                Model* model = Map[object];
                model->transform = MatrixMultiply(rotation, matrixPosition);
                DrawModel(*model, (Vector3){0,0,0}, geometry->GetScale(), geometry->GetColor());

                Vector3 rayPosition{(float)position.x, (float)position.y, (float)position.z};

                if(accelerationIndicatorEnabled){
                    IPhysics::Vector3 acceleration = rigidbody->GetAcceleration();
                    Vector3 rayAcceleration = {(float)acceleration.x, (float)acceleration.y, (float)acceleration.z};
                    Vector3 finalRayPosition = rayPosition + rayAcceleration;

                    DrawLine3D(rayPosition, finalRayPosition, BLUE);
                }
                
                if(velocityIndicatorEnabled){
                    IPhysics::Vector3 velocity = rigidbody->GetVelocity();
                    Vector3 rayVelocity = {(float)velocity.x, (float)velocity.y, (float)velocity.z};
                    Vector3 finalRayPosition = rayPosition + rayVelocity;

                    DrawLine3D(rayPosition, finalRayPosition, YELLOW);
                }

                ++iterator;
            }



        EndMode3D();
        
        // List Button
        if (GuiButton((Rectangle){ 24, 24, 24, 24 }, "#214#")) {
            if(leftHandGuiState == LeftHandSideGuiState::ListObjectBox){
                leftHandGuiState = LeftHandSideGuiState::None;
            }
            else{
                leftHandGuiState = LeftHandSideGuiState::ListObjectBox;
            }
        }

        // Add Button
        if (GuiButton((Rectangle){ 72, 24, 24, 24 }, "#80#")) {
            if(leftHandGuiState == LeftHandSideGuiState::AddObjectBox){
                leftHandGuiState = LeftHandSideGuiState::None;
            }
            else{
                leftHandGuiState = LeftHandSideGuiState::AddObjectBox;
            }
        }

        // Help Button
        if (GuiButton((Rectangle){ 120, 24, 24, 24 },"#193#")){
            if(leftHandGuiState == LeftHandSideGuiState::HelpBox){
                leftHandGuiState = LeftHandSideGuiState::None;
            }
            else{
                leftHandGuiState = LeftHandSideGuiState::HelpBox;
            }
        }

        // Pause Button
        if (GuiButton((Rectangle){ 168, 24, 24, 24 }, pauseButtonText.c_str())){
            if (world.GetPhysicsState()){
                pauseButtonText = "#131#";
                world.SetPhysicsState(false);
            }
            else{
                pauseButtonText = "#132#";
                world.SetPhysicsState(true);
            }
        }

        // Settings Button
        if (GuiButton((Rectangle){ 1224, 24, 24, 24 },"#142#")){
            if(rightHandGuiState == RightHandSideGuiState::SettingsBox){
                rightHandGuiState = RightHandSideGuiState::None;
            }
            else{
                rightHandGuiState = RightHandSideGuiState::SettingsBox;
            }
        }

        if(leftHandGuiState == LeftHandSideGuiState::ListObjectBox){
            ShowListMenu();
            ShowListView();
        }

        if (leftHandGuiState == LeftHandSideGuiState::AddObjectBox){
            AddObjectMenu();
            ShowAddView();
        }

        if(leftHandGuiState == LeftHandSideGuiState::HelpBox){
            ShowHelpMenu();
        }

        // Displaying Errors
        Vector2 startingPosition = { 960, 648 };
        for (auto iterator = errors.begin(); iterator != errors.end();){
            IApp::Error error = *iterator;
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::FatalError){
                break;
            }
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::NormalError){
                DrawRectangle( startingPosition.x, startingPosition.y, 288, 48, RED);
                DrawRectangleLines( startingPosition.x, startingPosition.y, 288, 48, MAROON);

                DrawText(error.GetErrorTitle().c_str(), startingPosition.x + 8, startingPosition.y + 8, 10, WHITE);
                DrawText(error.GetErrorMessage().c_str(), startingPosition.x + 8, startingPosition.y + 24, 10, LIGHTGRAY);
                if(GuiButton({startingPosition.x + 256, startingPosition.y + 8, 24, 24}, "#113#")){
                    iterator = errors.erase(iterator);
                    break;
                }
            }
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::Warning){
                DrawRectangle( startingPosition.x, startingPosition.y, 288, 48, YELLOW);
                DrawRectangleLines( startingPosition.x, startingPosition.y, 288, 48, BROWN);

                DrawText(error.GetErrorTitle().c_str(), startingPosition.x + 8, startingPosition.y + 8, 10, WHITE);
                DrawText(error.GetErrorMessage().c_str(), startingPosition.x + 8, startingPosition.y + 24, 10, LIGHTGRAY);
                if(GuiButton({startingPosition.x + 256, startingPosition.y + 8, 24, 24}, "#113#")){
                    iterator = errors.erase(iterator);
                    break;
                }
                
            }
            if(error.GetErrorSeverity() == IApp::ErrorSeverity::Information){
                DrawRectangle( startingPosition.x, startingPosition.y, 288, 48, LIGHTGRAY);
                DrawRectangleLines( startingPosition.x, startingPosition.y, 288, 48, DARKGRAY);

                DrawText(error.GetErrorTitle().c_str(), startingPosition.x + 8, startingPosition.y + 8, 10, WHITE);
                DrawText(error.GetErrorMessage().c_str(), startingPosition.x + 8, startingPosition.y + 24, 10, LIGHTGRAY);
                if(GuiButton({startingPosition.x + 256, startingPosition.y + 8, 24, 24}, "#113#")){
                    iterator = errors.erase(iterator);
                    break;
                }
                
            }

            ++iterator;
            startingPosition.y -= 72;

        }

        if(rightHandGuiState == RightHandSideGuiState::SettingsBox){
            ShowSettingsMenu();
        }

    EndDrawing();
}

int main(void)
{
    IApp::LanguageManager::Initialise();
    std::string initialLanguage = "English (UK)";
    IApp::LanguageManager::LoadLanguage(initialLanguage);
    InitialiseGUI();
    ResetAddObject();


    std::cout << "Working dir: " << std::filesystem::current_path() << std::endl;



    // Example object 1
    IPhysics::Vector3 velocity(0, 0, -5);
    IPhysics::Object* object1 = new IPhysics::Object();
    IPhysics::Geometry* geometry1 = object1->AddComponent<IPhysics::Geometry>();
    IPhysics::RigidBody* rigidbody1 = object1->AddComponent<IPhysics::RigidBody>();
    IPhysics::Information* information1 = object1->AddComponent<IPhysics::Information>();
    IPhysics::Vector3 position1(10.0, 0.0, 0.0);
    IPhysics::real mass1 = 7.5 * pow(10, 12);
    IPhysics::Quaternion quaternion1(0, 0, 0, 1);
    IPhysics::real damping = 1;
    rigidbody1->SetPosition(position1);
    rigidbody1->SetMass(mass1);
    rigidbody1->SetOrientation(quaternion1);
    rigidbody1->SetLinearDamping(damping);
    rigidbody1->SetAngularDamping(damping);
    rigidbody1->SetInverseInertiaTensor(standardTensor);
    rigidbody1->AddVelocity(velocity);
    geometry1->SetMesh(IApp::MeshManager::GetMesh("Sphere"));
    geometry1->SetColor(Color(RED));
    geometry1->SetScale(1.0);
    std::string name1 = "One";
    information1->SetName(name1);
    Model* model1 = new Model(LoadModelFromMesh(*geometry1->GetMesh()));
    Map.emplace(object1, model1);

    // Example object 2
    IPhysics::Object* object2 = new IPhysics::Object();
    IPhysics::Geometry* geometry2 = object2->AddComponent<IPhysics::Geometry>();
    IPhysics::RigidBody* rigidbody2 = object2->AddComponent<IPhysics::RigidBody>();
    IPhysics::Information* information2 = object2->AddComponent<IPhysics::Information>();
    IPhysics::Vector3 position2(-10.0, 0.0, 0.0);
    IPhysics::real mass2 = 7.5 * pow(10, 12);
    IPhysics::Quaternion quaternion2(0, 0, 0, 1);
    rigidbody2->SetPosition(position2);
    rigidbody2->SetMass(mass2);
    rigidbody2->SetOrientation(quaternion2);
    rigidbody2->SetLinearDamping(damping);
    rigidbody2->SetAngularDamping(damping);
    rigidbody2->SetInverseInertiaTensor(standardTensor);
    geometry2->SetMesh(IApp::MeshManager::GetMesh("Sphere"));
    geometry2->SetColor(Color(BLUE));
    geometry2->SetScale(1.0);
    std::string name2 = "Two";
    information2->SetName(name2);
    Model* model2 = new Model(LoadModelFromMesh(*geometry2->GetMesh()));
    Map.emplace(object2, model2);

    realGravity->AddObject(object1);
    realGravity->AddObject(object2);
    world.AddForceRegistry(object1, realGravity);
    world.AddForceRegistry(object2, realGravity);
    world.AddObject(object1);
    world.AddObject(object2);

    #ifdef __EMSCRIPTEN__
        emscripten_set_main_loop(Update, 0, 1);
    #else
        while (!WindowShouldClose())
        {
            Update();
        }
        CloseWindow();
    #endif


    CloseWindow();

    return 0;
}

