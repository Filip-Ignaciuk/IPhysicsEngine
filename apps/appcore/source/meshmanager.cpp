#include "meshmanager.hpp"

IApp::MeshManager::MeshMap IApp::MeshManager::meshes;
IApp::MeshManager::MeshColours IApp::MeshManager::meshColours;

void IApp::MeshManager::LoadDefaults(){
    Mesh* cubeMesh = new Mesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    Mesh* sphereMesh = new Mesh(GenMeshSphere(1.0f, 32, 64));
    Mesh* cylinderMesh = new Mesh(GenMeshCylinder(1.0f, 32, 64));
    meshes.emplace("Box", cubeMesh);
    meshes.emplace("Sphere", sphereMesh);
    meshes.emplace("Cylinder", cylinderMesh);
    meshColours.emplace("Red", RED);
    meshColours.emplace("Green", GREEN);
    meshColours.emplace("Blue", BLUE);
}

void IApp::MeshManager::Unload(){
    for(auto& pair : meshes){
        delete pair.second;
    }
    meshes.clear();
}

Mesh* IApp::MeshManager::GetMesh(std::string _meshName){
    MeshMap::iterator mapIterator = meshes.find(_meshName);
    if(mapIterator != meshes.end()){
        return mapIterator->second;
    }
    return nullptr;
}

Color IApp::MeshManager::GetColor(std::string _colour){
    MeshColours::iterator mapIterator = meshColours.find(_colour);
    if(mapIterator != meshColours.end()){
        return mapIterator->second;
    }
    return BLACK;
}

std::vector<std::string> IApp::MeshManager::GetMeshStrings(){
    std::vector<std::string> meshStrings;
    MeshMap::iterator mapIterator = meshes.begin();
    while (mapIterator != meshes.end())
    {
        meshStrings.emplace_back(mapIterator->first);
        ++mapIterator;
    }
    return meshStrings;
    
}

std::vector<std::string> IApp::MeshManager::GetColourStrings(){
    std::vector<std::string> meshColourStrings;
    MeshColours::iterator mapIterator = meshColours.begin();
    while (mapIterator != meshColours.end())
    {
        meshColourStrings.emplace_back(mapIterator->first);
        ++mapIterator;
    }
    return meshColourStrings;
    
}