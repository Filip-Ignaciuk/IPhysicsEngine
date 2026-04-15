#include "meshmanager.hpp"

IPhysicsEngine::MeshManager::Map IPhysicsEngine::MeshManager::meshes;

void IPhysicsEngine::MeshManager::LoadDefaults(){
    Mesh* cubeMesh = new Mesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    Mesh* sphereMesh = new Mesh(GenMeshSphere(1.0f, 32, 64));
    Mesh* cylinderMesh = new Mesh(GenMeshCylinder(1.0f, 32, 64));
    meshes.emplace("Box", cubeMesh);
    meshes.emplace("Sphere", sphereMesh);
    meshes.emplace("Cylinder", cylinderMesh);
}

void IPhysicsEngine::MeshManager::Unload(){
    for(auto& pair : meshes){
        delete pair.second;
    }
    meshes.clear();
}

Mesh* IPhysicsEngine::MeshManager::GetMesh(std::string _meshName){
    Map::iterator mapIterator = meshes.find(_meshName);
    if(mapIterator != meshes.end()){
        return mapIterator->second;
    }
    return nullptr;
}

std::vector<std::string> IPhysicsEngine::MeshManager::GetMeshStrings(){
    std::vector<std::string> meshNames;
    Map::iterator mapIterator = meshes.begin();
    while (mapIterator != meshes.end())
    {
        meshNames.emplace_back(mapIterator->first);
        ++mapIterator;
    }
    return meshNames;
    
}
