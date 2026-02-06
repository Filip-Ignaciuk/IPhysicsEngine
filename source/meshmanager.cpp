#include "meshmanager.hpp"

IPhysicsEngine::MeshManager::MeshManager(){
    Mesh cubeMesh = GenMeshCube(1.0f, 1.0f, 1.0f);
    Mesh sphereMesh = GenMeshSphere(1.0f, 32, 64);
    Mesh cylinderMesh = GenMeshSphere(1.0f, 32, 64);
    meshes.emplace("Cube", &cubeMesh);
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
    }
    return meshNames;
    
}
