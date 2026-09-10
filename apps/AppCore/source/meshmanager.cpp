#include "meshmanager.hpp"

IApp::MeshManager::MeshMap IApp::MeshManager::meshes;
IApp::MeshManager::ModelMap IApp::MeshManager::models;
IApp::MeshManager::MeshColours IApp::MeshManager::meshColours;

void IApp::MeshManager::LoadDefaults() {
  Mesh cubeMesh = GenMeshCube(1.0f, 1.0f, 1.0f);
  Mesh sphereMesh = GenMeshSphere(1.0f, 32, 64);
  meshes.emplace("Box", cubeMesh);
  meshes.emplace("Sphere", sphereMesh);
  Model boxModel = LoadModelFromMesh(cubeMesh);
  Model sphereModel = LoadModelFromMesh(sphereMesh);
  models.emplace("Box", boxModel);
  models.emplace("Sphere", sphereModel);
  meshColours.emplace("Red", RED);
  meshColours.emplace("Green", GREEN);
  meshColours.emplace("Blue", BLUE);
}

void IApp::MeshManager::Unload() {
  meshes.clear();
  models.clear();
}

Mesh* IApp::MeshManager::GetMesh(std::string meshName) {
  MeshMap::iterator mapIterator = meshes.find(meshName);
  if (mapIterator != meshes.end()) {
    return &mapIterator->second;
  }
  return nullptr;
}

Model* IApp::MeshManager::GetModel(std::string modelName) {
  ModelMap::iterator mapIterator = models.find(modelName);
  if (mapIterator != models.end()) {
    return &mapIterator->second;
  }
  return nullptr;
}

Color IApp::MeshManager::GetColor(std::string colour) {
  MeshColours::iterator mapIterator = meshColours.find(colour);
  if (mapIterator != meshColours.end()) {
    return mapIterator->second;
  }
  return BLACK;
}

std::vector<std::string> IApp::MeshManager::GetMeshStrings() {
  std::vector<std::string> meshStrings;
  MeshMap::iterator mapIterator = meshes.begin();
  while (mapIterator != meshes.end()) {
    meshStrings.emplace_back(mapIterator->first);
    ++mapIterator;
  }
  return meshStrings;
}

std::vector<std::string> IApp::MeshManager::GetColourStrings() {
  std::vector<std::string> meshColourStrings;
  MeshColours::iterator mapIterator = meshColours.begin();
  while (mapIterator != meshColours.end()) {
    meshColourStrings.emplace_back(mapIterator->first);
    ++mapIterator;
  }
  return meshColourStrings;
}