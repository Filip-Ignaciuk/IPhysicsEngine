#include "meshmanager.hpp"
#include <raylib.h>
#include <utility>

/*
 *  MeshManager
 */

// Statics
void IApp::MeshManager::LoadDefaults() {
  Mesh cubeMesh = GenMeshCube(1.0f, 1.0f, 1.0f);
  Mesh sphereMesh = GenMeshSphere(1.0f, 32, 64);

  meshes.emplace("Box", cubeMesh);
  meshes.emplace("Sphere", sphereMesh);

  Model boxModel = LoadModelFromMesh(cubeMesh);
  Model sphereModel = LoadModelFromMesh(sphereMesh);

  models.emplace("Box", boxModel);
  models.emplace("Sphere", sphereModel);

  meshColors.emplace("Red", RED);
  meshColors.emplace("Green", GREEN);
  meshColors.emplace("Blue", BLUE);
}

void IApp::MeshManager::Unload() {
  // Unload Models
  for(std::pair<std::string, Model> modelPair : models){
    UnloadModel(modelPair.second);
  }

  // Unload Meshes
  for(std::pair<std::string, Mesh> meshPair : meshes){
    UnloadMesh(meshPair.second);
  }

  models.clear();
  meshes.clear();
  meshColors.clear();
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

const Color& IApp::MeshManager::GetColor(std::string color) {
  MeshColors::iterator mapIterator = meshColors.find(color);
  if (mapIterator != meshColors.end()) {
    return mapIterator->second;
  }
  return Black;
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
  MeshColors::iterator mapIterator = meshColors.begin();
  while (mapIterator != meshColors.end()) {
    meshColourStrings.emplace_back(mapIterator->first);
    ++mapIterator;
  }
  return meshColourStrings;
}

void IApp::MeshManager::LoadMesh(const std::string& meshName, const Mesh& mesh){
  meshes.emplace(meshName, mesh);
  Model model = LoadModelFromMesh(mesh);
  models.emplace(meshName, model);
}

IApp::MeshManager::MeshMap IApp::MeshManager::meshes;
IApp::MeshManager::ModelMap IApp::MeshManager::models;
IApp::MeshManager::MeshColors IApp::MeshManager::meshColors;