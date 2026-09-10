#ifndef IPHYSICS_MESHMANAGER_HPP
#define IPHYSICS_MESHMANAGER_HPP
#include <string>
#include <unordered_map>
#include <vector>

#include "raylib.h"

namespace IApp {
class MeshManager {
 private:
  typedef std::unordered_map<std::string, Mesh> MeshMap;
  typedef std::unordered_map<std::string, Model> ModelMap;
  typedef std::unordered_map<std::string, Color> MeshColours;
  static MeshMap meshes;
  static ModelMap models;
  static MeshColours meshColours;

 public:
  static void LoadDefaults();
  static void Unload();
  static Mesh* GetMesh(std::string meshName);
  static Model* GetModel(std::string modelName);
  static Color GetColor(std::string colour);
  static std::vector<std::string> GetMeshStrings();
  static std::vector<std::string> GetColourStrings();
  static void SetMesh(std::string& meshName, Mesh* mesh);
};
}  // namespace IApp

#endif