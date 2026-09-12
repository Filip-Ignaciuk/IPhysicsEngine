#ifndef IPHYSICS_MESHMANAGER_HPP
#define IPHYSICS_MESHMANAGER_HPP
#include <string>
#include <unordered_map>
#include <vector>

#include "raylib.h"

namespace IApp {
class MeshManager final{
 public:
  // Statics
  static void LoadDefaults();
  static void Unload();

  static Mesh* GetMesh(std::string meshName);
  static Model* GetModel(std::string modelName);
  static const Color& GetColor(std::string color);

  static std::vector<std::string> GetMeshStrings();
  static std::vector<std::string> GetColourStrings();

  static void LoadMesh(const std::string& meshName, const Mesh& mesh);
  static void LoadColour(const std::string& colourName, const Color& color);

 private:
  typedef std::unordered_map<std::string, Mesh> MeshMap;
  typedef std::unordered_map<std::string, Model> ModelMap;
  typedef std::unordered_map<std::string, Color> MeshColors;

  // Statics
  static MeshMap meshes;
  static ModelMap models;
  static MeshColors meshColors;
};
}  // namespace IApp

#endif