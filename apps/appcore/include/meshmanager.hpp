#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "raylib.h"

namespace IApp{
    class MeshManager
    {
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
        static Mesh* GetMesh(std::string _meshName);
        static Model* GetModel(std::string _modelName);
        static Color GetColor(std::string _colour);
        static std::vector<std::string> GetMeshStrings();
        static std::vector<std::string> GetColourStrings();
        static void SetMesh(std::string& _meshName, Mesh* _mesh);

    };
}