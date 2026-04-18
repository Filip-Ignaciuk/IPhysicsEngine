#pragma once
#include <unordered_map>

#include "raylib.h"
#include "core.hpp"

namespace IApp{
    class MeshManager
    {
    private:
        typedef std::unordered_map<std::string, Mesh*> MeshMap;
        typedef std::unordered_map<std::string, Color> MeshColours;
        static MeshMap meshes;
        static MeshColours meshColours;
    public:
        static void LoadDefaults();
        static void Unload();
        static Mesh* GetMesh(std::string _meshName);
        static Color GetColor(std::string _colour);
        static std::vector<std::string> GetMeshStrings();
        static std::vector<std::string> GetColourStrings();
        static void SetMesh(std::string& _meshName, Mesh* _mesh);

    };
}