#pragma once
#include <unordered_map>

#include "raylib.h"
#include "core.hpp"

namespace IPhysics{
    class MeshManager
    {
    private:
        typedef std::unordered_map<std::string, Mesh*> Map;
        static Map meshes;
    public:
        static void LoadDefaults();
        static void Unload();
        static Mesh* GetMesh(std::string _meshName);
        static std::vector<std::string> GetMeshStrings();
        static void SetMesh(std::string& _meshName, Mesh* _mesh);

    };
}