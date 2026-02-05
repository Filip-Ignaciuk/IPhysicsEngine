#pragma once
#include <unordered_map>

#include "raylib.h"
#include "core.hpp"

namespace IPhysicsEngine{
    class MeshManager
    {
    private:
        typedef std::unordered_map<std::string, Mesh*> Map;
        Map meshes;
    public:
        MeshManager();
        
        Mesh* GetMesh(std::string _meshName);
        std::vector<std::string> GetMeshStrings();
        void SetMesh(std::string& _meshName, Mesh* _mesh);

    };
}