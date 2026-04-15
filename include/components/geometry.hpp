#pragma once
#include "raylib.h"
#include "components/component.hpp"
#include "core.hpp"

namespace IPhysicsEngine{
    class Geometry : public Component{
        private:
        Mesh* m_mesh;
        float m_scale;
        Color m_color;

        public:
        Geometry();

        Mesh* GetMesh();
        real GetScale();
        Color GetColor();

        void SetMesh(Mesh* _mesh);
        void SetScale(float _scale);
        void SetColor(Color _color);

    };
}