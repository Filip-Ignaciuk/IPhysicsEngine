#ifndef IPHYSICS_WORLD_HPP
#define IPHYSICS_WORLD_HPP

#include <vector>
#include <algorithm>

#include "object.hpp"
#include "collidebroad.hpp"
#include "collidenarrow.hpp"
#include "forcegenerator.hpp"
#include "contacts.hpp"

namespace IPhysics
{
    class World{
        public:
        // Mutators
        void StartFrame();
        void RunPhysics(real timestep);

        void AddObject(Object* object);
        void RemoveObject(Object* object);
        void RemoveLastObject();

        void AddForceRegistration(Object* object,
            const std::shared_ptr<ForceGenerator>& forceGenerator);
        void RemoveForceRegistration(Object* object);

        void SetPhysicsState(bool state);

        // Queries
        [[nodiscard]] const std::vector<Object*>& GetObjects();

        [[nodiscard]] const ForceRegistration& GetForceRegistration(Object* object) const;

        [[nodiscard]] const ForceRegistry& GetForceRegistry() const;

        [[nodiscard]] bool GetPhysicsState() const;

        protected:
        static constexpr unsigned MAX_CONTACTS = 256;

        std::vector<Object*> m_objects;
        bool m_physicsState = false;

        BoundingVolumeHierarchyNode<BoundingSphere>* m_root = nullptr;
        ContactResolver m_contactResolver{};
        ForceRegistry m_registry;
    };
}

#endif