#include "forcegenerator.hpp"

#include <iterator>

IPhysics::ForceRegistry::ForceRegistry(){
    constexpr std::vector<ForceRegistration> temporary;
    registrations = temporary;
}

void IPhysics::ForceRegistry::Add(Object* object, const std::shared_ptr<ForceGenerator>& forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = object->GetComponent<RigidBody>();
    forceRegistration.forceGenerator = forceGenerator;
    registrations.emplace_back(forceRegistration);
}

void IPhysics::ForceRegistry::Remove(Object* object, const std::shared_ptr<ForceGenerator>& forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = object->GetComponent<RigidBody>();
    forceRegistration.forceGenerator = forceGenerator;
    std::erase(registrations, forceRegistration);
}

void IPhysics::ForceRegistry::Remove(Object* object){
    auto* rigidBody = object->GetComponent<RigidBody>();
    for(const ForceRegistration& forceRegistration : registrations){
        if(forceRegistration.rigidBody == rigidBody){
            std::erase(registrations, forceRegistration);
        }
    }
}

void IPhysics::ForceRegistry::RemoveAll() {
    registrations.clear();
}

const IPhysics::ForceRegistration* IPhysics::ForceRegistry::Get(Object* object) const {
    for (const ForceRegistration& registration : registrations) {
        if (registration.rigidBody == object->GetComponent<RigidBody>()) {
            return &registration;
        }
    }
    return nullptr;
}

void IPhysics::ForceRegistry::Clear(){
    registrations.clear();
}

void IPhysics::ForceRegistry::UpdateForces(real duration){
    auto iterator = registrations.begin();

    while (iterator != registrations.end())
    {
        const ForceRegistration& forceRegistration = *iterator;
        forceRegistration.forceGenerator->UpdateForce(forceRegistration.rigidBody, duration);
        ++iterator;
    }
}
