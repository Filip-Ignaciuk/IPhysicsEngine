#include "forcegenerator.hpp"

#include <iterator>

IPhysics::ForceRegistry::ForceRegistry(){
    constexpr std::vector<ForceRegistration> temporary;
    registrations = temporary;
}

void IPhysics::ForceRegistry::Add(Object* _object, const std::shared_ptr<ForceGenerator>& _forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = _object->GetComponent<RigidBody>();
    forceRegistration.forceGenerator = _forceGenerator;
    registrations.emplace_back(forceRegistration);
}

void IPhysics::ForceRegistry::Remove(Object* _object, const std::shared_ptr<ForceGenerator>& _forceGenerator){
    ForceRegistration forceRegistration{};
    forceRegistration.rigidBody = _object->GetComponent<RigidBody>();
    forceRegistration.forceGenerator = _forceGenerator;
    std::erase(registrations, forceRegistration);
}

void IPhysics::ForceRegistry::Remove(Object* _object){
    auto* rigidBody = _object->GetComponent<RigidBody>();
    for(const ForceRegistration& forceRegistration : registrations){
        if(forceRegistration.rigidBody == rigidBody){
            std::erase(registrations, forceRegistration);
        }
    }
}

void IPhysics::ForceRegistry::RemoveAll() {
    registrations.clear();
}

const IPhysics::ForceRegistration* IPhysics::ForceRegistry::Get(Object* _object) const {
    for (const ForceRegistration& registration : registrations) {
        if (registration.rigidBody == _object->GetComponent<RigidBody>()) {
            return &registration;
        }
    }
    return nullptr;
}

void IPhysics::ForceRegistry::Clear(){
    registrations.clear();
}

void IPhysics::ForceRegistry::UpdateForces(real _duration){
    auto iterator = registrations.begin();

    while (iterator != registrations.end())
    {
        const ForceRegistration& forceRegistration = *iterator;
        forceRegistration.forceGenerator->UpdateForce(forceRegistration.rigidBody, _duration);
        ++iterator;
    }
}
