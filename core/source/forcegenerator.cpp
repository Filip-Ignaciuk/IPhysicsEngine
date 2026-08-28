#include "forcegenerator.hpp"

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
            registrations.erase(remove(registrations.begin(), registrations.end(), forceRegistration), registrations.end());
        }
    }
}

void IPhysics::ForceRegistry::RemoveAll() {
    registrations.clear();
}

IPhysics::ForceRegistration* IPhysics::ForceRegistry::Get(Object* _object) const {
    for (ForceRegistration forceRegistration : registrations) {
        if (forceRegistration.rigidBody == _object->GetComponent<RigidBody>()) {
            return &forceRegistration;
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
        ForceRegistration forceRegistration = *iterator;
        forceRegistration.forceGenerator->UpdateForce(forceRegistration.rigidBody, _duration);
        ++iterator;
    }
}
