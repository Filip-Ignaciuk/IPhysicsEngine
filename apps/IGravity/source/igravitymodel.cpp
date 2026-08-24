#include "igravitymodel.hpp"
#include <numbers>

/*
 * IGravityModel
 */

// Constructors
IGravityModel::IGravityModel(IPhysics::real _timeStep)
    : m_timeStep(_timeStep) {
}

// Mutators
void IGravityModel::SetupSimulation() {
    UpdateNumberOfParticles(1000);
}

void IGravityModel::UpdateSimulation() {
    m_world.StartFrame();
    if(m_world.GetPhysicsState()){
        m_world.RunPhysics(m_timeStep);
    }
}

// Can both increase and decrease the number of particles based on the count provided.
void IGravityModel::UpdateNumberOfParticles(int _count) {
    int boundedCount = 0;
    // Check if _count is larger than maximum or smaller than minimum.
    if (_count > MAXIMUM_PARTICLE_COUNT - m_numberOfParticles) {
        // Bound to maximum allowed increase, AKA add maximum amount of particles.
        boundedCount = MAXIMUM_PARTICLE_COUNT - m_numberOfParticles;
    }
    else if (_count < -m_numberOfParticles) {
        // Bound to minimum allowed decrease, AKA delete all particles.
        boundedCount = -m_numberOfParticles;
    }
    else {
        boundedCount = _count;
    }

    if (boundedCount < 0) {
        boundedCount = -boundedCount;
        for (int i = 0; i < boundedCount; ++i) {
            m_world.RemoveLastObject();
        }
    }
    else {
        const auto force_generator
            = std::make_shared<IPhysics::RealGravity>(6.674 * pow(10, -11));
        for (int i = 0; i < boundedCount; ++i) {
            // Creating object
            auto* object = new IPhysics::Object();
            auto* rigid_body = object->AddComponent<IPhysics::RigidBody>();
            // Setting random location
            IPhysics::Vector3 random_position
            = RandomGalaxyPosition();

            rigid_body->SetPosition(random_position);
            rigid_body->SetInverseMass(1.0 / (1.0 * pow(10, 12)));
            rigid_body->SetLinearDamping(1.0f);

            // Adding it to world
            m_world.AddObject(object);
            force_generator->AddObject(object);
            m_world.AddForceRegistration(object, force_generator);
        }
    }
}

// Queries
const std::vector<IPhysics::Object*>& IGravityModel::GetParticles() {
    return m_world.GetObjects();
}

IPhysics::Vector3 IGravityModel::RandomGalaxyPosition() {
    IPhysics::real scaleRadius = 40.0f;

    IPhysics::real randomNumber1 = IPhysics::RandomStore::RandomReal(0, 1);
    IPhysics::real randomNumber2 = IPhysics::RandomStore::RandomReal(0, 1);

    IPhysics::real r = -scaleRadius * log(1 - randomNumber1);

    IPhysics::real theta = 2 * std::numbers::pi * randomNumber2;

    IPhysics::real xPosition = 0 + r * RealCos(theta);
    IPhysics::real yPosition = 0 + r * RealSin(theta);
    return {xPosition, yPosition, 0};
}