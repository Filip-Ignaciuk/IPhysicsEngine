#include "particle.hpp"

namespace IPhysicsEngine
{
    IPhysicsEngine::Particle::Particle(){

    }

    IPhysicsEngine::Particle::Particle(Vector3 _position, real _damping, real _inverseMass) : m_position(_position), m_damping(_damping), m_inverseMass(_inverseMass){

    }

    void IPhysicsEngine::Particle::SetMass(real _mass){
        if (m_inverseMass <= 0.0f){
            return;
        }
        m_inverseMass = 1 / _mass;
    }

    void IPhysicsEngine::Particle::SetInverseMass(real _inverseMass){
        if (m_inverseMass < 0.0f){
            return;
        }
        m_inverseMass = _inverseMass;
    }

    void IPhysicsEngine::Particle::SetAcceleration(Vector3 _acceleration){
        m_acceleration = _acceleration;
    }

    void IPhysicsEngine::Particle::Integrate(real _duration){
        if (m_inverseMass <= 0.0f){
            return;
        }

        m_position.AddScaledVector(m_velocity, _duration);

        Vector3 resultingAcceleration = m_acceleration;

        m_velocity.AddScaledVector(resultingAcceleration, _duration);

        m_velocity *= RealPow(m_damping, _duration);

    }

    real IPhysicsEngine::Particle::GetKineticEnergy(){
        if (m_inverseMass <= 0.0f){
            return 0;
        }

        return 0.5 * (1 / m_inverseMass) * m_velocity.SquareMagnitude();
    }

    Vector3 IPhysicsEngine::Particle::GetPosition(){
        return m_position;
    }

    Vector3 IPhysicsEngine::Particle::GetVelocity(){
        return m_velocity;
    }

    Vector3 IPhysicsEngine::Particle::GetAcceleration(){
        return m_acceleration;
    }
    
}