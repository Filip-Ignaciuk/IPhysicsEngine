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

    void IPhysicsEngine::Particle::SetDamping(real _damping){
        m_damping = _damping;
    }

    void IPhysicsEngine::Particle::SetPosition(Vector3 _position){
        m_position = _position;
    }

    void IPhysicsEngine::Particle::SetVelocity(Vector3 _velocity){
        m_velocity = _velocity;
    }

    void IPhysicsEngine::Particle::SetAcceleration(Vector3 _acceleration){
        m_acceleration = _acceleration;
    }

    bool IPhysicsEngine::Particle::Integrate(real _duration){
        if (m_inverseMass <= 0.0f){
            return false;
        }

        m_position.AddScaledVector(m_velocity, _duration);

        Vector3 resultingAcceleration = m_acceleration;
        resultingAcceleration.AddScaledVector(m_forceAccumulated, m_inverseMass);


        m_velocity *= RealPow(m_damping, _duration);

        m_velocity.AddScaledVector(resultingAcceleration, _duration);

        ClearAccumulator();

        return true;

    }

    void IPhysicsEngine::Particle::ClearAccumulator(){
        m_forceAccumulated.Clear();
    }

    real IPhysicsEngine::Particle::GetKineticEnergy() const{
        if (m_inverseMass <= 0.0f){
            return 0;
        }

        return 0.5 * (1 / m_inverseMass) * m_velocity.SquareMagnitude();
    }

    Vector3 IPhysicsEngine::Particle::GetPosition() const{
        return m_position;
    }

    Vector3 IPhysicsEngine::Particle::GetVelocity() const{
        return m_velocity;
    }

    Vector3 IPhysicsEngine::Particle::GetAcceleration() const{
        return m_acceleration;
    }

    void IPhysicsEngine::Particle::AddForce(Vector3 _force){
        m_forceAccumulated += _force;
    }
    
}