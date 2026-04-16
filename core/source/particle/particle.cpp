#include "particle.hpp"

namespace IPhysics
{
    IPhysics::Particle::Particle() : m_inverseMass(1.0f), m_damping(0.1f), m_acceleration(Origin), m_forceAccumulated(Origin), m_position(Origin), m_velocity(Origin){

    }

    IPhysics::Particle::Particle(Vector3 _position, real _damping, real _inverseMass) : m_position(_position), m_damping(_damping), m_inverseMass(_inverseMass){

    }

    void IPhysics::Particle::SetMass(real _mass){
        if (m_inverseMass <= 0.0f){
            return;
        }
        m_inverseMass = 1 / _mass;
    }

    void IPhysics::Particle::SetInverseMass(real _inverseMass){
        if (m_inverseMass < 0.0f){
            return;
        }
        m_inverseMass = _inverseMass;
    }

    void IPhysics::Particle::SetDamping(real _damping){
        m_damping = _damping;
    }

    void IPhysics::Particle::SetPosition(Vector3 _position){
        m_position = _position;
    }

    void IPhysics::Particle::SetVelocity(Vector3 _velocity){
        m_velocity = _velocity;
    }

    void IPhysics::Particle::SetAcceleration(Vector3 _acceleration){
        m_acceleration = _acceleration;
    }

    bool IPhysics::Particle::Integrate(real _duration){
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

    void IPhysics::Particle::ClearAccumulator(){
        m_forceAccumulated.Clear();
    }

    real IPhysics::Particle::GetKineticEnergy() const{
        if (m_inverseMass <= 0.0f){
            return 0;
        }

        return 0.5 * (1 / m_inverseMass) * m_velocity.SquareMagnitude();
    }

    Vector3 IPhysics::Particle::GetPosition() const{
        return m_position;
    }

    Vector3 IPhysics::Particle::GetVelocity() const{
        return m_velocity;
    }

    Vector3 IPhysics::Particle::GetAcceleration() const{
        return m_acceleration;
    }

    real IPhysics::Particle::GetDamping() const{
        return m_damping;
    }
    
    real IPhysics::Particle::GetInverseMass() const{
        return m_inverseMass;
    }

    real IPhysics::Particle::GetMass() const{
        return 1 / m_inverseMass;
    }

    void IPhysics::Particle::AddForce(Vector3 _force){
        m_forceAccumulated += _force;
    }
    
}