#include "particle.hpp"

namespace IPhysics
{
    IPhysics::Particle::Particle() : m_inverseMass(1.0f), m_damping(0.1f), m_acceleration(Origin), m_forceAccumulated(Origin), m_position(Origin), m_velocity(Origin){

    }

    IPhysics::Particle::Particle(Vector3 position, real damping, real inverseMass) : m_position(position), m_damping(damping), m_inverseMass(inverseMass){

    }

    void IPhysics::Particle::SetMass(real mass){
        if (m_inverseMass <= 0.0f){
            return;
        }
        m_inverseMass = 1 / mass;
    }

    void IPhysics::Particle::SetInverseMass(real inverseMass){
        if (m_inverseMass < 0.0f){
            return;
        }
        m_inverseMass = inverseMass;
    }

    void IPhysics::Particle::SetDamping(real damping){
        m_damping = damping;
    }

    void IPhysics::Particle::SetPosition(Vector3 position){
        m_position = position;
    }

    void IPhysics::Particle::SetVelocity(Vector3 velocity){
        m_velocity = velocity;
    }

    void IPhysics::Particle::SetAcceleration(Vector3 acceleration){
        m_acceleration = acceleration;
    }

    bool IPhysics::Particle::Integrate(real duration){
        // Reject infinite masses.
        if (m_inverseMass <= 0.0f){
            return false;
        }

        // Update linear position.
        m_position.AddScaledVector(m_velocity, duration);

        // Work out acceleration from force.
        Vector3 resultingAcceleration = m_acceleration;
        resultingAcceleration.AddScaledVector(m_forceAccumulated, m_inverseMass);

        // Update linear velocity.
        m_velocity.AddScaledVector(resultingAcceleration, duration);
        // Add drag.
        m_velocity *= RealPow(m_damping, duration);

        // Clear forces.
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

    void IPhysics::Particle::AddForce(Vector3 force){
        m_forceAccumulated += force;
    }
    
}