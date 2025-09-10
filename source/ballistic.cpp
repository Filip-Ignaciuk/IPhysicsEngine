#include "ballistic.hpp"


IPhysicsEngine::BallisticParticle::BallisticParticle(Vector3 _position, real _damping, real _inverseMass) : Particle(_position, _damping, _inverseMass){
}

bool IPhysicsEngine::BallisticParticle::Integrate(real _duration){
    Particle::Integrate(_duration);
    
    if (m_position.GetY() < 0){
        return false;
    }
    return true;

}
