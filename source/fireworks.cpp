#include "fireworks.hpp"

bool IPhysicsEngine::Firework::Integrate(real _duration){
    Particle::Integrate(_duration);
    m_age -= _duration;
    return (m_age < 0) || (m_position.GetY() < 0);
}