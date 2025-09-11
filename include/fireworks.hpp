#pragma once

#include "particle.hpp"

namespace IPhysicsEngine{

    struct FireworkRule
    {
        unsigned type;

        real minAge;
        real maxAge;
        Vector3 minVelocity;
        Vector3 maxVelocity;
        real damping;

        struct Payload{
            unsigned type;
            unsigned count;
            void set(unsigned _type, unsigned _count){
                Payload::type = _type;
                Payload::count = _count;
            }
        };

        void Create(Firework* _firework, const Firework* _parent = NULL) const{
            _firework->SetType(type);
            _firework->SetAge(RandomReal(minAge, maxAge));

            Vector3 velocity;
            if (_parent){
                _firework->SetPosition(_parent->GetPosition());
                velocity += _parent->GetVelocity();
            }
            else{
                int positionx = RandomInt(0, 3) - 1;
                Vector3 start(positionx * 5.0f, 0,0);
                _firework->SetPosition(start);

            }
            velocity += RandomVector3(minVelocity.Magnitude(), maxVelocity.Magnitude());
            _firework->SetVelocity(velocity);

            _firework->SetMass(1);
            _firework->SetDamping(damping);
            _firework->SetAcceleration(Gravity);
            _firework->ClearAccumulator();
        }

        unsigned payloadCount;

        Payload* payloads;
    };
    


    class Firework : public Particle{
        private:
            unsigned m_type;
            real m_age;
        public:
        bool Integrate(real _duration) override;
        
        void SetType(unsigned _type);
        void SetAge(unsigned _age);

        unsigned GetAge(unsigned _age);
        unsigned GetType(unsigned _type);

    };
}