#pragma once
#include "particle.hpp"

namespace IPhysicsEngine{

    class Firework : public Particle{
        private:
            unsigned m_type;
            real m_age;
        public:
        Firework();
        bool Integrate(real _duration) override;
        
        void SetType(unsigned _type);
        void SetAge(unsigned _age);

        unsigned GetType();
        real GetAge();

    };
    struct Payload{
            unsigned type;
            unsigned count;
            Payload() : type(0), count(0) {}
            void Set(unsigned _type, unsigned _count){
                Payload::type = _type;
                Payload::count = _count;
            }
    };
    struct FireworkRule
    {
        unsigned type;

        real minAge;
        real maxAge;
        Vector3 minVelocity;
        Vector3 maxVelocity;
        real damping;

        

        unsigned payloadCount;

        Payload* payloads;
    
        FireworkRule();

        void Initialise(unsigned _payloadCount);
        void SetParameters(unsigned _type, real _minAge, real _maxAge, const Vector3& _minVelocity, const Vector3& _maxVelocity, real _damping);
        void Create(Firework* _firework, const Firework* _parent = NULL) const;
    };
    


    

    class FireworkManager{
        private:
            const static inline unsigned maxFireworks = 1024;
            static inline Firework fireworks[maxFireworks];
            static inline unsigned nextFirework = 0;
            const static unsigned ruleCount = 9;
            static inline FireworkRule fireworkRules[ruleCount];
            

        public:
            static void Initialise();
            static int Update(real _duration);

            static unsigned GetMaxFireworks();
            static Firework* GetFireworks();
            static FireworkRule* GetFireworkRules();

            static void Create(unsigned _type, const Firework* _parent);
            static void Create(unsigned _type, unsigned _number, const Firework* _parent);
    };
}