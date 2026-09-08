#pragma once
#include "particle.hpp"

namespace IPhysics{

    class Firework : public Particle{
        private:
            unsigned m_type;
            real m_age;
        public:
        Firework();
        bool Integrate(real duration) override;
        
        void SetType(unsigned type);
        void SetAge(unsigned age);

        unsigned GetType();
        real GetAge();

    };
    struct Payload{
            unsigned type;
            unsigned count;
            Payload() : type(0), count(0) {}
            void Set(unsigned type, unsigned count){
                Payload::type = type;
                Payload::count = count;
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

        void Initialise(unsigned payloadCount);
        void SetParameters(unsigned type, real minAge, real maxAge, const Vector3& minVelocity, const Vector3& maxVelocity, real damping);
        void Create(Firework* firework, const Firework* parent = NULL) const;
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
            static int Update(real duration);

            static unsigned GetMaxFireworks();
            static Firework* GetFireworks();
            static FireworkRule* GetFireworkRules();

            static void Create(unsigned type, const Firework* parent);
            static void Create(unsigned type, unsigned number, const Firework* parent);
    };
}