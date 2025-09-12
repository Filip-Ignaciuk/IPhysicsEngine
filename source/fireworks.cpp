#include "fireworks.hpp"

IPhysicsEngine::FireworkRule::FireworkRule() : payloadCount(0), payloads(NULL){

}

void IPhysicsEngine::FireworkRule::Initialise(unsigned _payloadCount){
    FireworkRule::payloadCount = payloadCount;
    payloads = new Payload[payloadCount];
}

void IPhysicsEngine::FireworkRule::SetParameters(unsigned _type, real _minAge, real _maxAge, const Vector3& _minVelocity, const Vector3& _maxVelocity, real _damping){
    FireworkRule::type = _type;
    FireworkRule::minAge = _minAge;
    FireworkRule::maxAge = _maxAge;
    FireworkRule::minVelocity = _minVelocity;
    FireworkRule::maxVelocity = _maxVelocity;
    FireworkRule::damping = _damping;
}



void IPhysicsEngine::FireworkRule::Create(Firework* _firework, const Firework* _parent) const{
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
    velocity += RandomVector3(minVelocity, maxVelocity);
    _firework->SetVelocity(velocity);

    _firework->SetMass(1);
    _firework->SetDamping(damping);
    _firework->ClearAccumulator();
}

IPhysicsEngine::Firework::Firework() : Particle(){

}

bool IPhysicsEngine::Firework::Integrate(real _duration){
    Particle::Integrate(_duration);
    m_age -= _duration;
    return (m_age < 0) || (m_position.GetY() < 0);
}

void IPhysicsEngine::Firework::SetType(unsigned _type){
    m_type = _type;
}   

void IPhysicsEngine::Firework::SetAge(unsigned _age){
    m_age = _age;
}

unsigned IPhysicsEngine::Firework::GetType(){
    return m_type;
}

IPhysicsEngine::real IPhysicsEngine::Firework::GetAge(){
    return m_age;
}

void IPhysicsEngine::FireworkManager::Create(unsigned _type, const Firework* _parent){
    FireworkRule* rule = fireworkRules + (_type - 1);
    IPhysicsEngine::Firework* firework = new IPhysicsEngine::Firework();
    fireworks[nextFirework] = *firework;
    rule->Create(&fireworks[nextFirework], _parent);

    nextFirework = (nextFirework + 1) % maxFireworks;
}

void IPhysicsEngine::FireworkManager::Create(unsigned _type, unsigned _number, const Firework* _parent)
{
    for (unsigned i = 0; i < _number; i++)
    {
        Create(_type, _parent);
    }
}

void IPhysicsEngine::FireworkManager::Initialise(){

    for (Firework* firework = fireworks; firework < fireworks + maxFireworks; firework++){
        firework = new Firework();
        firework->SetType(0);
    }
    


    fireworkRules[0].Initialise(2);
    fireworkRules[0].SetParameters(
        1, // type
        0.5f, 1.4f, // age range
        IPhysicsEngine::Vector3(-5, 25, -5), // min velocity
        IPhysicsEngine::Vector3(5, 28, 5), // max velocity
        0.1 // damping
        );
    fireworkRules[0].payloads[0].Set(3, 5);
    fireworkRules[0].payloads[1].Set(5, 5);

    fireworkRules[1].Initialise(1);
    fireworkRules[1].SetParameters(
        2, // type
        0.5f, 1.0f, // age range
        IPhysicsEngine::Vector3(-5, 10, -5), // min velocity
        IPhysicsEngine::Vector3(5, 20, 5), // max velocity
        0.8 // damping
        );
    fireworkRules[1].payloads[0].Set(4, 2);

    fireworkRules[2].Initialise(0);
    fireworkRules[2].SetParameters(
        3, // type
        0.5f, 1.5f, // age range
        IPhysicsEngine::Vector3(-5, -5, -5), // min velocity
        IPhysicsEngine::Vector3(5, 5, 5), // max velocity
        0.1 // damping
        );

    fireworkRules[3].Initialise(0);
    fireworkRules[3].SetParameters(
        4, // type
        0.25f, 0.5f, // age range
        IPhysicsEngine::Vector3(-20, 5, -5), // min velocity
        IPhysicsEngine::Vector3(20, 5, 5), // max velocity
        0.2 // damping
        );

    fireworkRules[4].Initialise(1);
    fireworkRules[4].SetParameters(
        5, // type
        0.5f, 1.0f, // age range
        IPhysicsEngine::Vector3(-20, 2, -5), // min velocity
        IPhysicsEngine::Vector3(20, 18, 5), // max velocity
        0.01 // damping
        );
    fireworkRules[4].payloads[0].Set(3, 5);

    fireworkRules[5].Initialise(0);
    fireworkRules[5].SetParameters(
        6, // type
        3, 5, // age range
        IPhysicsEngine::Vector3(-5, 5, -5), // min velocity
        IPhysicsEngine::Vector3(5, 10, 5), // max velocity
        0.95 // damping
        );

    fireworkRules[6].Initialise(1);
    fireworkRules[6].SetParameters(
        7, // type
        4, 5, // age range
        IPhysicsEngine::Vector3(-5, 50, -5), // min velocity
        IPhysicsEngine::Vector3(5, 60, 5), // max velocity
        0.1 // damping
        );
    fireworkRules[6].payloads[0].Set(8, 10);

    fireworkRules[7].Initialise(0);
    fireworkRules[7].SetParameters(
        8, // type
        5.5f, 6.5f, // age range
        IPhysicsEngine::Vector3(-10, -10, -10), // min velocity
        IPhysicsEngine::Vector3(10, 10, 10), // max velocity
        0.1 // damping
        );

    fireworkRules[8].Initialise(0);
    fireworkRules[8].SetParameters(
        9, // type
        3, 5, // age range
        IPhysicsEngine::Vector3(-15, 10, -5), // min velocity
        IPhysicsEngine::Vector3(15, 15, 5), // max velocity
        0.95 // damping
        );
}


void IPhysicsEngine::FireworkManager::Update(real _duration){
    for (Firework* firework = fireworks; firework < fireworks + maxFireworks; firework++){
        if (firework->GetType() > 0){
            if (firework->Integrate(_duration)){
                FireworkRule* rule = fireworkRules + (firework->GetType()-1);

                firework->SetType(0);

                for (unsigned i = 0; i < rule->payloadCount; i++)
                {
                    Payload* payload = rule->payloads + i;
                    Create(payload->type, payload->count, firework);
                }
                
            }
        }
    }
}

unsigned IPhysicsEngine::FireworkManager::GetMaxFireworks(){
    return maxFireworks;
}

IPhysicsEngine::Firework* IPhysicsEngine::FireworkManager::GetFireworks(){
    return fireworks;
}