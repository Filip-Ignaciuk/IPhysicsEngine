#include "fireworks.hpp"

IPhysics::FireworkRule::FireworkRule()
    : type(0),
      minAge(0),
      maxAge(0),
      minVelocity(Origin),
      maxVelocity(Origin),
      damping(0),
      payloadCount(0),
      payloads(nullptr) {}

void IPhysics::FireworkRule::Initialise(unsigned payloadCount) {
  FireworkRule::payloadCount = payloadCount;
  payloads = new Payload[payloadCount];
  for (unsigned i = 0; i < payloadCount; i++) {
    payloads[i] = Payload();
  }
}

void IPhysics::FireworkRule::SetParameters(unsigned type, real minAge,
                                           real maxAge,
                                           const Vector3& minVelocity,
                                           const Vector3& maxVelocity,
                                           real damping) {
  FireworkRule::type = type;
  FireworkRule::minAge = minAge;
  FireworkRule::maxAge = maxAge;
  FireworkRule::minVelocity = minVelocity;
  FireworkRule::maxVelocity = maxVelocity;
  FireworkRule::damping = damping;
}

void IPhysics::FireworkRule::Create(Firework* firework,
                                    const Firework* parent) const {
  firework->SetType(type);
  firework->SetAge(RandomReal(minAge, maxAge));

  Vector3 velocity;
  if (parent) {
    firework->SetPosition(parent->GetPosition());
    velocity += parent->GetVelocity();
  } else {
    int positionx = RandomInt(0, 3) - 1;
    Vector3 start(positionx * 5.0f, 0, 0);
    firework->SetPosition(start);
  }
  velocity += RandomVector3(minVelocity, maxVelocity);
  firework->SetVelocity(velocity);

  firework->SetMass(1);
  firework->SetDamping(damping);
  firework->ClearAccumulator();
}

IPhysics::Firework::Firework() : m_age(0), m_type(0), Particle() {}

bool IPhysics::Firework::Integrate(real duration) {
  Particle::Integrate(duration);
  m_age -= duration;
  return (m_age < 0) || (m_position.GetY() < 0);
}

void IPhysics::Firework::SetType(unsigned type) { m_type = type; }

void IPhysics::Firework::SetAge(unsigned age) { m_age = age; }

unsigned IPhysics::Firework::GetType() { return m_type; }

IPhysics::real IPhysics::Firework::GetAge() { return m_age; }

void IPhysics::FireworkManager::Create(unsigned type, const Firework* parent) {
  FireworkRule* rule = fireworkRules + (type - 1);

  rule->Create(&fireworks[nextFirework], parent);

  nextFirework = (nextFirework + 1) % maxFireworks;
}

void IPhysics::FireworkManager::Create(unsigned type, unsigned number,
                                       const Firework* parent) {
  for (unsigned i = 0; i < number; i++) {
    Create(type, parent);
  }
}

void IPhysics::FireworkManager::Initialise() {
  for (Firework* firework = fireworks; firework < fireworks + maxFireworks;
       firework++) {
    firework = new Firework();
    firework->SetType(0);
  }

  fireworkRules[0].Initialise(2);
  fireworkRules[0].SetParameters(1,                              // type
                                 0.5f, 1.4f,                     // age range
                                 IPhysics::Vector3(-5, 25, -5),  // min velocity
                                 IPhysics::Vector3(5, 28, 5),    // max velocity
                                 0.1                             // damping
  );
  fireworkRules[0].payloads[0].Set(3, 5);
  fireworkRules[0].payloads[1].Set(5, 5);

  fireworkRules[1].Initialise(1);
  fireworkRules[1].SetParameters(2,                              // type
                                 0.5f, 1.0f,                     // age range
                                 IPhysics::Vector3(-5, 10, -5),  // min velocity
                                 IPhysics::Vector3(5, 20, 5),    // max velocity
                                 0.8                             // damping
  );
  fireworkRules[1].payloads[0].Set(4, 2);

  fireworkRules[2].Initialise(0);
  fireworkRules[2].SetParameters(3,                              // type
                                 1.5f, 2.5f,                     // age range
                                 IPhysics::Vector3(-5, -5, -5),  // min velocity
                                 IPhysics::Vector3(5, 5, 5),     // max velocity
                                 0.1                             // damping
  );

  fireworkRules[3].Initialise(0);
  fireworkRules[3].SetParameters(4,                              // type
                                 0.25f, 0.5f,                    // age range
                                 IPhysics::Vector3(-20, 5, -5),  // min velocity
                                 IPhysics::Vector3(20, 5, 5),    // max velocity
                                 0.2                             // damping
  );

  fireworkRules[4].Initialise(1);
  fireworkRules[4].SetParameters(
      5,                               // type
      1.5f, 1.75f,                     // age range
      IPhysics::Vector3(-20, 2, -5),   // min velocity
      IPhysics::Vector3(20, 230, 50),  // max velocity
      0.01                             // damping
  );
  fireworkRules[4].payloads[0].Set(3, 5);

  fireworkRules[5].Initialise(0);
  fireworkRules[5].SetParameters(6,                             // type
                                 3, 5,                          // age range
                                 IPhysics::Vector3(-5, 5, -5),  // min velocity
                                 IPhysics::Vector3(5, 10, 5),   // max velocity
                                 0.95                           // damping
  );

  fireworkRules[6].Initialise(1);
  fireworkRules[6].SetParameters(7,                              // type
                                 4, 5,                           // age range
                                 IPhysics::Vector3(-5, 50, -5),  // min velocity
                                 IPhysics::Vector3(5, 60, 5),    // max velocity
                                 0.01                            // damping
  );
  fireworkRules[6].payloads[0].Set(8, 10);

  fireworkRules[7].Initialise(0);
  fireworkRules[7].SetParameters(8,                              // type
                                 0.25f, 0.5f,                    // age range
                                 IPhysics::Vector3(-1, -1, -1),  // min velocity
                                 IPhysics::Vector3(1, 1, 1),     // max velocity
                                 0.01                            // damping
  );

  fireworkRules[8].Initialise(0);
  fireworkRules[8].SetParameters(
      9,                               // type
      3, 5,                            // age range
      IPhysics::Vector3(-15, 10, -5),  // min velocity
      IPhysics::Vector3(15, 15, 5),    // max velocity
      0.95                             // damping
  );
}

int IPhysics::FireworkManager::Update(real duration) {
  int pay = 0;
  for (Firework* firework = fireworks; firework < fireworks + maxFireworks;
       firework++) {
    if (firework->GetType() > 0) {
      if (firework->Integrate(duration)) {
        FireworkRule* rule = fireworkRules + (firework->GetType() - 1);

        firework->SetType(0);
        for (unsigned i = 0; i < rule->payloadCount; i++) {
          Payload* payload = rule->payloads + i;
          Create(payload->type, payload->count, firework);
        }
      }
    }
  }
  return pay;
}

unsigned IPhysics::FireworkManager::GetMaxFireworks() { return maxFireworks; }

IPhysics::Firework* IPhysics::FireworkManager::GetFireworks() {
  return fireworks;
}

IPhysics::FireworkRule* IPhysics::FireworkManager::GetFireworkRules() {
  return fireworkRules;
}