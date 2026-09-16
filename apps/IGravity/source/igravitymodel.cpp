#include "igravitymodel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
#include <vector>


#include "core.hpp"
#include "gravity.hpp"
#include "cudagravity.cuh"

#include "barneshutgravity.hpp"

#include "object.hpp"
#include "precision.hpp"
#include "rigidbody.hpp"

/*
 * IGravityModel
 */

// Constructors
IGravityModel::IGravityModel(IPhysics::real timeStep) : m_timeStep(timeStep) {
  m_gravityForceGenerator = 
    std::make_shared<IPhysics::Gravity>(6.674 * pow(10, -11));
  
  // Start with an inital amount of particles
  UpdateNumberOfParticles(1000);
}

// Mutators
void IGravityModel::UpdateSimulation() {
  m_world.StartFrame();
  if (m_world.GetPhysicsState()) {
    auto start = std::chrono::high_resolution_clock::now();
    m_world.RunPhysics(m_timeStep);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  } else {
  }
}

// Can both increase and decrease the number of particles based on the count
// provided.
void IGravityModel::UpdateNumberOfParticles(int count) {
  int boundedCount = 0;
  // Check if count is larger than maximum or smaller than minimum.
  if (count > MAXIMUM_PARTICLE_COUNT - m_world.GetNumberOfObjects()) {
    // Bound to maximum allowed increase, AKA add maximum amount of particles.
    boundedCount = MAXIMUM_PARTICLE_COUNT - m_world.GetNumberOfObjects();
  } else if (count < -m_world.GetNumberOfObjects()) {
    // Bound to minimum allowed decrease, AKA delete all particles.
    boundedCount = -m_world.GetNumberOfObjects();
  } else {
    boundedCount = count;
  }

  if (boundedCount < 0) {
    boundedCount = -boundedCount;
    for (int i = 0; i < boundedCount; ++i) {
      IPhysics::Object* currentObject = m_world.GetLastObject();
      
      // Remove object from vector in force generator
      m_gravityForceGenerator->RemoveObject(currentObject);
      m_world.RemoveForceRegistration(currentObject);

      m_world.RemoveLastObject();
    }
  } 
  else {
    bool needsCudaAlgorithmOnFirstObject = false;

    if(m_world.GetNumberOfObjects() == 0 && IsUsingCUDAAlgorithm()){
      needsCudaAlgorithmOnFirstObject = true;
    }

    for (int i = 0; i < boundedCount; ++i) {
      // Creating object
      auto* object = new IPhysics::Object();
      auto* rigid_body = object->AddComponent<IPhysics::RigidBody>();
      
      // Setting random location
      IPhysics::Vector3 random_position = RandomGalaxyPosition();

      rigid_body->SetPosition(random_position);
      rigid_body->SetInverseMass(1.0 / (1.0 * pow(10, 12)));
      rigid_body->SetLinearDamping(1.0f);

      // Adding it to world
      m_world.AddObject(object);
      m_gravityForceGenerator->AddObject(object);
      if(!IsUsingCUDAAlgorithm()){
        m_world.AddForceRegistration(object, m_gravityForceGenerator);
      
      }
    }

    // Setting correct velocities.
    CalculateParticleVelocities();

    if(needsCudaAlgorithmOnFirstObject){
      m_world.AddForceRegistration(
        m_world.GetObjects()[0], 
        m_gravityForceGenerator);
    }

  }
}

void IGravityModel::UpdateAlgorithmType(GravityAlgorithm gravityAlgorithm){
  if(gravityAlgorithm == m_gravityAlgorithm){
    return;
  }

  std::shared_ptr<IPhysics::Gravity> newGravityForceGenerator;

  if(gravityAlgorithm == GravityAlgorithm::Naive){
    newGravityForceGenerator =
      std::make_shared<IPhysics::Gravity>(GRAVITY_CONSTANT);
  }
  else if(gravityAlgorithm == GravityAlgorithm::NaiveCuda){
    #ifdef IPHYSICS_USE_CUDA
    newGravityForceGenerator =
      std::make_shared<IPhysics::CudaGravity>(GRAVITY_CONSTANT);
    #endif
  }
  else if(gravityAlgorithm == GravityAlgorithm::BarnesHut){
    newGravityForceGenerator =
      std::make_shared<IPhysics::BarnesHutGravity>(
        GRAVITY_CONSTANT,
        0.5);
  }
  else{
    // Somehow invalid
    return;
  }

  m_gravityAlgorithm = gravityAlgorithm;
  

  // Delete all objects in force generator and delete force registration.
  // Add the object to the new force generator and register it in the registry.
  // NOTE: CUDA algorithms only apply to one object so has a special case 
  // of application
  for(IPhysics::Object* object : m_world.GetObjects()){
    m_gravityForceGenerator->RemoveObject(object);
    // If its CUDA this will still work as it would just do nothing if no
    // Registration is present.
    m_world.RemoveForceRegistration(object);

    newGravityForceGenerator->AddObject(object);

    if(!IsUsingCUDAAlgorithm()){
      m_world.AddForceRegistration(object, newGravityForceGenerator);
    }
  }

  if(IsUsingCUDAAlgorithm() && m_world.GetObjects().size() != 0){
    IPhysics::Object* object = m_world.GetObjects()[0];
    m_world.AddForceRegistration(object, newGravityForceGenerator);
  }

  m_gravityForceGenerator = newGravityForceGenerator;
}

void IGravityModel::SetSimulationPause(bool wantsPaused) {
  m_world.SetPhysicsState(wantsPaused);
}

void IGravityModel::Restart(){
  const int previousSize = m_world.GetNumberOfObjects();
  UpdateNumberOfParticles(-previousSize);
  UpdateNumberOfParticles(previousSize);
}

// Queries
const std::vector<IPhysics::Object*>& IGravityModel::GetParticles() {
  return m_world.GetObjects();
}

IPhysics::Vector3 IGravityModel::RandomGalaxyPosition() {
  IPhysics::real scaleRadius = 800.0;

  IPhysics::real randomNumber1 = IPhysics::RandomStore::RandomReal(0, 1);
  IPhysics::real randomNumber2 = IPhysics::RandomStore::RandomReal(0, 1);

  IPhysics::real r = -scaleRadius * log(1 - randomNumber1);

  IPhysics::real theta = 2 * std::numbers::pi * randomNumber2;

  IPhysics::real xPosition = 0 + r * RealCos(theta);
  IPhysics::real yPosition = 0 + r * RealSin(theta);
  return {xPosition, yPosition, 0};
}

void IGravityModel::CalculateParticleVelocities(){
  const IPhysics::Matrix3 rotationMatrix{
    0, -1, 0,
    1, 0, 0,
    0, 0, 1
  };
  for(int i = 0; i < m_world.GetNumberOfObjects(); ++i){
    IPhysics::RigidBody* rigidBody = m_world.GetObjects()[i]->
    GetComponent<IPhysics::RigidBody>();

    // We assume the galaxy is 0, 0 so far.
    IPhysics::Vector3 displacementFromCentre = 
    rigidBody->GetPosition() - IPhysics::Origin;
    IPhysics::real distanceFromCentre = displacementFromCentre.Magnitude();

    if(distanceFromCentre == 0){
      continue;
    }
    IPhysics::real speed = 3 * distanceFromCentre / 
    sqrt(distanceFromCentre * distanceFromCentre + 1000);



    IPhysics::Vector3 velocity = rotationMatrix * (rigidBody->GetPosition() - IPhysics::Origin);
    velocity.Normalise();
    velocity = velocity * speed;
    rigidBody->AddVelocity(velocity);
  }
}

bool IGravityModel::IsSimulationPaused() const { 
  return m_world.GetPhysicsState(); 
}

const int IGravityModel::GetMaximumParticleCount() const{
  return MAXIMUM_PARTICLE_COUNT;
}

bool IGravityModel::IsUsingCUDAAlgorithm() const{
  return m_gravityAlgorithm == GravityAlgorithm::NaiveCuda;
}