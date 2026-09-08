#include "barneshutgravity.hpp"

IPhysics::BarnesHutGravity::BarnesHutGravity(IPhysics::real gravityConstant,
                                             IPhysics::real thresholdValue)
    : Gravity(gravityConstant), thresholdValue(thresholdValue) {
  CreateTreeRoot();
}

void IPhysics::BarnesHutGravity::AddObject(IPhysics::Object* object) {
  m_rigidBodies.emplace_back(object->GetComponent<IPhysics::RigidBody>());
}

void IPhysics::BarnesHutGravity::RemoveObject(IPhysics::Object* object) {
  std::erase(m_rigidBodies, object->GetComponent<IPhysics::RigidBody>());
}

void IPhysics::BarnesHutGravity::UpdateForce(IPhysics::RigidBody* rigidBody,
                                             IPhysics::real duration) {
  if (totalProcessedParticles == m_rigidBodies.size()) {
    totalProcessedParticles = 0;
  }

  if (totalProcessedParticles == 0) {
    CreateTree();
  }

  rigidBody->AddForce(TraverseNode(root, rigidBody));
  ++totalProcessedParticles;
}

void IPhysics::BarnesHutGravity::CreateTree() {
  CreateTreeRoot();
  for (IPhysics::RigidBody* rigidbody : m_rigidBodies) {
    AddObjectToNode(root, rigidbody);
  }
}

void IPhysics::BarnesHutGravity::CreateTreeRoot() {
  delete root;
  root = new bhtn();
  root->midPoint = {0, 0, 0};

  IPhysics::real largestDistance = 0;
  IPhysics::Vector3 furthestPoint{};

  for (const IPhysics::RigidBody* rigidbody : m_rigidBodies) {
    if (rigidbody->GetPosition().Magnitude() > largestDistance) {
      largestDistance = rigidbody->GetPosition().Magnitude();
      furthestPoint = rigidbody->GetPosition();
    }
  }
  if (std::abs(furthestPoint.x) > std::abs(furthestPoint.y)) {
    root->width = std::abs(furthestPoint.x);
  } else {
    root->width = std::abs(furthestPoint.y);
  }
}

void IPhysics::BarnesHutGravity::AddObjectToNode(
    bhtn* node, IPhysics::RigidBody* rigidBody) {
  // Empty Leaf
  if (node->IsExternalNode() && node->rigidBody == nullptr) {
    node->rigidBody = rigidBody;
    node->mass = rigidBody->GetMass();
    node->centreOfMass = rigidBody->GetPosition();

    return;
  }
  // Internal Node
  // Update values along the way
  if (!node->IsExternalNode()) {
    // Update centre of mass and total mass
    const IPhysics::real totalMass = node->mass + rigidBody->GetMass();
    IPhysics::Vector3 newCentreOfMass{};
    newCentreOfMass.x = (node->centreOfMass.x * node->mass +
                         rigidBody->GetPosition().x * rigidBody->GetMass()) /
                        totalMass;
    newCentreOfMass.y = (node->centreOfMass.y * node->mass +
                         rigidBody->GetPosition().y * rigidBody->GetMass()) /
                        totalMass;
    node->mass = totalMass;
    node->centreOfMass = newCentreOfMass;

    // Add object to correct quadrant.
    const bool isEast = rigidBody->GetPosition().x >= node->midPoint.x;
    const bool isSouth = rigidBody->GetPosition().y <= node->midPoint.y;
    if (isEast) {
      AddObjectToNode((isSouth ? node->se : node->ne), rigidBody);
    } else {
      AddObjectToNode((isSouth ? node->sw : node->nw), rigidBody);
    }

    return;
  }

  // Full Leaf
  // Create children nodes and disperse rigid bodies accordingly.

  // Creating children nodes.
  auto* nw = new bhtn();
  nw->width = node->width / 2;
  nw->midPoint = IPhysics::Vector3{node->midPoint.x - (node->width / 2),
                                   node->midPoint.y + (node->width / 2), 0};
  node->nw = nw;

  auto* ne = new bhtn();
  ne->width = node->width / 2;
  ne->midPoint = IPhysics::Vector3{node->midPoint.x + (node->width / 2),
                                   node->midPoint.y + (node->width / 2), 0};
  node->ne = ne;

  auto* sw = new bhtn();
  sw->width = node->width / 2;
  sw->midPoint = IPhysics::Vector3{node->midPoint.x - (node->width / 2),
                                   node->midPoint.y - (node->width / 2), 0};
  node->sw = sw;

  auto* se = new bhtn();
  se->width = node->width / 2;
  se->midPoint = IPhysics::Vector3{node->midPoint.x + (node->width / 2),
                                   node->midPoint.y - (node->width / 2), 0};
  node->se = se;

  // Update centre of mass and total mass
  const IPhysics::real totalMass = node->mass + rigidBody->GetMass();
  IPhysics::Vector3 newCentreOfMass{};
  newCentreOfMass.x = (node->centreOfMass.x * node->mass +
                       rigidBody->GetPosition().x * rigidBody->GetMass()) /
                      totalMass;
  newCentreOfMass.y = (node->centreOfMass.y * node->mass +
                       rigidBody->GetPosition().y * rigidBody->GetMass()) /
                      totalMass;
  node->mass = totalMass;
  node->centreOfMass = newCentreOfMass;

  // Add first object to correct quadrant.
  const bool isEastfirst = rigidBody->GetPosition().x >= node->midPoint.x;
  const bool isSouthfirst = rigidBody->GetPosition().y <= node->midPoint.y;
  if (isEastfirst) {
    AddObjectToNode((isSouthfirst ? node->se : node->ne), rigidBody);
  } else {
    AddObjectToNode((isSouthfirst ? node->sw : node->nw), rigidBody);
  }

  // Add second object to correct quadrant.
  const bool isEastSecond =
      node->rigidBody->GetPosition().x >= node->midPoint.x;
  const bool isSouthSecond =
      node->rigidBody->GetPosition().y <= node->midPoint.y;
  if (isEastSecond) {
    AddObjectToNode((isSouthSecond ? node->se : node->ne), node->rigidBody);
  } else {
    AddObjectToNode((isSouthSecond ? node->sw : node->nw), node->rigidBody);
  }

  node->rigidBody = nullptr;
}

IPhysics::Vector3 IPhysics::BarnesHutGravity::CalculateGravityForce(
    IPhysics::real mass1, const IPhysics::Vector3& centreOfMass1,
    IPhysics::real mass2, const IPhysics::Vector3& centreOfMass2) const {
  const IPhysics::real totalMass = mass1 * mass2;
  const IPhysics::Vector3 distance = centreOfMass1 - centreOfMass2;
  const IPhysics::real distanceMagnitude = distance.Magnitude();

  const IPhysics::real forceMagnitude =
      -1 * m_gravityConstant * totalMass /
      (distanceMagnitude * distanceMagnitude * distanceMagnitude);
  return distance * forceMagnitude;
}

IPhysics::Vector3 IPhysics::BarnesHutGravity::TraverseNode(
    const bhtn* node, const IPhysics::RigidBody* rigidBody) {
  IPhysics::Vector3 totalForce{0, 0, 0};

  if (node->IsExternalNode()) {
    if (node->rigidBody == nullptr) {
      return totalForce;
    }
    if (node->rigidBody == rigidBody) {
      return totalForce;
    }
    totalForce += CalculateGravityForce(
        rigidBody->GetMass(), rigidBody->GetPosition(),
        node->rigidBody->GetMass(), node->rigidBody->GetPosition());
  } else {
    const IPhysics::Vector3 displacementBetweenMasses =
        rigidBody->GetPosition() - node->centreOfMass;
    IPhysics::real distanceBetweenMasses =
        displacementBetweenMasses.Magnitude();

    if ((node->width / distanceBetweenMasses) < thresholdValue) {
      totalForce +=
          CalculateGravityForce(rigidBody->GetMass(), rigidBody->GetPosition(),
                                node->mass, node->centreOfMass);
    } else {
      totalForce += TraverseNode(node->nw, rigidBody);
      totalForce += TraverseNode(node->ne, rigidBody);
      totalForce += TraverseNode(node->sw, rigidBody);
      totalForce += TraverseNode(node->se, rigidBody);
    }
  }
  return totalForce;
}