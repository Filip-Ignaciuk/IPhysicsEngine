#include "barneshutgravity.hpp"

IPhysics::BarnesHutGravity::BarnesHutGravity(
    IPhysics::real _gravityConstant,
    IPhysics::real thresholdValue) :
    Gravity(_gravityConstant),
    thresholdValue(thresholdValue) {
    CreateTreeRoot();
}

void IPhysics::BarnesHutGravity::AddObject(IPhysics::Object* _object) {
    m_rigidBodies.emplace_back(_object->GetComponent<IPhysics::RigidBody>());
}

void IPhysics::BarnesHutGravity::RemoveObject(IPhysics::Object* _object) {
    std::erase(m_rigidBodies, _object->GetComponent<IPhysics::RigidBody>());
}

void IPhysics::BarnesHutGravity::UpdateForce(
    IPhysics::RigidBody* _rigidBody, IPhysics::real _duration) {
    if (totalProcessedParticles == m_rigidBodies.size()) {
        totalProcessedParticles = 0;
    }
    
    if (totalProcessedParticles == 0) {
        CreateTree();
    }

    _rigidBody->AddForce(TraverseNode(root, _rigidBody));
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
   }
   else {
        root->width = std::abs(furthestPoint.y);
   }
}

void IPhysics::BarnesHutGravity::AddObjectToNode(bhtn* _node, IPhysics::RigidBody* _rigidBody) {
    // Empty Leaf
    if (_node->IsExternalNode() && _node->rigidBody == nullptr) {
        _node->rigidBody = _rigidBody;
        _node->mass = _rigidBody->GetMass();
        _node->centreOfMass = _rigidBody->GetPosition();

        return;
    }
    // Internal Node
    // Update values along the way
    if (!_node->IsExternalNode()) {
        // Update centre of mass and total mass
        const IPhysics::real totalMass = _node->mass + _rigidBody->GetMass();
        IPhysics::Vector3 newCentreOfMass{};
        newCentreOfMass.x =
            (_node->centreOfMass.x * _node->mass
            + _rigidBody->GetPosition().x * _rigidBody->GetMass()) / totalMass;
        newCentreOfMass.y =
            (_node->centreOfMass.y * _node->mass
        + _rigidBody->GetPosition().y * _rigidBody->GetMass()) / totalMass;
        _node->mass = totalMass;
        _node->centreOfMass = newCentreOfMass;

        // Add object to correct quadrant.
        const bool isEast = _rigidBody->GetPosition().x >= _node->midPoint.x;
        const bool isSouth = _rigidBody->GetPosition().y <= _node->midPoint.y;
        if (isEast) {
            AddObjectToNode((isSouth ? _node->se : _node->ne), _rigidBody);
        }
        else {
            AddObjectToNode((isSouth ? _node->sw : _node->nw), _rigidBody);
        }

        return;
    }

    // Full Leaf
    // Create children nodes and disperse rigid bodies accordingly.

    // Creating children nodes.
    auto* nw = new bhtn();
    nw->width = _node->width / 2;
    nw->midPoint = IPhysics::Vector3{
        _node->midPoint.x - (_node->width / 2),
        _node->midPoint.y + (_node->width / 2),
        0
    };
    _node->nw = nw;

    auto* ne = new bhtn();
    ne->width = _node->width / 2;
    ne->midPoint = IPhysics::Vector3{
        _node->midPoint.x + (_node->width / 2),
        _node->midPoint.y + (_node->width / 2),
        0
    };
    _node->ne = ne;

    auto* sw = new bhtn();
    sw->width = _node->width / 2;
    sw->midPoint = IPhysics::Vector3{
        _node->midPoint.x - (_node->width / 2),
        _node->midPoint.y - (_node->width / 2),
        0
    };
    _node->sw = sw;

    auto* se = new bhtn();
    se->width = _node->width / 2;
    se->midPoint = IPhysics::Vector3{
        _node->midPoint.x + (_node->width / 2),
        _node->midPoint.y - (_node->width / 2),
        0
    };
    _node->se = se;

    // Update centre of mass and total mass
    const IPhysics::real totalMass = _node->mass + _rigidBody->GetMass();
    IPhysics::Vector3 newCentreOfMass{};
    newCentreOfMass.x =
        (_node->centreOfMass.x * _node->mass
        + _rigidBody->GetPosition().x * _rigidBody->GetMass()) / totalMass;
    newCentreOfMass.y =
        (_node->centreOfMass.y * _node->mass
    + _rigidBody->GetPosition().y * _rigidBody->GetMass()) / totalMass;
    _node->mass = totalMass;
    _node->centreOfMass = newCentreOfMass;

    // Add first object to correct quadrant.
    const bool isEastfirst = _rigidBody->GetPosition().x >= _node->midPoint.x;
    const bool isSouthfirst = _rigidBody->GetPosition().y <= _node->midPoint.y;
    if (isEastfirst) {
        AddObjectToNode((isSouthfirst ? _node->se : _node->ne), _rigidBody);
    }
    else {
        AddObjectToNode((isSouthfirst ? _node->sw : _node->nw), _rigidBody);
    }

    // Add second object to correct quadrant.
    const bool isEastSecond = _node->rigidBody->GetPosition().x >= _node->midPoint.x;
    const bool isSouthSecond = _node->rigidBody->GetPosition().y <= _node->midPoint.y;
    if (isEastSecond) {
        AddObjectToNode((isSouthSecond ? _node->se : _node->ne), _node->rigidBody);
    }
    else {
        AddObjectToNode((isSouthSecond ? _node->sw : _node->nw), _node->rigidBody);
    }

    _node->rigidBody = nullptr;
}

IPhysics::Vector3 IPhysics::BarnesHutGravity::CalculateGravityForce(
    IPhysics::real _mass1,
    const IPhysics::Vector3& _centreOfMass1,
    IPhysics::real _mass2,
    const IPhysics::Vector3& _centreOfMass2) const {

    const IPhysics::real totalMass = _mass1 * _mass2;
    const IPhysics::Vector3 distance = _centreOfMass1 - _centreOfMass2;
    const IPhysics::real distanceMagnitude = distance.Magnitude();

    const IPhysics::real forceMagnitude = -1 * m_gravityConstant
    * totalMass / (distanceMagnitude * distanceMagnitude * distanceMagnitude);
    return distance * forceMagnitude;
}

IPhysics::Vector3 IPhysics::BarnesHutGravity::TraverseNode(
    const bhtn *_node,
    const IPhysics::RigidBody* _rigidBody) {
    IPhysics::Vector3 totalForce{0, 0, 0};

    if (_node->IsExternalNode()) {
        if (_node->rigidBody == nullptr) {
            return totalForce;
        }
        if (_node->rigidBody == _rigidBody) {
            return totalForce;
        }
        totalForce += CalculateGravityForce(
            _rigidBody->GetMass(),
            _rigidBody->GetPosition(),
            _node->rigidBody->GetMass(),
            _node->rigidBody->GetPosition());
        }
    else {
        const IPhysics::Vector3 displacementBetweenMasses = _rigidBody->GetPosition()
        - _node->centreOfMass;
        IPhysics::real distanceBetweenMasses = displacementBetweenMasses.Magnitude();

        if ((_node->width / distanceBetweenMasses) < thresholdValue) {
            totalForce += CalculateGravityForce(
        _rigidBody->GetMass(),
        _rigidBody->GetPosition(),
        _node->mass,
        _node->centreOfMass);
        }
        else {
            totalForce += TraverseNode(_node->nw, _rigidBody);
            totalForce += TraverseNode(_node->ne, _rigidBody);
            totalForce += TraverseNode(_node->sw, _rigidBody);
            totalForce += TraverseNode(_node->se, _rigidBody);
        }
    }
    return totalForce;
}