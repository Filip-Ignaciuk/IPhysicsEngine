#include "contacts.hpp"

void IPhysicsEngine::Contact::SetBodyData(RigidBody* _one, RigidBody* _two, real _friction, real _restitution){
    body[0] = _one;
    body[1] = _two;
    friction = _friction;
    restitution = _restitution;
}

void IPhysicsEngine::Contact::MatchAwakeState(){
    // Collisions with the world do not wake up bodies.
    if(!body[1]){
        return;
    }

    bool firstBodyAwake = body[0]->GetIsAwake();
    bool secondBodyAwake = body[1]->GetIsAwake();

    // Wake up only the sleeping one.
    if(firstBodyAwake ^ secondBodyAwake){
        if(firstBodyAwake){
            body[1]->SetIsAwake(true);
        }
        else{
            body[0]->SetIsAwake(true);
        }
    }
}

void IPhysicsEngine::Contact::CreateContactBasis(){
    Vector3 contactTangent[2];

    // Firstly we check if the Z or Y axis is nearer.
    if(RealAbs(contactNormal.x) > RealAbs(contactNormal.y)){
        const real scale = (real)1.0f/RealSqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

        contactTangent[0].x = contactNormal.z * scale;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x * scale;

        contactTangent[1].x = contactNormal.y * contactTangent[0].x;
        contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
    }
    else{
        const real scale = (real)1.0f/RealSqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z * scale;
        contactTangent[0].z = contactNormal.y * scale;

        contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = contactNormal.x * contactTangent[0].y;
    }

    contactToWorld.SetComponents(contactNormal, contactTangent[0], contactTangent[1]);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Contact::CalculateFrictionlessImpulse(Matrix3* _inverseInertiaTensor){
    // Obtain the torque axis, the direction the object would start rotating if you applied a unit of impulse
    // at the contact point along the normal.
    Vector3 deltaVelocityWorld = relativeContactPosition[0] % contactNormal;
    deltaVelocityWorld = _inverseInertiaTensor[0].Transform(deltaVelocityWorld);
    deltaVelocityWorld = deltaVelocityWorld % relativeContactPosition[0];
    real deltaVelocity = deltaVelocityWorld * contactNormal;
    deltaVelocity += body[0]->GetInverseMass();

    if(body[1]){
        Vector3 deltaVelocityWorldSecond = relativeContactPosition[1] % contactNormal;
        deltaVelocityWorldSecond = _inverseInertiaTensor[1].Transform(deltaVelocityWorldSecond);
        deltaVelocityWorldSecond = deltaVelocityWorldSecond % relativeContactPosition[1];
        deltaVelocity += deltaVelocityWorldSecond * contactNormal;
        deltaVelocity += body[1]->GetInverseMass();
    }

    Vector3 impulseContact;
    impulseContact.x = desiredDeltaVelocity / deltaVelocity;
    impulseContact.y = 0;
    impulseContact.z = 0;
    return impulseContact;
}

void IPhysicsEngine::Contact::ApplyVelocityChange(Vector3 _velocityChange[2], Vector3 _rotationChange[2]){
    // Get the inverse mass and inverse inertia tensor, both in world coordinates.
    Matrix3 inverseInertiaTensor[2];
    inverseInertiaTensor[0] = body[0]->GetInverseInertiaTensorWorld();
    if(body[1]){
        inverseInertiaTensor[1] = body[1]->GetInverseInertiaTensorWorld();
    }
    Vector3 impulseContact;
    if(friction == (real)0.0){
        impulseContact = CalculateFrictionlessImpulse(inverseInertiaTensor);
    }

    Vector3 impulse = contactToWorld.Transform(impulseContact);
    Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
    _rotationChange[0] = inverseInertiaTensor[0].Transform(impulsiveTorque);
    _velocityChange[0].Clear();
    _velocityChange[0].AddScaledVector(impulse, body[0]->GetInverseMass());

    body[0]->AddVelocity(_velocityChange[0]);
    body[0]->AddRotation(_rotationChange[0]);

    if(body[1]){
        Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
        _rotationChange[1] = inverseInertiaTensor[1].Transform(impulsiveTorque);
        _velocityChange[1].Clear();
        _velocityChange[1].AddScaledVector(impulse, -body[1]->GetInverseMass());

        body[1]->AddVelocity(_velocityChange[1]);
        body[1]->AddRotation(_rotationChange[1]);

    }
}


void IPhysicsEngine::Contact::ApplyPositionChange(Vector3 _linearChange[2], Vector3 _angularChange[2], real _penetration){
    const real angularLimit = (real)0.2f;
    real angularMove[2];
    real linearMove[2];

    real totalInertia = 0;
    real linearInertia[2];
    real angularInertia[2];

    for(unsigned i = 0; i < 2; ++i){
        if(body[1]){
            Matrix3 inverseInertiaTensor;
            inverseInertiaTensor = body[i]->GetInverseInertiaTensorWorld();
            Vector3 angularInertiaWorld = relativeContactPosition[i] % contactNormal;
            angularInertiaWorld = inverseInertiaTensor.Transform(angularInertiaWorld);
            angularInertiaWorld = angularInertiaWorld % relativeContactPosition[i];
            angularInertia[i] = angularInertiaWorld * contactNormal;

            linearInertia[i] = body[i]->GetInverseMass();

            totalInertia += linearInertia[i] + angularInertia[i];
        }
    }
}

void IPhysicsEngine::Contact::CalculateInternals(real _duration){
    if(!body[0]){
        SwapBodies();
    }

    CalculateContactBasis();

    relativeContactPosition[0] = contactPoint - body[0]->GetPosition();
    if(body[1]){
        relativeContactPosition[1] = contactPoint - body[1]->GetPosition();
    }

    contactVelocity = CalculateLocalVelocity(0, _duration);
    if (body[1]){
        contactVelocity -= CalculateLocalVelocity(1, _duration);
    }

    CalculateDesiredDeltaVelocity(_duration);
}

IPhysicsEngine::Vector3 IPhysicsEngine::Contact::CalculateLocalVelocity(unsigned _bodyIndex, real _duration){
    RigidBody* thisBody = body[_bodyIndex];

    Vector3 velocity = thisBody->GetRotation() % relativeContactPosition[_bodyIndex];
    velocity += thisBody->GetVelocity();

    Vector3 localContactVelocity = contactToWorld.TransformTranspose(velocity);

    Vector3 actualVelocity = thisBody->GetLastFrameAcceleration() * _duration;

    return contactVelocity;


}

void IPhysicsEngine::Contact::CalculateDesiredDeltaVelocity(real _duration){
    const static real velocityLimit = (real)0.25f;

    real velocityFromAcceleration = 0;

    if(body[0]->GetIsAwake()){
        velocityFromAcceleration += body[0]->GetLastFrameAcceleration() * _duration * contactNormal;
    }
    if(body[1] && body[1]->GetIsAwake()){
        velocityFromAcceleration -= body[1]->GetLastFrameAcceleration() * _duration * contactNormal;
    }

    // If the velocity is very low, limit the restitution.
    real thisRestitution = restitution;
    if(RealAbs(contactVelocity.x) < velocityLimit){
        thisRestitution = (real)0.0f;
    }

    desiredDeltaVelocity = -contactVelocity.x -thisRestitution * (contactVelocity.x - velocityFromAcceleration);

}

void IPhysicsEngine::Contact::SwapBodies(){
    contactNormal *= -1;
    RigidBody* temporary = body[0];
    body[0] = body[1];
    body[1] = temporary;
}

void IPhysicsEngine::Contact::CalculateContactBasis(){
    Vector3 contactTangent[2];

    // Check whether the z axis is nearer to the x or y axis.
    if(RealAbs(contactNormal.x) > RealAbs(contactNormal.y)){
        // Scale factor to ensure results are normalised.
        const real scaleFactor = (real)1.0f / RealSqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

        // The new X axis is at right angles to the world Y-axis.
        contactTangent[0].x = contactNormal.z * scaleFactor;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x * scaleFactor;

        // The new Y axis is at right angles to the new x and z axes.
        contactTangent[1].x = contactNormal.y * contactTangent[0].x;
        contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
    }
    else
    {
        // Scaling factor to ensure the results are normalised
        const real scaleFactor = (real)1.0f / RealSqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

        // The new X-axis is at right angles to the world X-axis
        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z * scaleFactor;
        contactTangent[0].z = contactNormal.y * scaleFactor;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = contactNormal.x * contactTangent[0].y;
    }
    // Make a matrix from the three vectors.
    contactToWorld.SetComponents(contactNormal, contactTangent[0], contactTangent[1]);
}

void IPhysicsEngine::ContactResolver::ResolveContacts(Contact* _contactArray, unsigned _numberOfContacts, real _duration){
    if (_numberOfContacts == 0){
        return;
    }

    PrepareContacts(_contactArray, _numberOfContacts, _duration);

    AdjustPositions(_contactArray, _numberOfContacts, _duration);

    AdjustVelocities(_contactArray, _numberOfContacts, _duration);
}

void IPhysicsEngine::ContactResolver::PrepareContacts(Contact* _contactArray, unsigned numberOfContacts, real _duration){
    Contact* lastContact = _contactArray + numberOfContacts;
    for (Contact* contact = _contactArray; contact < lastContact; contact++){
        contact->CalculateInternals(_duration);
    }
}

void IPhysicsEngine::ContactResolver::AdjustVelocities(Contact* _contactArray, unsigned numberOfContacts, real _duration){
    Vector3 velocityChange[2];
    Vector3 rotationChange[2];
    Vector3 deltaVelocity;
    velocityIterationsUsed = 0;
    while(velocityIterationsUsed < velocityIterations){
        real max = velocityEpsilon;
        unsigned index = numberOfContacts;
        for(unsigned i = 0; i < numberOfContacts; i++){
            if(_contactArray[i].desiredDeltaVelocity > max){
                max = _contactArray[i].desiredDeltaVelocity;
                index = i;
            }
        }
        if (index == numberOfContacts){
            break;
        }
        _contactArray[index].MatchAwakeState();
        _contactArray[index].ApplyVelocityChange(velocityChange, rotationChange);
        
        // With the change in velocity of the two bodies, the update of contact
        // velocities means that some of the relative closing velocities need replacing.
        for(unsigned i = 0; i < numberOfContacts; i++){
            for(unsigned b = 0; b < 2; b++){
                if(_contactArray[i].body[b]){
                    for(unsigned d = 0; d < 2; d++){
                        if(_contactArray[i].body[b] == _contactArray[index].body[d]){
                            deltaVelocity = velocityChange[d] + rotationChange[d].VectorProduct(_contactArray[i].relativeContactPosition[b]);

                            // The sign of the change is negative if we're dealing with the second body in a contact.
                            int sign = -1;
                            if(b == 0){
                                sign = 1;
                            }
                            _contactArray[i].contactVelocity += _contactArray->contactToWorld.TransformTranspose(deltaVelocity) * sign;
                            _contactArray[i].CalculateDesiredDeltaVelocity(_duration);
                        }
                    }
                }
            }
        }

        velocityIterationsUsed++;
    }
}

void IPhysicsEngine::ContactResolver::AdjustPositions(Contact* _contactArray, unsigned numberOfContacts, real _duration){
    unsigned i;
    unsigned index;
    Vector3 linearChange[2];
    Vector3 angularChange[2];
    Vector3 deltaPosition;
    positionIterationsUsed = 0;
    while(positionIterationsUsed < positionIterations){
        unsigned max = positionEpsilon;
        index = numberOfContacts;
        for (i = 0; i < numberOfContacts; i++)
        {
            if(_contactArray[i].penetration > max){
                max = _contactArray[i].penetration;
                index = i;
            }
        }
        if (index == numberOfContacts){
            break;
        }

        _contactArray[index].MatchAwakeState();
        _contactArray[index].ApplyPositionChange(linearChange, angularChange, max);

        // Could potentially changed the penetration of other bodies, so update contacts.
        for (i = 0; i < numberOfContacts; i++){
            // Check each body in contact
            for(unsigned b = 0; b < 2; b++){
                if(_contactArray[i].body[b]){
                    // Check for a match with each body in the newly resolved contact.
                    for(unsigned d = 0; d < 2; d++){
                        if(_contactArray[i].body[b] == _contactArray[index].body[d]){
                            deltaPosition = linearChange[d] + angularChange[d].VectorProduct(_contactArray[i].relativeContactPosition[b]);
                            
                            // The sign of the change is positive if we're dealing with the second body in a contact,
                            // and negative otherwise
                            int sign = 1;
                            if(b == 0){
                                sign = -1;
                            }
                            _contactArray[i].penetration += deltaPosition.ScalarProduct(_contactArray[i].contactNormal) * sign;
                        }
                    }
                }
            }
        }
        positionIterationsUsed++;
    }
}