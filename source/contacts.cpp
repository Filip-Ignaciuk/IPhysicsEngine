#include "contacts.hpp"
#include "rigidbody.hpp"

void IPhysicsEngine::Contact::SetBodyData(RigidBody* _one, RigidBody* _two, real _friction, real _restitution){
    body[0] = _one;
    body[1] = _two;
    friction = _friction;
    restitution = _restitution;
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
