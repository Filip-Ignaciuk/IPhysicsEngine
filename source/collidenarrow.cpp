#include "collidenarrow.hpp"

void IPhysicsEngine::CollisionData::AddContacts(unsigned _count){
    contactsLeft = contactsLeft - _count;
    contactCount = contactCount + _count;
    contacts = contacts + _count;
}

unsigned IPhysicsEngine::CollisionDetector::SphereAndSphere(const CollisionSphere& _firstPrimitive, const CollisionSphere& _secondPrimitive, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }
    Vector3 positionOne = _firstPrimitive.GetAxis(3);
    Vector3 positionTwo = _secondPrimitive.GetAxis(3);

    Vector3 midline = positionOne - positionTwo;
    real size = midline.Magnitude();

    // Check if the line is in desired range
    if(size <= (real)0.0 || size >= _firstPrimitive.radius + _secondPrimitive.radius){
        return 0;
    }

    Vector3 normal = midline * ((real)1.0/size);

    Contact* contact = _data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = positionOne + midline * (real)0.5;
    contact->penetration = (_firstPrimitive.radius + _secondPrimitive.radius - size);
    contact->SetBodyData(_firstPrimitive.rigidbody, _secondPrimitive.rigidbody, _data->friction, _data->restitution);
    _data->AddContacts(1);
    return 1;

}

unsigned IPhysicsEngine::CollisionDetector::SphereAndHalfSpace(const CollisionSphere& _firstPrimitive, const CollisionPlane& _secondPrimitive, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }
    Vector3 position = _firstPrimitive.GetAxis(3);
    // Find the distance from the plane to the centre of the sphere
    real ballDistance = _firstPrimitive.radius - (_secondPrimitive.normal * position) - _secondPrimitive.offset;
    // Ball is not penatrating the plane.
    if(ballDistance >= 0){
        return 0;
    }

    Contact* contact = _data->contacts;
    contact->contactNormal = _secondPrimitive.normal;
    contact->penetration = -ballDistance;
    contact->contactPoint = position - _secondPrimitive.normal * (ballDistance + _firstPrimitive.radius);
    contact->SetBodyData(_firstPrimitive.rigidbody, NULL, _data->friction, _data->restitution);
    _data->AddContacts(1);
    return 1;
}

unsigned IPhysicsEngine::CollisionDetector::SphereAndPlane(const CollisionSphere& _firstPrimitive, const CollisionPlane& _secondPrimitive, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }
    Vector3 position = _firstPrimitive.GetAxis(3);
    // Find the distance from the plane.
    real centreDistance = (_secondPrimitive.normal * position) - _secondPrimitive.offset;
    if (centreDistance * centreDistance > _firstPrimitive.radius * _firstPrimitive.radius){
        return 0;
    }
    Vector3 normal = _secondPrimitive.normal;
    real penatration = -centreDistance;
    if (centreDistance < 0){
        normal *= -1;
        penatration = - penatration;
    }
    penatration += _firstPrimitive.radius;

    Contact* contact = _data->contacts;
    contact->contactNormal = _secondPrimitive.normal;
    contact->penetration = penatration;
    contact->contactPoint = position - _secondPrimitive.normal * centreDistance;
    contact->SetBodyData(_firstPrimitive.rigidbody, NULL, _data->friction, _data->restitution);
    _data->AddContacts(1);
    return 1;

}