#include "collidenarrow.hpp"

void IPhysicsEngine::CollisionData::AddContacts(unsigned _count){
    contactsLeft = contactsLeft - _count;
    contactCount = contactCount + _count;
    contacts = contacts + _count;
}

IPhysicsEngine::real IPhysicsEngine::IntersectionTests::TransformToAxis(const CollisionBox& _box, const Vector3& _axis){
    return _box.halfSize.GetX() * RealAbs(_axis * _box.GetAxis(0)) + _box.halfSize.GetY() * RealAbs(_axis * _box.GetAxis(1)) + _box.halfSize.GetZ() * RealAbs(_axis * _box.GetAxis(2));
}

bool IPhysicsEngine::IntersectionTests::BoxAndHalfSpace(const CollisionBox& _box, const CollisionPlane& _plane){
    real projectedRadius = TransformToAxis(_box, _plane.normal);
    real boxDistance = _plane.normal * _box.GetAxis(3) - projectedRadius;
    return boxDistance  <= _plane.offset;
}

unsigned IPhysicsEngine::CollisionDetector::SphereAndSphere(const CollisionSphere& _firstSphere, const CollisionSphere& _secondSphere, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }
    Vector3 positionOne = _firstSphere.GetAxis(3);
    Vector3 positionTwo = _secondSphere.GetAxis(3);

    Vector3 midline = positionOne - positionTwo;
    real size = midline.Magnitude();

    // Check if the line is in desired range
    if(size <= (real)0.0 || size >= _firstSphere.radius + _secondSphere.radius){
        return 0;
    }

    Vector3 normal = midline * ((real)1.0/size);

    Contact* contact = _data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = positionOne + midline * (real)0.5;
    contact->penetration = (_firstSphere.radius + _secondSphere.radius - size);
    contact->SetBodyData(_firstSphere.rigidbody, _secondSphere.rigidbody, _data->friction, _data->restitution);
    _data->AddContacts(1);
    return 1;

}

unsigned IPhysicsEngine::CollisionDetector::SphereAndHalfSpace(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }
    Vector3 position = _sphere.GetAxis(3);
    // Find the distance from the plane to the centre of the sphere
    real ballDistance = _sphere.radius - (_plane.normal * position) - _plane.offset;
    // Ball is not penatrating the plane.
    if(ballDistance >= 0){
        return 0;
    }

    Contact* contact = _data->contacts;
    contact->contactNormal = _plane.normal;
    contact->penetration = -ballDistance;
    contact->contactPoint = position - _plane.normal * (ballDistance + _sphere.radius);
    contact->SetBodyData(_sphere.rigidbody, NULL, _data->friction, _data->restitution);
    _data->AddContacts(1);
    return 1;
}

unsigned IPhysicsEngine::CollisionDetector::SphereAndPlane(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }
    Vector3 position = _sphere.GetAxis(3);
    // Find the distance from the plane.
    real centreDistance = (_plane.normal * position) - _plane.offset;
    if (centreDistance * centreDistance > _sphere.radius * _sphere.radius){
        return 0;
    }
    Vector3 normal = _plane.normal;
    real penatration = -centreDistance;
    if (centreDistance < 0){
        normal *= -1;
        penatration = - penatration;
    }
    penatration += _sphere.radius;

    Contact* contact = _data->contacts;
    contact->contactNormal = _plane.normal;
    contact->penetration = penatration;
    contact->contactPoint = position - _plane.normal * centreDistance;
    contact->SetBodyData(_sphere.rigidbody, NULL, _data->friction, _data->restitution);
    _data->AddContacts(1);
    return 1;

}

unsigned IPhysicsEngine::CollisionDetector::BoxAndHalfSpace(const CollisionBox& _box, const CollisionPlane& _plane, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }

    if(!IntersectionTests::BoxAndHalfSpace(_box, _plane)){
        return 0;
    }
    // Represents every combination for each halfsize.
    static real mults[8][3] = {{1,1,1}, {-1,1,1}, {1,-1,1}, {-1,-1,1}, {1,1,-1}, {-1,1,-1}, {1,-1,-1}, {-1,-1,-1}};
    Contact* contact = _data->contacts;
    unsigned contactsUsed = 0;
    for(unsigned i = 0; i < 8; ++i){
        Vector3 vertexPosition(mults[i][0], mults[i][1], mults[i][2]);
        vertexPosition.ComponentProductUpdate(_box.halfSize);
        vertexPosition = _box.transform.Transform(vertexPosition);

        real vertexDistance = vertexPosition * _plane.normal;

        
        if(vertexDistance <= _plane.offset){
            // We have penatration.
            contact->contactPoint = _plane.normal;
            contact->contactPoint *= (vertexDistance - _plane.offset);
            contact->contactPoint += vertexPosition;
            contact->contactNormal = _plane.normal;
            contact->penetration = _plane.offset - vertexDistance;
            contact->SetBodyData(_box.rigidbody, _plane.rigidbody, _data->friction, _data->restitution);
            contact++;
            contactsUsed++;
            if(contactsUsed == (unsigned)_data->contactsLeft){
                return contactsUsed;
            }

        }
    }

    _data->AddContacts(contactsUsed);
    return contactsUsed;
}

unsigned IPhysicsEngine::CollisionDetector::BoxAndSphere(const CollisionBox& _box, const CollisionSphere& _sphere, CollisionData* _data){

}
