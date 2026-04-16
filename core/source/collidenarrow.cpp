#include "collidenarrow.hpp"

void IPhysics::CollisionData::AddContacts(unsigned _count){
    contactsLeft = contactsLeft - _count;
    contactCount = contactCount + _count;
    contacts = contacts + _count;
}

IPhysics::real IPhysics::IntersectionTests::TransformToAxis(const CollisionBox& _box, const Vector3& _axis){
    return _box.halfSize.x * RealAbs(_axis * _box.GetAxis(0)) + _box.halfSize.y * RealAbs(_axis * _box.GetAxis(1)) + _box.halfSize.z * RealAbs(_axis * _box.GetAxis(2));
}

bool IPhysics::IntersectionTests::OverlapOnAxis(const CollisionBox& _box1, const CollisionBox& _box2, const Vector3& _axis, const Vector3& _toCentre){
    real projectOne = TransformToAxis(_box1, _axis);
    real projectTwo = TransformToAxis(_box2, _axis);
    real distance = RealAbs(_toCentre * _axis);
    return (distance < projectOne + projectTwo);
}
bool IPhysics::IntersectionTests::BoxAndHalfSpace(const CollisionBox& _box, const CollisionPlane& _plane){
    real projectedRadius = TransformToAxis(_box, _plane.normal);
    real boxDistance = _plane.normal * _box.GetAxis(3) - projectedRadius;
    return boxDistance  <= _plane.offset;
}

bool IPhysics::IntersectionTests::BoxAndBox(const CollisionBox& _box1, const CollisionBox& _box2){
    Vector3 toCentre = _box2.GetAxis(3) - _box1.GetAxis(3);
    return OverlapOnAxis(_box1, _box2, _box1.GetAxis(0), toCentre) && 
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(1), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(2), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box2.GetAxis(0), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box2.GetAxis(1), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box2.GetAxis(2), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(0) % _box2.GetAxis(0), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(0) % _box2.GetAxis(1), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(0) % _box2.GetAxis(2), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(1) % _box2.GetAxis(0), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(1) % _box2.GetAxis(1), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(1) % _box2.GetAxis(2), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(2) % _box2.GetAxis(0), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(2) % _box2.GetAxis(1), toCentre) &&
       OverlapOnAxis(_box1, _box2, _box1.GetAxis(2) % _box2.GetAxis(2), toCentre);
}

unsigned IPhysics::CollisionDetector::SphereAndSphere(const CollisionSphere& _firstSphere, const CollisionSphere& _secondSphere, CollisionData* _data){
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

unsigned IPhysics::CollisionDetector::SphereAndHalfSpace(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data){
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

unsigned IPhysics::CollisionDetector::SphereAndPlane(const CollisionSphere& _sphere, const CollisionPlane& _plane, CollisionData* _data){
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

unsigned IPhysics::CollisionDetector::BoxAndHalfSpace(const CollisionBox& _box, const CollisionPlane& _plane, CollisionData* _data){
    if(_data->contactsLeft <= 0){
        return 0;
    }

    if(!IntersectionTests::BoxAndHalfSpace(_box, _plane)){
        return 0;
    }
    static real multiples[8][3] = {{1,1,1}, {-1,1,1}, {1,-1,1}, {-1,-1,1}, {1,1,-1}, {-1,1,-1}, {1,-1,-1}, {-1,-1,-1}};
    Contact* contact = _data->contacts;
    unsigned contactsUsed = 0;
    for(unsigned i = 0; i < 8; ++i){
        Vector3 vertexPosition(multiples[i][0], multiples[i][1], multiples[i][2]);
        vertexPosition.ComponentProductUpdate(_box.halfSize);
        vertexPosition = _box.transform.Transform(vertexPosition);

        real vertexDistance = vertexPosition * _plane.normal;
        
        if(vertexDistance <= _plane.offset){
            // We have penetration.
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

unsigned IPhysics::CollisionDetector::BoxAndSphere(const CollisionBox& _box, const CollisionSphere& _sphere, CollisionData* _data){
    Vector3 centre =  _sphere.GetAxis(3);
    Vector3 relativeCentre = _box.transform.TransformInverse(centre);

    if( RealAbs(relativeCentre.x) - _sphere.radius > _box.halfSize.x ||
        RealAbs(relativeCentre.y) - _sphere.radius > _box.halfSize.y ||
        RealAbs(relativeCentre.z) - _sphere.radius > _box.halfSize.z
    ){
        return 0;
    }

    Vector3 closestPoint(0,0,0);
    real distance;

    distance = relativeCentre.x;
    if(distance > _box.halfSize.x){
        distance = _box.halfSize.x;
    }
    if(distance < -_box.halfSize.x){
        distance = -_box.halfSize.x;
    }
    closestPoint.x = distance;

    distance = relativeCentre.y;
    if(distance > _box.halfSize.y){
        distance = _box.halfSize.y;
    }
    if(distance < -_box.halfSize.y){
        distance = -_box.halfSize.y;
    }
    closestPoint.y = distance;

    distance = relativeCentre.z;
    if(distance > _box.halfSize.z){
        distance = _box.halfSize.z;
    }
    if(distance < -_box.halfSize.z){
        distance = -_box.halfSize.z;
    }
    closestPoint.z = distance;

    distance = (closestPoint - relativeCentre).SquareMagnitude();
    if(distance > _sphere.radius * _sphere.radius){
        return 0;
    }

    Vector3 closestPointInWorld = _box.transform.Transform(closestPoint);
    Contact* contact = _data->contacts;
    contact->contactNormal = (closestPointInWorld - centre);
    contact->contactNormal.Normalise();
    contact->penetration = _sphere.radius - RealSqrt(distance);
    contact->SetBodyData(_box.rigidbody, _sphere.rigidbody, _data->friction, _data->restitution);
    _data->AddContacts(1);
    return 1;
}
// We use a macro to make our lives easier, this macro is from the book.
#define CHECK_OVERLAP(axis, index) \
    if (!TryAxis(_box1, _box2, (axis), toCentre, (index), penetration, best)) return 0;

unsigned IPhysics::CollisionDetector::BoxAndBox(const CollisionBox& _box1, const CollisionBox& _box2, CollisionData* _data){
    Vector3 toCentre = _box2.GetAxis(3) - _box1.GetAxis(3);
    real penetration = REAL_MAX;
    unsigned best = 0xffffff;

    CHECK_OVERLAP(_box1.GetAxis(0), 0);
    CHECK_OVERLAP(_box1.GetAxis(1), 1);
    CHECK_OVERLAP(_box1.GetAxis(2), 2);

    CHECK_OVERLAP(_box2.GetAxis(0), 3);
    CHECK_OVERLAP(_box2.GetAxis(1), 4);
    CHECK_OVERLAP(_box2.GetAxis(2), 5);

    unsigned bestSingleAxis = best;

    CHECK_OVERLAP(_box1.GetAxis(0) % _box2.GetAxis(0), 6);
    CHECK_OVERLAP(_box1.GetAxis(0) % _box2.GetAxis(1), 7);
    CHECK_OVERLAP(_box1.GetAxis(0) % _box2.GetAxis(2), 8);
    CHECK_OVERLAP(_box1.GetAxis(1) % _box2.GetAxis(0), 9);
    CHECK_OVERLAP(_box1.GetAxis(1) % _box2.GetAxis(1), 10);
    CHECK_OVERLAP(_box1.GetAxis(1) % _box2.GetAxis(2), 11);
    CHECK_OVERLAP(_box1.GetAxis(2) % _box2.GetAxis(0), 12);
    CHECK_OVERLAP(_box1.GetAxis(2) % _box2.GetAxis(1), 13);
    CHECK_OVERLAP(_box1.GetAxis(2) % _box2.GetAxis(2), 14);

    if (best < 3){
        // Box 2 has vertex in face of Box 1
        FillPointFaceBoxBox(_box1, _box2, toCentre, _data, best, penetration);
        _data->AddContacts(1);
        return 1;
    }
    else if (best < 6){
        // Box 1 has vertex in face of Box 2
        FillPointFaceBoxBox(_box2, _box1, toCentre * -1.0f, _data, best - 3, penetration);
        _data->AddContacts(1);
        return 1;
    }
    else{
        best -= 6;
        unsigned oneAxisIndex = best / 3;
        unsigned twoAxisIndex = best % 3;
        Vector3 oneAxis = _box1.GetAxis(oneAxisIndex);
        Vector3 twoAxis = _box2.GetAxis(twoAxisIndex);
        Vector3 axis = oneAxis % twoAxis;
        axis.Normalise();

        // If box is not pointing to the other box then make it.
        if (axis * toCentre > 0){
            axis = axis * -1.0f;
        }

        Vector3 pointOnOneEdge = _box1.halfSize;
        Vector3 pointOnTwoEdge = _box2.halfSize;
        for (unsigned i = 0; i < 3; i++)
        {
            if (i == oneAxisIndex) {
                pointOnOneEdge[i] = 0;
            }
            else if (_box1.GetAxis(i) * axis > 0) {
                pointOnOneEdge[i] = -pointOnOneEdge[i];
            }

            if (i == twoAxisIndex){
                pointOnTwoEdge[i] = 0;
            } 
            else if (_box2.GetAxis(i) * axis < 0) {
                pointOnTwoEdge[i] = -pointOnTwoEdge[i];
            }
        }
        
        pointOnOneEdge = _box1.transform * pointOnOneEdge;
        pointOnTwoEdge = _box2.transform * pointOnTwoEdge;
        
        Vector3 vertex = ContactPoint(pointOnOneEdge, oneAxis, _box1.halfSize[oneAxisIndex], pointOnTwoEdge, twoAxis, _box2.halfSize[twoAxisIndex], bestSingleAxis > 2);

        Contact* contact = _data->contacts;

        contact->penetration = penetration;
        contact->contactNormal = axis;
        contact->contactPoint = vertex;
        contact->SetBodyData(_box2.rigidbody, _box2.rigidbody, _data->friction, _data->restitution);
        _data->AddContacts(1);
        return 1;
    }
    return 0;
}

bool IPhysics::CollisionDetector::TryAxis(const CollisionBox& _box1, const CollisionBox& _box2, Vector3 _axis, const Vector3& _toCentre, unsigned _index, real& _smallestPenetration, unsigned& _smallestCase){
    // Dont bother checking almost parallel axes.
    if(_axis.SquareMagnitude() < 0.0001){
        return true;
    }

    _axis.Normalise();
    real penetration = PenetrationOnAxis(_box1, _box2, _axis, _toCentre);
    if (penetration < 0){
        return false;
    }
    if(penetration < _smallestPenetration){
        _smallestPenetration = penetration;
        _smallestCase = _index;
    }
    return true;
}


IPhysics::real IPhysics::CollisionDetector::PenetrationOnAxis(const CollisionBox& _box1, const CollisionBox& _box2, const Vector3& _axis, const Vector3& _toCentre){
    real projectOne = IntersectionTests::TransformToAxis(_box1, _axis);
    real projectTwo = IntersectionTests::TransformToAxis(_box2, _axis);
    real distance = RealAbs(_toCentre * _axis);
    return projectOne + projectTwo - distance;
}

void IPhysics::CollisionDetector::FillPointFaceBoxBox(const CollisionBox& _box1, const CollisionBox& _box2, const Vector3& _toCentre, CollisionData* _data, unsigned _best, real _penetration){
    // This method defaults with vertex from box 2 intersecting a fact from box 1.
    Contact* contact = _data->contacts;
    Vector3 normal = _box1.GetAxis(_best);
    if (_box1.GetAxis(_best) * _toCentre  > 0){
        normal = normal * -1.0f;
    }
    
    Vector3 vertex = _box2.halfSize;
    if (_box2.GetAxis(0) * normal < 0) vertex.x = -vertex.x;
    if (_box2.GetAxis(1) * normal < 0) vertex.y = -vertex.y;
    if (_box2.GetAxis(2) * normal < 0) vertex.z = -vertex.z;

    contact->contactNormal = normal;
    contact->penetration = _penetration;
    contact->contactPoint = _box2.transform * vertex;
    contact->SetBodyData(_box1.rigidbody, _box2.rigidbody, _data->friction, _data->restitution);
}

IPhysics::Vector3 IPhysics::CollisionDetector::ContactPoint(const Vector3& _pointOnOneEdge, const Vector3& _oneAxis, real _oneSize, const Vector3& _pointOnTwoEdge, const Vector3& _twoAxis, real _twoSize, bool _useOne){
    Vector3 centreOffset = _pointOnOneEdge - _pointOnTwoEdge;
    real edge1LengthSquared = _oneAxis.SquareMagnitude();
    real edge2LengthSquared = _twoAxis.SquareMagnitude();
    real directionDot = _oneAxis * _twoAxis;

    real offsetAlongEdge1 = _oneAxis * centreOffset;
    real offsetAlongEdge2 = _twoAxis * centreOffset;

    real denominator = edge1LengthSquared * edge2LengthSquared - directionDot * directionDot;

    if(RealAbs(denominator) < 0.0001f){
        if(_useOne){
            return _pointOnOneEdge;
        }
        else{
            return _pointOnTwoEdge;
        }
    }

    real edge1ClosestOffset = (directionDot * offsetAlongEdge2 - edge2LengthSquared * offsetAlongEdge1) / denominator;
    real edge2ClosestOffset = (edge1LengthSquared * offsetAlongEdge2 - directionDot * offsetAlongEdge1) / denominator;

    if (edge1ClosestOffset > _oneSize || edge1ClosestOffset < -_oneSize ||
        edge2ClosestOffset > _twoSize || edge2ClosestOffset < -_twoSize)
    {
        if(_useOne){
            return _pointOnOneEdge;
        }
        else{
            return _pointOnTwoEdge;
        }
    }

    Vector3 closestPointOnEdge1 = _pointOnOneEdge + _oneAxis * edge1ClosestOffset;
    Vector3 closestPointOnEdge2 = _pointOnTwoEdge + _twoAxis * edge2ClosestOffset;

    return (closestPointOnEdge1 + closestPointOnEdge2) * 0.5f;
}
