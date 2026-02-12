#include "collidebroad.hpp"

template<class BoundingVolumeClass>
bool IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::IsLeaf() const{
    return body != nullptr;
}

template<class BoundingVolumeClass>
unsigned IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::GetPotentialContacts(PotentialContact* _contacts, unsigned _limit) const{
    if(IsLeaf() || _limit == 0){
        return 0;
    }
    return children[0]->GetPotentialContactsWith(children[1], contacts, limit);
}

template<class BoundingVolumeClass>
unsigned IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::GetPotentialContactsWith(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other, PotentialContact* _contacts, unsigned _limit) const{
    if(!Overlaps(_other) || _limit == 0){
        return 0;
    }

    if(IsLeaf() && _other->IsLeaf()){
        _contacts->body[0] = body;
        _contacts->body[1] = _other->body;
        return 1;
    }

    if(_other->IsLeaf() || (!IsLeaf() && volume->GetSize() >= _other->volume->GetSize())){
        unsigned count = children[0]->GetPotentialContactsWith(_other, _contacts, _limit);

        if(_limit > count){
            return count + children[1]->GetPotentialContactsWith(_other, _contacts + count, _limit - count);
        }
        else{
            return count;
        }
    }
    else{
        unsigned count = GetPotentialContactsWith(_other->children[0], _contacts, _limit);

        if (_limit > count){
            return count + GetPotentialContactsWith(_other->children[1], _contacts + count, _limit - count);
        }
        else return count;

    }
}

IPhysicsEngine::BoundingSphere::BoundingSphere(const Vector3& _centre, real _radius) : m_centre(_centre), m_radius(_radius){

}

IPhysicsEngine::BoundingSphere::BoundingSphere(const BoundingSphere& _one, const BoundingSphere& _two){
    Vector3 centreOffset = _two.m_centre - _one.m_centre;
    real distance = centreOffset.SquareMagnitude();
    real radiusDifference = _two.m_radius - _one.m_radius;
    
    // Check whether one sphere fully encompasses another.
    if (radiusDifference * radiusDifference >= distance){
        if(_one.m_radius > _two.m_radius){
            m_centre = _one.m_centre;
            m_radius = _one.m_radius;
        }
        else{
            m_centre = _two.m_centre;
            m_radius = _two.m_radius;
        }
    }
    else{
        distance = RealSqrt(distance);
        m_radius = (distance + _one.m_radius + _two.m_radius) * ((real)0.5);
        // We calculate the new centre based of the centre of one moved towards two's by an
        // Amoung proportional to two's radius.
        m_centre = _one.m_centre;
        if (distance > 0){
            m_centre += centreOffset * ((m_radius - _one.m_radius)/ distance);
        }
    }
}

bool IPhysicsEngine::BoundingSphere::Overlaps(const BoundingSphere* _other) const{
    Vector3 displacement = _other->m_centre - m_centre;
    real radiusLength = _other->m_radius + m_radius;
    return displacement.SquareMagnitude() < (radiusLength * radiusLength);
}
