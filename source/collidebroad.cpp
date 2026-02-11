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
    Vector3 displacement = _one.m_centre - _two.m_centre;
    real radius = (displacement.Magnitude() + _one.m_radius + _two.m_radius) * 0.5;

    
    m_radius = radius;
}
