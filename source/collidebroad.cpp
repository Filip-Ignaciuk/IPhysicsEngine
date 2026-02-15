#include "collidebroad.hpp"

template<class BoundingVolumeClass>
IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::BoundingVolumeHierarchyNode(BoundingVolumeHierarchyNode* _parent, const BoundingVolumeClass& _volume, RigidBody* _body) : parent(_parent), volume(_volume), body(_body){
    children[0] = children[1] = nullptr;
}

template<class BoundingVolumeClass>
bool IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::IsLeaf() const{
    return body != nullptr;
}

template<class BoundingVolumeClass>
unsigned IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::GetPotentialContacts(PotentialContact* _contacts, unsigned _limit) const{
    if(IsLeaf() || _limit == 0){
        return 0;
    }
    return children[0]->GetPotentialContactsWith(children[1], _contacts, _limit);
}

template<class BoundingVolumeClass>
void IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::Insert(RigidBody* _newBody, const BoundingVolumeClass& _newVolume){
    if(IsLeaf()){
        children[0] = new BoundingVolumeHierarchyNode<BoundingVolumeClass>(this, volume, body);

        children[1] = new BoundingVolumeHierarchyNode<BoundingVolumeClass>(this, _newVolume, _newBody);

        this->body = nullptr;

        RecalculateBoundingVolume();
    }
    else{
        if(children[0]->volume.GetGrowth(_newVolume) < children[1]->volume.GetGrowth(_newVolume)){
            children[0]->Insert(_newBody, _newVolume);
        }
        else{
            children[1]->Insert(_newBody, _newVolume);
        }
    }
}

template<class BoundingVolumeClass>
bool IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::Overlaps(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other) const{
    return volume->overlaps(_other->volume);
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

template<class BoundingVolumeClass>
IPhysicsEngine::BoundingVolumeHierarchyNode<BoundingVolumeClass>::~BoundingVolumeHierarchyNode(){
    // If we don't have a parent, then we ignore the sibling.
    if(parent){
        BoundingVolumeHierarchyNode<BoundingVolumeClass>* sibling;
        // Find our sibling
        if(parent->children[0] == this){
            sibling = parent->children[1];
        }
        else{
            sibling = parent->children[0];
        }

        parent->volume = sibling->volume;
        parent->body = sibling->body;
        parent->children[0] = sibling->children[0];
        parent->children[1] = sibling->children[1];

        // Delete the sibling
        sibling->parent = nullptr;
        sibling->body = nullptr;
        sibling->children[0] = nullptr;
        sibling->children[1] = nullptr;
        delete sibling;

        // Recalculate the parent's bounding volume.
        parent->RecalculateBoundingVolume();
    }

    if(children[0]){
        children[0]->parent = nullptr;
        delete children[0];
    }
    if(children[1]){
        children[1]->parent = nullptr;
        delete children[1];
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
