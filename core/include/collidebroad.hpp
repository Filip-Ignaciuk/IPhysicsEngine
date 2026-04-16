#pragma once
#include "core.hpp"
#include "object.hpp"
#include "components/rigidbody.hpp"
namespace IPhysics{
    struct PotentialContact
    {
        Object* object[2];
    };

    template<class BoundingVolumeClass>
    class BoundingVolumeHierarchyNode{
        public:
        BoundingVolumeHierarchyNode* children[2];

        BoundingVolumeClass volume;

        Object* object;

        BoundingVolumeHierarchyNode* parent;


        BoundingVolumeHierarchyNode(BoundingVolumeHierarchyNode* _parent, const BoundingVolumeClass _volume, Object* _object) : parent(_parent), volume(_volume), object(_object){
            children[0] = children[1] = nullptr;
        }

        bool IsLeaf() const{
            return object != nullptr;
        }

        unsigned GetPotentialContacts(PotentialContact* _contacts, unsigned _limit) const{
            if(IsLeaf() || _limit == 0){
                return 0;
            }
            return children[0]->GetPotentialContactsWith(children[1], _contacts, _limit);
        }

        void Insert(Object* _newObject, const BoundingVolumeClass _newVolume){
            if(IsLeaf()){
                children[0] = new BoundingVolumeHierarchyNode<BoundingVolumeClass>(this, volume, object);

                children[1] = new BoundingVolumeHierarchyNode<BoundingVolumeClass>(this, _newVolume, _newObject);

                this->object = nullptr;

                RecalculateBoundingVolume();
            }
            else{
                if(children[0]->volume.GetGrowth(_newVolume) < children[1]->volume.GetGrowth(_newVolume)){
                    children[0]->Insert(_newObject, _newVolume);
                }
                else{
                    children[1]->Insert(_newObject, _newVolume);
                }
            }
        }

        ~BoundingVolumeHierarchyNode(){
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
                parent->object = sibling->object;
                parent->children[0] = sibling->children[0];
                parent->children[1] = sibling->children[1];

                // Delete the sibling
                sibling->parent = nullptr;
                sibling->object = nullptr;
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

        

        protected:
        bool Overlaps(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other) const{
            return volume.Overlaps(&_other->volume);
        }

        unsigned GetPotentialContactsWith(const BoundingVolumeHierarchyNode<BoundingVolumeClass>* _other, PotentialContact* _contacts, unsigned _limit) const{
            if(!Overlaps(_other) || _limit == 0){
                return 0;
            }

            if(IsLeaf() && _other->IsLeaf()){
                _contacts->object[0] = object;
                _contacts->object[1] = _other->object;
                return 1;
            }

            if(_other->IsLeaf() || (!IsLeaf() && volume.GetSize() >= _other->volume.GetSize())){
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

        void RecalculateBoundingVolume(bool recurse = true){
            if (IsLeaf()){
                return;
            }
            volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);
            if (parent) {
                parent->RecalculateBoundingVolume(true);
            }
        }


    };
}

namespace IPhysics{
    struct BoundingSphere{
        Vector3 m_centre;
        real m_radius;
        BoundingSphere(const Vector3& _centre, real _radius);
        BoundingSphere(const BoundingSphere& _one, const BoundingSphere& _two);
        bool Overlaps(const BoundingSphere* _other) const;
        real GetGrowth(const BoundingSphere &other) const;
        real GetSize() const;
    };
}