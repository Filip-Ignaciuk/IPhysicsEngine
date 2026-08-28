#ifndef IPHYSICS_BHTN_HPP
#define IPHYSICS_BHTN_HPP

#include "core.hpp"
#include "rigidbody.hpp"

struct bhtn {

    ~bhtn() {
        delete nw;
        delete ne;
        delete sw;
        delete se;
    }

    // Data
    IPhysics::real width = 0;
    IPhysics::Vector3 midPoint;

    IPhysics::RigidBody* rigidBody = nullptr;
    IPhysics::real mass = 0;
    IPhysics::Vector3 centreOfMass{};

    bhtn* nw = nullptr;
    bhtn* ne = nullptr;
    bhtn* sw = nullptr;
    bhtn* se = nullptr;

    [[nodiscard]] bool IsExternalNode() const{
        return nw == nullptr
        && ne  == nullptr
        && sw == nullptr
        && se  == nullptr;
    }
};

#endif
