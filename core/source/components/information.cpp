#include "information.hpp"

IPhysics::Information::Information(){}

std::string IPhysics::Information::GetName(){
    return m_name;
}

void IPhysics::Information::SetName(std::string& _name){
    m_name = _name;
}