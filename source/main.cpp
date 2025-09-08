#include <iostream>
#include <chrono>
#include<unistd.h>

#include "core.hpp"
#include "particle.hpp"

int main(){
    IPhysicsEngine::Vector3 origin(0,0,0);
    IPhysicsEngine::Particle particle1(origin, 1, 1);
    IPhysicsEngine::Particle particle2(origin, 1, 1);
    IPhysicsEngine::Vector3 up(0,100,0);
    IPhysicsEngine::Vector3 down(0,-100,0);

    particle1.SetAcceleration(up);
    particle2.SetAcceleration(down);

    const IPhysicsEngine::real duration = 1.0L / 60.0L;
    int count = 0;

    while(true){
        std::cout << "Duration: " <<  std::to_string(duration) << std::endl;

        particle1.Integrate(duration);
        particle2.Integrate(duration);

        IPhysicsEngine::Vector3 particle1Position = particle1.GetPosition();
        IPhysicsEngine::Vector3 particle2Position = particle2.GetPosition();

        std::cout << "Particle 1:" << std::endl;
        std::cout << "X: " <<  std::to_string(particle1Position.GetX()) << std::endl;
        std::cout << "Y: " <<  std::to_string(particle1Position.GetY()) << std::endl;
        std::cout << "Z: " <<  std::to_string(particle1Position.GetZ()) << std::endl;

        std::cout << "Particle 2:" << std::endl;
        std::cout << "X: " << std::to_string(particle2Position.GetX()) << std::endl;
        std::cout << "Y: " <<  std::to_string(particle2Position.GetY()) << std::endl;
        std::cout << "Z: " <<  std::to_string(particle2Position.GetZ()) << std::endl;

        sleep(duration);
        if(count == 120){
            return 1;
        }
        count++;
    }
}