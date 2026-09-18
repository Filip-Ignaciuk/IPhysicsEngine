#ifndef IGRAVITY_HPP
#define IGRAVITY_HPP

#include "igravitycontroller.hpp"
#include "igravitymodel.hpp"
#include "igravityview.hpp"

class IGravity final{
 public:
  // Constructors
  IGravity();

  // Mutators
  void Run();

 private:


  const int m_screenWidth = 1280;
  const int m_screenHeight = 720;
  const int m_frameRate = 60;
  IPhysics::real m_timeStep;

  IGravityModel* m_iGravityModel;
  IGravityView* m_iGravityView;
  IGravityController* m_iGravityController;

  void Benchmark();
};

#endif
