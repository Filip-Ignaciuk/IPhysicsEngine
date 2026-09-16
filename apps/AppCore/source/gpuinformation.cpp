#include "gpuinformation.hpp"

// Work around for the ability to compile project on non cuda devices.

void IApp::GPUInformation::Initialise(){
    hasCUDA = false;
}

bool IApp::GPUInformation::HasCUDA(){
  return hasCUDA;
}

const std::string& IApp::GPUInformation::GetDeviceName(){
  return deviceName;
}

const std::string& IApp::GPUInformation::GetComputeCapability(){
  return computeCapability;
}

const std::string& IApp::GPUInformation::GetTotalGlobalMemory(){
  return totalGlobalMemory;
}

const std::string& IApp::GPUInformation::GetMultiProcessorCount(){
  return multiProcessorCount;
}

bool IApp::GPUInformation::hasCUDA;
std::string IApp::GPUInformation::deviceName;
std::string IApp::GPUInformation::computeCapability;
std::string IApp::GPUInformation::totalGlobalMemory;
std::string IApp::GPUInformation::multiProcessorCount;
