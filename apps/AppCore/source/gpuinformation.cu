#include "gpuinformation.hpp"

#include <cuda_runtime.h>

void IApp::GPUInformation::Initialise(){
  // Detect if system is CUDA compatible.
  int deviceCount = 0;
  cudaError_t err = cudaGetDeviceCount(&deviceCount);

  int currentDevice = 0;
  cudaGetDevice(&currentDevice);

  if (err != cudaSuccess || deviceCount == 0) {
    return;
  }
  hasCUDA = true;
  cudaDeviceProp properties;
  cudaGetDeviceProperties(&properties, currentDevice);
  deviceName = properties.name;

  computeCapability = 
  std::to_string(properties.major) + 
  "." +  std::to_string(properties.minor);

  totalGlobalMemory = std::to_string(properties.totalGlobalMem);
  multiProcessorCount = std::to_string(properties.multiProcessorCount);
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
