#ifndef IGRAVITY_GPUINFORMATION_HPP
#define IGRAVITY_GPUINFORMATION_HPP

#include <string>

namespace IApp {

class GPUInformation{
public:
    static void Initialise();

    // CUDA
    [[nodiscard]] static bool HasCUDA();
    [[nodiscard]] static const std::string& GetDeviceName();
    [[nodiscard]] static const std::string& GetComputeCapability();
    [[nodiscard]] static const std::string& GetTotalGlobalMemory();
    [[nodiscard]] static const std::string& GetMultiProcessorCount();

    private:
    static bool hasCUDA;
    static std::string deviceName;
    static std::string computeCapability;
    static std::string totalGlobalMemory;
    static std::string multiProcessorCount;
};

}

#endif