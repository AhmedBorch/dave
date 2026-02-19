#ifndef SONAR_3D_HH_
#define SONAR_3D_HH_

#include <gz/sensors/RenderingSensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <memory>
#include "sonar_calculation_cuda.cuh"

namespace gz {
namespace sensors {

class Sonar3D : public RenderingSensor {
    public: Sonar3D();
    public: virtual ~Sonar3D();
};

}
}
#endif