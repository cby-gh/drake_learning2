#pragma once
#include <memory>

#include <Eigen/Core>
#include "drake/systems/framework/basic_vector.h"
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/primitives/affine_system.h>
#include "drake/systems/framework/event.h" 
#include "drake/systems/framework/witness_function.h"
#include <drake/common/trajectories/trajectory.h>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
namespace drake{
namespace dairlib {
namespace cby{

template<typename T>
class extension6_trajectory final : public drake::systems::LeafSystem<T>
{
    public:
    const drake::multibody::MultibodyPlant<T>* plant_;
    drake::trajectories::PiecewisePolynomial<T> trajectory_;
    extension6_trajectory(const drake::trajectories::PiecewisePolynomial<T>& trajectory, const drake::multibody::MultibodyPlant<T>* plant);
    ~extension6_trajectory() override;
    protected:
    private:
    void copystateout(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
    int n=0;
    int m=1;

};
}
}

// The following code was added to prevent scalar conversion to symbolic scalar
// types. The quadrotor_plant makes use of classes that are not compatible with
// the symbolic scalar. This NonSymbolicTraits is explained in
// drake/systems/framework/system_scalar_converter.h.
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::dairlib::cby::extension6_trajectory> : public NonSymbolicTraits {
};
}  // namespace scalar_conversion
}  // namespace systems

}