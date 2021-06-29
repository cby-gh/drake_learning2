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
namespace ourbot{
namespace cby{
namespace extension5{
template <typename T>
class periodic_input final : public  drake::systems::LeafSystem<T>
{
    periodic_input(const drake::trajectories::Trajectory<double>& trajectory, const drake::systems::MultibodyPlant<double>* plant);
    template <typename U>
    explicit periodic_input(const periodic_input<U>& u);
    ~periodic_input() override;
    protected:
    private:
    void CopyInputOut(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output)const;
    template <typename> friend class quadrotor_plant;
    drake::trajectories::Trajectory<double>& trajectory_;
};
} 
}
}