#include "periodic_input.h"

#include <drake/common/default_scalars.h>
#include <drake/math/rotation_matrix.h>
#include <drake/systems/controllers/linear_quadratic_regulator.h>
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
namespace ourbot{
namespace cby{
namespace extension5{
template<typename T>
periodic_input<T>::periodic_input(const drake::trajectories::Trajectory<double>& trajectory, const drake::systems::MultibodyPlant<double>* plant)
:drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<periodic_input>{}),
trajectory_(trajectory)
{
        //this->DeclareInputPort("fivesecondflag", systems::kVectorValued, 1);//declare a vector input port with size 4
        this->DeclareVectorOutputPort("controller_output", drake::systems::BasicVector<T>(plant->num_positions()),
                            &periodic_input::CopyInputOut,
                            {this->all_state_ticket()});
}

void periodic_input<T>::CopyInputOut(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output)const
{
    T  current_time = context.get_time();
    T traj_time=0.0;
    if(current_time>1)
    {
        traj_time =  current_time-floor(current_time);
    }
    else
    {
        traj_time= current_time;
    }
    
    if(floor(current_time)%2==0)//落足点先右(蓝)后左(红)
    {

    }
}
}
}
}