#include <memory>
#include <math.h>
#include <gflags/gflags.h>

#include "extension9_trajectory.h"
#include <drake/lcm/drake_lcm.h>
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/common/is_approx_equal_abstol.h"
namespace drake{
namespace dairlib {
namespace cby{

    template <typename T>
    extension6_trajectory <T>::extension6_trajectory(const drake::trajectories::PiecewisePolynomial<T>& trajectory, const drake::multibody::MultibodyPlant<T>* plant)
    //:drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<extension6_trajectory>{}),
    :trajectory_(trajectory),
    plant_(plant) 
    { 
        this->DeclareVectorOutputPort("trajectory_output", drake::systems::BasicVector<T>(plant->num_positions()),// + plant.num_velocities()
                        &extension6_trajectory::copystateout,
                        {this->all_sources_ticket()});
    }
    //template <typename T>
    //template <typename U>
    //extension6_trajectory<T>:: extension6_trajectory( extension6_trajectory<U> other)
    //    : extension6_trajectory<T>(other.trajectory_, other.plant_) {}//

    template <typename T>
    extension6_trajectory<T>::~extension6_trajectory() {}

    template <typename T>
    void extension6_trajectory<T>::copystateout(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output)const
    {
        T current_time = context.get_time();
        //T current_time_copy = current_time;
        T current_time_copy = current_time;//static_cast<double>()
        drake::systems::BasicVector<T>  cache(9);//plant_.num_positions()
        if((current_time_copy>=0)&&(current_time_copy<=1))
        {
            T t =current_time_copy;
            //std::cout<<"current_time_copy"<<current_time_copy<<std::endl;
            drake::MatrixX<T>  traj_values = trajectory_.value(t);
            cache.SetAtIndex(0,traj_values(0,0));
            cache.SetAtIndex(1,traj_values(1,0));
            cache.SetAtIndex(2,traj_values(2,0));
            cache.SetAtIndex(3,traj_values(3,0));
            cache.SetAtIndex(4,traj_values(4,0));
            cache.SetAtIndex(5,traj_values(5,0));
            cache.SetAtIndex(6,traj_values(6,0));
            cache.SetAtIndex(7,traj_values(7,0));
            cache.SetAtIndex(8,traj_values(8,0));
            //cache.SetAtIndex(8,traj_values(8,0));
            //cache.SetAtIndex(9,traj_values(9,0));
            //cache.SetAtIndex(10,traj_values(10,0));
            //cache.SetAtIndex(11,traj_values(11,0));
            //cache.SetAtIndex(12,traj_values(12,0));
            //cache.SetAtIndex(13,traj_values(13,0));
            //cache.SetAtIndex(14,traj_values(14,0));
            //cache.SetAtIndex(15,traj_values(15,0));
        }
        else if(current_time_copy>1)
        {
            T t = current_time_copy-floor(current_time_copy);
            drake::MatrixX<T>  traj_values = trajectory_.value(t);
            //std::cout<<"current_time_copy"<<current_time_copy<<std::endl;
            if((int)(floor(current_time_copy))%2==0)//偶数相,红色(左足)支撑相->蓝色(右足)支撑相
            {
                cache.SetAtIndex(0,traj_values(0,0)+floor(current_time_copy)*0.2);//planar_x增加,其余不变
                cache.SetAtIndex(1,traj_values(1,0));
                cache.SetAtIndex(2,traj_values(2,0));
                cache.SetAtIndex(3,traj_values(3,0));
                cache.SetAtIndex(4,traj_values(4,0));
                cache.SetAtIndex(5,traj_values(5,0));
                cache.SetAtIndex(6,traj_values(6,0));
                cache.SetAtIndex(7,traj_values(7,0));
                cache.SetAtIndex(8,traj_values(8,0));
                //cache.SetAtIndex(8,traj_values(8,0));
                //cache.SetAtIndex(9,traj_values(9,0));
                //cache.SetAtIndex(10,traj_values(10,0));
                //cache.SetAtIndex(11,traj_values(11,0));
                //cache.SetAtIndex(12,traj_values(12,0));
                //cache.SetAtIndex(13,traj_values(13,0));
                //cache.SetAtIndex(14,traj_values(14,0));
                //cache.SetAtIndex(15,traj_values(15,0));
            }
            else if((int)(floor(current_time_copy))%2==1)//奇数相,蓝色(右足)支撑相->红色(左足)支撑相,左右腿交换
            {
                cache.SetAtIndex(0,traj_values(0,0)+floor(current_time_copy)*0.2);//planar_x增加,其余不变
                cache.SetAtIndex(1,traj_values(1,0));//z相不变
                cache.SetAtIndex(2,traj_values(2,0));//pitch不变
                cache.SetAtIndex(3,traj_values(4,0));
                cache.SetAtIndex(4,traj_values(3,0));
                cache.SetAtIndex(5,traj_values(6,0));
                cache.SetAtIndex(6,traj_values(5,0));
                cache.SetAtIndex(7,traj_values(8,0));
                cache.SetAtIndex(8,traj_values(7,0));
                //cache.SetAtIndex(8,traj_values(8,0));//xdot不变
                //cache.SetAtIndex(9,traj_values(9,0));//zdot不变
                //cache.SetAtIndex(10,traj_values(11,0));
                //cache.SetAtIndex(11,traj_values(10,0));
                //cache.SetAtIndex(12,traj_values(13,0));
                //cache.SetAtIndex(13,traj_values(12,0));
                //cache.SetAtIndex(14,traj_values(15,0));
                //cache.SetAtIndex(15,traj_values(14,0));
            }
        }
        output->SetFromVector(cache.CopyToVector());
        
    }
} 
}
}
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::dairlib::cby::extension6_trajectory)