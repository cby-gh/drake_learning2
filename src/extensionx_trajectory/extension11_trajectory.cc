#include <memory>
#include <math.h>
#include <gflags/gflags.h>

#include "extension11_trajectory.h"
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
        int vx=1;
        int flag=0;
        T current_time = context.get_time();
        T current_time_copy = current_time;//static_cast<double>()
        drake::systems::BasicVector<T>  cache(8);//plant_.num_positions()
        if((current_time_copy>=0)&&(current_time_copy<=2))
        flag=0;
        else if((current_time_copy>2)&&(current_time_copy<=4))        
        flag=1;
        else if((current_time_copy>4)&&(current_time_copy<=6))        
        flag=2;
        else if((current_time_copy>6)&&(current_time_copy<=8))        
        flag=3;
        else if((current_time_copy>8)&&(current_time_copy<=10))        
        flag=4;
        else if(current_time_copy>10)     
        flag=5;

        if((flag==0)||(flag==1)||(flag==2)||(flag==3)||(flag==4))
        {
            T t =current_time_copy-flag*2;
            drake::MatrixX<T>  traj_values = trajectory_.value(t);
            //std::cout<<"traj_values.size()"<<traj_values.size()<<std::endl;
            cache.SetAtIndex(0,traj_values(0,0)+0.4*flag);
            cache.SetAtIndex(1,traj_values(1,0));
            cache.SetAtIndex(2,traj_values(2,0));
            cache.SetAtIndex(3,traj_values(3,0));
            cache.SetAtIndex(4,traj_values(4,0));
            cache.SetAtIndex(5,traj_values(5,0));
            cache.SetAtIndex(6,traj_values(6,0));
            cache.SetAtIndex(7,traj_values(7,0));

        }
        else
        {
            std::cout<<"something wrong!!"<<std::endl;
        }
        //std::cout<<"flag"<<flag<<std::endl;
        output->SetFromVector(cache.CopyToVector());
        
    }
} 
}
}
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::dairlib::cby::extension6_trajectory)