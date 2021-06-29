#include <memory>
#include <math.h>
#include <gflags/gflags.h>

#include "extension11_trajectory_fixed_ankle.h"
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
        drake::systems::BasicVector<T>  cache(6);//plant_.num_positions()

//2HZ
    //    if((current_time_copy>=0)&&(current_time_copy<=0.5))
    //     flag=0;
    //     else if((current_time_copy>0.5)&&(current_time_copy<=1.0))        
    //     flag=1;
    //     else if((current_time_copy>1.0)&&(current_time_copy<=1.5))        
    //     flag=2;
    //     else if((current_time_copy>1.5)&&(current_time_copy<=2.0))        
    //     flag=3;
    //     else if((current_time_copy>2.0)&&(current_time_copy<=2.5))        
    //     flag=4;
    //     else if((current_time_copy>2.5)&&(current_time_copy<=3.0))        
    //     flag=5;
    //     else if((current_time_copy>3.0)&&(current_time_copy<=3.5))        
    //     flag=6;
    //     else if((current_time_copy>3.5)&&(current_time_copy<=4.0))        
    //     flag=7;
    //     else if((current_time_copy>4.0)&&(current_time_copy<=4.5))        
    //     flag=8;
    //     else if((current_time_copy>4.5)&&(current_time_copy<=5.0))        
    //     flag=9;
    //     else if((current_time_copy>5.0)&&(current_time_copy<=5.5))             
    //     flag=10;
    //     else if((current_time_copy>5.5)&&(current_time_copy<=6.0))        
    //     flag=11;
    //     else if((current_time_copy>6.0)&&(current_time_copy<=6.5))        
    //     flag=12;
    //     else if((current_time_copy>6.5)&&(current_time_copy<=7.0))        
    //     flag=13;
    //     else if((current_time_copy>7.0)&&(current_time_copy<=7.5))        
    //     flag=14;
    //     else if((current_time_copy>7.5)&&(current_time_copy<=8.0))        
    //     flag=15;
    //     else if((current_time_copy>8.0)&&(current_time_copy<=8.5))        
    //     flag=16;
    //     else if((current_time_copy>8.5)&&(current_time_copy<=9.0))        
    //     flag=17;
    //     else if((current_time_copy>9.0)&&(current_time_copy<=9.5))        
    //     flag=18;
    //     else if((current_time_copy>9.5)&&(current_time_copy<=10.0))        
    //     flag=19;
    //     else if(current_time_copy>10.0)    
    //     flag=20;
    //     if(flag<=19)
    //     {
    //         T t =current_time_copy-flag*0.5;
    //         drake::MatrixX<T>  traj_values = trajectory_.value(t);
    //         //std::cout<<"traj_values.size()"<<traj_values.size()<<std::endl;
    //         cache.SetAtIndex(0,traj_values(0,0)+0.5*flag);
    //         cache.SetAtIndex(1,traj_values(1,0));
    //         cache.SetAtIndex(2,traj_values(2,0));
    //         cache.SetAtIndex(3,traj_values(3,0));
    //         cache.SetAtIndex(4,traj_values(4,0));
    //         cache.SetAtIndex(5,traj_values(5,0));
    //         //cache.SetAtIndex(6,traj_values(6,0));
    //         //cache.SetAtIndex(7,traj_values(7,0));

    //     }
    //     else
    //     {
    //         std::cout<<"something wrong!!"<<std::endl;
    //     }


//1hz
        if((current_time_copy>=0)&&(current_time_copy<=1))
        flag=0;
        else if((current_time_copy>1)&&(current_time_copy<=2))        
        flag=1;
        else if((current_time_copy>2)&&(current_time_copy<=3))        
        flag=2;
        else if((current_time_copy>3)&&(current_time_copy<=4))        
        flag=3;
        else if((current_time_copy>4)&&(current_time_copy<=5))        
        flag=4;
        else if((current_time_copy>5)&&(current_time_copy<=6))        
        flag=5;
        else if((current_time_copy>6)&&(current_time_copy<=7))        
        flag=6;
        else if((current_time_copy>7)&&(current_time_copy<=8))        
        flag=7;
        else if((current_time_copy>8)&&(current_time_copy<=9))        
        flag=8;
        else if((current_time_copy>9)&&(current_time_copy<=10))        
        flag=9;
        else if(current_time_copy>10)     
        flag=10;
        if(flag<=10)
        {
            T t =current_time_copy-flag*1;
            drake::MatrixX<T>  traj_values = trajectory_.value(t);
            //std::cout<<"traj_values.size()"<<traj_values.size()<<std::endl;
            cache.SetAtIndex(0,traj_values(0,0)+1.0*flag);
            cache.SetAtIndex(1,traj_values(1,0));
            cache.SetAtIndex(2,traj_values(2,0));
            cache.SetAtIndex(3,traj_values(3,0));
            cache.SetAtIndex(4,traj_values(4,0));
            cache.SetAtIndex(5,traj_values(5,0));
            //cache.SetAtIndex(6,traj_values(6,0));
            //cache.SetAtIndex(7,traj_values(7,0));

        }
        else
        {
            std::cout<<"something wrong!!"<<std::endl;
        }




        // //1.6hz
        // if((current_time_copy>=0)&&(current_time_copy<=0.625))
        // flag=0;
        // else if((current_time_copy>0.625)&&(current_time_copy<=1.25))        
        // flag=1;
        // else if((current_time_copy>1.25)&&(current_time_copy<=1.875))        
        // flag=2;
        // else if((current_time_copy>1.875)&&(current_time_copy<=2.5))        
        // flag=3;
        // else if((current_time_copy>2.5)&&(current_time_copy<=3.125))        
        // flag=4;
        // else if((current_time_copy>3.125)&&(current_time_copy<=3.75))        
        // flag=5;
        // else if((current_time_copy>3.75)&&(current_time_copy<=4.375))        
        // flag=6;
        // else if((current_time_copy>4.375)&&(current_time_copy<=5.0))        
        // flag=7;
        // else if((current_time_copy>5.0)&&(current_time_copy<=5.625))        
        // flag=8;
        // else if((current_time_copy>5.625)&&(current_time_copy<=6.25))        
        // flag=9;
        // else if((current_time_copy>6.25)&&(current_time_copy<=6.875))        
        // flag=10;
        // else if((current_time_copy>6.875)&&(current_time_copy<=7.5))        
        // flag=11;
        // else if((current_time_copy>7.5)&&(current_time_copy<=8.125))        
        // flag=12;
        // else if((current_time_copy>8.125)&&(current_time_copy<=8.75))
        // flag=13;        
        // else if((current_time_copy>8.75)&&(current_time_copy<=9.375))        
        // flag=14;
        // else if((current_time_copy>9.375)&&(current_time_copy<=10.0))
        // flag=15;        
        // else if(current_time_copy>10)     
        // flag=16;

        // if(flag<16)
        // {
        //     T t =current_time_copy-flag*0.625;
        //     drake::MatrixX<T>  traj_values = trajectory_.value(t);
        //     //std::cout<<"traj_values.size()"<<traj_values.size()<<std::endl;
        //     cache.SetAtIndex(0,traj_values(0,0)+0.9375*flag);
        //     cache.SetAtIndex(1,traj_values(1,0));
        //     cache.SetAtIndex(2,traj_values(2,0));
        //     cache.SetAtIndex(3,traj_values(3,0));
        //     cache.SetAtIndex(4,traj_values(4,0));
        //     cache.SetAtIndex(5,traj_values(5,0));
        //     //cache.SetAtIndex(6,traj_values(6,0));
        //     //cache.SetAtIndex(7,traj_values(7,0));

        // }
        // else
        // {
        //     std::cout<<"something wrong!!"<<std::endl;
        // }


        //2.5hz
        // if((current_time_copy>=0)&&(current_time_copy<=0.4))
        // flag=0;
        // else if((current_time_copy>0.4)&&(current_time_copy<=0.8))        
        // flag=1;
        // else if((current_time_copy>0.8)&&(current_time_copy<=1.2))        
        // flag=2;
        // else if((current_time_copy>1.2)&&(current_time_copy<=1.6))        
        // flag=3;
        // else if((current_time_copy>1.6)&&(current_time_copy<=2.0))        
        // flag=4;
        // else if((current_time_copy>2.0)&&(current_time_copy<=2.4))        
        // flag=5;
        // else if((current_time_copy>2.4)&&(current_time_copy<=2.8))        
        // flag=6;
        // else if((current_time_copy>2.8)&&(current_time_copy<=3.2))        
        // flag=7;
        // else if((current_time_copy>3.2)&&(current_time_copy<=3.6))        
        // flag=8;
        // else if((current_time_copy>3.6)&&(current_time_copy<=4.0))        
        // flag=9;
        // else if((current_time_copy>4.0)&&(current_time_copy<=4.4))        
        // flag=10;
        // else if((current_time_copy>4.4)&&(current_time_copy<=4.8))        
        // flag=11;
        // else if((current_time_copy>4.8)&&(current_time_copy<=5.2))        
        // flag=12;
        // else if((current_time_copy>5.2)&&(current_time_copy<=5.6))
        // flag=13;        
        // else if((current_time_copy>5.6)&&(current_time_copy<=6.0))        
        // flag=14;
        // else if((current_time_copy>6.0)&&(current_time_copy<=6.4))
        // flag=15; 
        // else if((current_time_copy>6.4)&&(current_time_copy<=6.8))        
        // flag=16;
        // else if((current_time_copy>6.8)&&(current_time_copy<=7.2))        
        // flag=17;
        // else if((current_time_copy>7.2)&&(current_time_copy<=7.6))        
        // flag=18;
        // else if((current_time_copy>7.6)&&(current_time_copy<=8.0))        
        // flag=19;
        // else if((current_time_copy>8.0)&&(current_time_copy<=8.4))        
        // flag=20;
        // else if((current_time_copy>8.4)&&(current_time_copy<=8.8))
        // flag=21;        
        // else if((current_time_copy>8.8)&&(current_time_copy<=9.2))        
        // flag=22;
        // else if((current_time_copy>9.2)&&(current_time_copy<=9.6))
        // flag=23;       
        // else if((current_time_copy>9.6)&&(current_time_copy<=10.0))
        // flag=24;              
        // else if(current_time_copy>10)      
        // flag=25;

        // if(flag<25)
        // {
        //     T t =current_time_copy-flag*0.4;
        //     drake::MatrixX<T>  traj_values = trajectory_.value(t);
        //     //std::cout<<"traj_values.size()"<<traj_values.size()<<std::endl;
        //     cache.SetAtIndex(0,traj_values(0,0)+0.2*flag);
        //     cache.SetAtIndex(1,traj_values(1,0));
        //     cache.SetAtIndex(2,traj_values(2,0));
        //     cache.SetAtIndex(3,traj_values(3,0));
        //     cache.SetAtIndex(4,traj_values(4,0));
        //     cache.SetAtIndex(5,traj_values(5,0));
        //     //cache.SetAtIndex(6,traj_values(6,0));
        //     //cache.SetAtIndex(7,traj_values(7,0));

        // }
        // else
        // {
        //     std::cout<<"something wrong!!"<<std::endl;
        // }
       //std::cout<<"flag"<<flag<<std::endl;
        output->SetFromVector(cache.CopyToVector());
        
    }
} 
}
}
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::dairlib::cby::extension6_trajectory)