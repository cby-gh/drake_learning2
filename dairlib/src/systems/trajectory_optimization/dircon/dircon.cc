#include "systems/trajectory_optimization/dircon/dircon.h"

#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"
#include "systems/trajectory_optimization/dircon/dircon_opt_constraints.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;
using drake::solvers::Constraint;
using drake::solvers::Binding;
    
using Eigen::MatrixXd;
using Eigen::VectorXd;

using multibody::KinematicAccelerationConstraint;
using multibody::KinematicPositionConstraint;
using multibody::foot_location_Constraint;
//using multibody::floatfoot_collosionavoidence_Constraint;
using multibody::KinematicVelocityConstraint;
//z注意通过mode_sequence和mode初始化的knots不一样count_knotpoints具有重叠的knots
template <typename T>
Dircon<T>::Dircon(const DirconModeSequence<T>& mode_sequence)
    : Dircon<T>({}, &mode_sequence, mode_sequence.plant(), mode_sequence.count_knotpoints()) 
{
        std::cout<<" mode_sequence.count_knotpoints()"<< mode_sequence.count_knotpoints()<<std::endl;
}
//z注意通过mode_sequence和mode初始化的knots不一样num_knotpoints不具有重叠的knots
template <typename T>
Dircon<T>::Dircon(DirconMode<T>* mode)
    : Dircon<T>(std::make_unique<DirconModeSequence<T>>(mode), nullptr,
                mode->plant(), mode->num_knotpoints())
{}

/// Private constructor. Determines which DirconModeSequence was provided,
/// a locally owned unique_ptr or an externally owned const reference
//测试代码， dircon的dicon构造函数基本上会生成约束和决策变量实体，142条决策变量，对于PlanarWalker两个mode
//auto all_constraint_begin  = GetAllConstraints();
//std::cout<<"num of dircon constraints: "<<all_constraint_begin.size()<<std::endl; 
//std::cout<<"content of begin dircon: "<<all_constraint_begin[0].evaluator()->upper_bound()<<std::endl; 
//std::cout<< "force_vars_ content:  " << impulse_vars_.size()<<std::endl;     
//std::cout<< "force_vars_ content size:  " << impulse_vars_[0].size()<<std::endl;    
//std::cout<< "force_vars_ content size:  " << impulse_vars_[1].size()<<std::endl;   
//          if(i_mode>0)
//{}     
template <typename T>
Dircon<T>::Dircon(std::unique_ptr<DirconModeSequence<T>> my_sequence,
                  const DirconModeSequence<T>* ext_sequence,
                  const MultibodyPlant<T>& plant, int num_knotpoints)
    : drake::systems::trajectory_optimization::MultipleShooting(
          plant.num_actuators(), plant.num_positions() + plant.num_velocities(), 
    num_knotpoints, 1e-8, 1e8),
    my_sequence_(std::move(my_sequence)),
    plant_(plant),
    mode_sequence_(ext_sequence ? *ext_sequence : *my_sequence_),
    contexts_(num_modes()),
    mode_start_(num_modes()) 
{
  //在初始化drake::systems::trajectory_optimization::MultipleShooting之后，已经存在一个约束，关于整体轨迹时长
  //两段轨迹，两个mode，每段九个,传入的时候扣除了重合的点，所以传入的时候就是1个knots，共18段，
  //即GetAllConstraints的第一条约束是初始化timestep时长约束
  //初始化完mutipleshooting之后存在的决策变量和初始约束和cost
  //初始化完mutipleshooting之后存在的决策变量和初始约束和cost
  // std::cout<<"num_knotpoints "<<num_knotpoints<<std::endl;//10+10 19
   std::cout<<"modex decision_variables-0 num_vars() "<<num_vars()<<std::endl;//10+10 398//10+20 608
  // std::cout<<"modex decision_variables-1 h_vars() "<<h_vars()<<std::endl;//10+10 18//10+20 28
  // std::cout<<"modex decision_variables-2 u_vars() "<<u_vars()<<std::endl;//10+10 76//10+20  116
  // std::cout<<"modex decision_variables-3 x_vars() "<<x_vars()<<std::endl;//10+10 304//10+20
  // std::cout<<"modex decision_variables-4 decision_variables() "<<decision_variables().size()<<std::endl;//398 608
  // std::cout<<"modex decision_variables-5 decision_variables() "<<decision_variables()<<std::endl;//398每个变量都在
  // std::cout<<"modex decision_variables-6 decision_variable(0).to_string()"<<decision_variable(0).to_string()<<std::endl;//398    
  // std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_initialization_mutipleshooting = GetAllConstraints();
  // std::cout<<"modex constraints-0 num of constraints after initialization mutipleshooting: "<<contraint_after_initialization_mutipleshooting.size()<<std::endl;
  // std::cout<<"modex constraints-1 num of constraints after initialization mutipleshooting: "<<std::endl;      
  // for(int i=0; i<contraint_after_initialization_mutipleshooting.size(); i++)
  // {
  //     std::cout<<"contraint_after_initialization_mutipleshooting.to_string(): "<<contraint_after_initialization_mutipleshooting[i].to_string()<<std::endl;//
  //     std::cout<<"contraint_after_initialization_mutipleshooting.variables(): "<<contraint_after_initialization_mutipleshooting[i].variables()<<std::endl; 
  //     std::cout<<"contraint_after_initialization_mutipleshooting.GetNumElements(): "<<contraint_after_initialization_mutipleshooting[i].GetNumElements()<<std::endl;     
  //  }

  // std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_initialization_mutipleshooting = GetAllCosts();
  // std::cout<<"modex costs-0num of cost after initialization mutipleshooting: "<<cost_after_initialization_mutipleshooting.size()<<std::endl;      
  // std::cout<<"modex costs-1 num of costs after initialization mutipleshooting: "<<std::endl;   
  // for(int i=0; i<cost_after_initialization_mutipleshooting.size(); i++)//no initial cost after multipleshooting
  //  {
  //       std::cout<<"cost_after_initialization_mutipleshooting.to_string(): "<<cost_after_initialization_mutipleshooting[i].to_string()<<std::endl;//
  //       std::cout<<"cost_after_initialization_mutipleshooting.variables(): "<<cost_after_initialization_mutipleshooting[i].variables()<<std::endl; 
  //       std::cout<<"cost_after_initialization_mutipleshooting.GetNumElements(): "<<cost_after_initialization_mutipleshooting[i].GetNumElements()<<std::endl;     
  //  }

  // Loop over all modes
  double total_time_i_time0 ;
  double total_time_i_time1 ;
  std::cout<<"num_modes():"<<num_modes()<<std::endl;
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) 
  {
    const auto& mode = get_mode(i_mode);
    // Identify starting index for this mode, accounting for shared knotpoints
    if (i_mode == 0)
    {
      mode_start_[i_mode] = 0;
    } 
      else
    {
      mode_start_[i_mode] =mode_start_[i_mode - 1] + mode_length(i_mode - 1) - 1;//将上一个mode的结束点作为新mode的起点,
      //而且本来就减去了重叠的knots,数目上也对
    }//mode_start_相当于存的是每个mode 开始的 sample time 在整体break points序列中的位置
    std::cout<<"mode_start_[i_mode]: "<<mode_start_[i_mode]<<std::endl;//两个mode就是0和9
    std::cout<<"i_mode: "<<i_mode<<std::endl;
    //
    // Set constraints on timesteps
    //
    if (mode_length(i_mode) > 1) //设置timestep约束,如果不大于1，成了单相了,很多约束没有意义，比如左右步的累计时长应当相等
    {
      double min_dt = mode.min_T() / (mode.num_knotpoints() - 1);
      double max_dt = mode.max_T() / (mode.num_knotpoints() - 1);       
      AddBoundingBoxConstraint(min_dt, max_dt, timestep(mode_start_[i_mode]));//注意这里仅仅约束了第一个,后续两两相等的初始量
      // if(i_mode == 0)
      //  {
      //      std::vector<drake::solvers::Binding<drake::solvers::Constraint> > constraints_after_set_timestep0= GetAllConstraints();
      //      std::cout<<"mode0 constraints-2 num of constraints after set specific time step: "<<GetAllConstraints().size()<<std::endl; //2
      //      for(int i=0; i<constraints_after_set_timestep0.size(); i++)
      //     {
      //         std::cout<<"constraints_after_set_timestep0.to_string(): "<<constraints_after_set_timestep0[i].to_string()<<std::endl;//
      //         std::cout<<"constraints_after_set_timestep0.variables(): "<<constraints_after_set_timestep0[i].variables()<<std::endl; 
      //         std::cout<<"constraints_after_set_timestep0.GetNumElements(): "<<constraints_after_set_timestep0[i].GetNumElements()<<std::endl;     
      //     }
      //   }
      // else if(i_mode == 1)
      //  {
      //       std::vector<drake::solvers::Binding<drake::solvers::Constraint> > constraints_after_set_timestep= GetAllConstraints();
      //       std::cout<<"mode1 constraints-0 num of constraints after set specific time step: "<<GetAllConstraints().size()<<std::endl; //72
      //  }
      for (int j = 0; j < mode.num_knotpoints() - 2; j++) 
      {
          // all timesteps must be equal
          AddLinearConstraint(timestep(mode_start_[i_mode] + j) ==timestep(mode_start_[i_mode] + j + 1));
      }
      // if(i_mode == 0)
      //  {
      //      std::cout<<"mode0 constraints-3 num of constraints after set all timesteps must be equal: "<<GetAllConstraints().size()<<std::endl;  //10
      //  }
      // else if(i_mode == 1)
      // {
      //         std::cout<<"mode1 constraints-1 num of constraints after set specific time step: "<<GetAllConstraints().size()<<std::endl; //80
      // }
      //且左右步的累计时长也一致
/*为了测试向前伸腿的可能性,注释掉了*/
      //AddConstraint(h_vars()[0]+h_vars()[1]+h_vars()[2]+h_vars()[3]+h_vars()[4]+h_vars()[5]+h_vars()[6]+h_vars()[7]+h_vars()[8]==
      //              h_vars()[9]+h_vars()[10]+h_vars()[11]+h_vars()[12]+h_vars()[13]+h_vars()[14]+h_vars()[15]+h_vars()[16]+h_vars()[17]);

      // if(i_mode==0)
      // {
      //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_timeconstraint = GetAllConstraints();
      //     std::cout<<"mode0 constraints-4 num of constraints after set two mode durarions are same:  "<<contraint_after_timeconstraint.size()<<std::endl;  //11
      //     std::cout<<"check after set timestep constraints of mode 0"<<std::endl;  //11
      //     for(int i=0;i<contraint_after_timeconstraint.size();i++)
      //     std::cout<<"constriant["<<i<<"]: "<<contraint_after_timeconstraint[i].to_string()<<std::endl;  
      // }
      // else if(i_mode == 1)
      // {
      //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > cost_after_set_timestep= GetAllConstraints();
      //     std::cout<<"mode1 constraints-2 num of constraints after set specific time step: "<<GetAllConstraints().size()<<std::endl; //81
      // }
    }
    // Create new decision variables
    //在添加之前drake::systems::trajectory_optimization::MultipleShooting初始化完成就已经添加了398个变量
    force_vars_.push_back(NewContinuousVariables(mode.evaluators().count_full() * mode.num_knotpoints(),
                                            "lambda[" + std::to_string(i_mode) + "]"));
    // force_vars_.push_back(NewContinuousVariables(mode.evaluators().get_evaluator(0).num_full() * mode.num_knotpoints(),
    //                                         "lambda[" + std::to_string(i_mode) + "]"));
    //if(i_mode==0)
    // {
    //     std::cout<<"mode0 decision_variables-7 after add lamda: "<<decision_variables().size()<<std::endl;//398+10*3=428
    //     std::cout<<"modex decision_variables-8 after add lamda: "<<decision_variables()<<std::endl;//每个变量都在
    //     std::cout<<"mode0 decision_variables-9 after add lamda: to_string()"<<decision_variable(0).to_string()<<std::endl;
    //  } 
    // else if(i_mode==1)
    // {
    //    std::cout<<"mode1 decision_variables-0 after add lamda: "<<decision_variables().size()<<std::endl;//483+30=513
    //    std::cout<<"modex decision_variables-8 after add lamda: "<<decision_variables()<<std::endl;//每个变量都在
    //    std::cout<<"mode1 decision_variables-0 after add lamda: to_string()"<<decision_variable(0).to_string()<<std::endl;
    //  }   
    collocation_force_vars_.push_back(NewContinuousVariables(mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
                                                              "lambda_c[" + std::to_string(i_mode) + "]"));
    // collocation_force_vars_.push_back(NewContinuousVariables(mode.evaluators().get_evaluator(0).num_full() * (mode.num_knotpoints() - 1),
    //                                                           "lambda_c[" + std::to_string(i_mode) + "]"));
  //   if(i_mode==0)
  //   {   
  //         std::cout<<"mode0 decision_variables-10 after add lamda_c:"<<decision_variables().size()<<std::endl;//428+9*3=455
  //         std::cout<<"modex decision_variables-11 after add lamda_c: "<<decision_variables()<<std::endl;//每个变量都在
  //         std::cout<<"mode0 decision_variables-12 after add lamda_c:to_string()"<<decision_variable(0).to_string()<<std::endl;
  //   }    
  //  else  if(i_mode==1)
  //  {   
  //       std::cout<<"mode1 decision_variables-1 after add lamda_c:"<<decision_variables().size()<<std::endl;//513+9*3=540
  //       std::cout<<"modex decision_variables-11 after add lamda_c: "<<decision_variables()<<std::endl;//每个变量都在
  //       std::cout<<"modex decision_variables-12 after add lamda_c:to_string()"<<decision_variable(0).to_string()<<std::endl;
  //  }            
    collocation_slack_vars_.push_back(NewContinuousVariables(mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
                                                                 "gamma[" + std::to_string(i_mode) + "]"));//num_knotpoints=10, num_knotpoints-1=9
    // collocation_slack_vars_.push_back(NewContinuousVariables(mode.evaluators().get_evaluator(0).num_full()* (mode.num_knotpoints() - 1),
    //                                                              "gamma[" + std::to_string(i_mode) + "]"));//num_knotpoints=10, num_knotpoints-1=9
    // if(i_mode==0)
    // {      
    //     std::cout<<"mode0 decision_variables-13 after add gamma_c:"<<decision_variables().size()<<std::endl;//455+9*3=482
    //     std::cout<<"modex decision_variables-14 after add gamma_c: "<<decision_variables()<<std::endl;//每个变量都在
    //     std::cout<<"mode0 decision_variables-15 after add gamma_c:to_string()"<<decision_variable(0).to_string()<<std::endl;//
    // } 
    // else if(i_mode==1)
    // {      
    //     std::cout<<"mode1 decision_variables-13 after add gamma_c:"<<decision_variables().size()<<std::endl;//540+9*3=567
    //     std::cout<<"modex decision_variables-14 after add gamma_c: "<<decision_variables()<<std::endl;//每个变量都在
    //     std::cout<<"mode1 decision_variables-15 after add gamma_c:to_string()"<<decision_variable(0).to_string()<<std::endl;//
    //  }          
    int current_vars = num_vars();
    std::cout<<"current_vars"<<current_vars<<std::endl;//
      // quaternion_slack_vars_ (slack variables used to scale quaternion norm to
      // 1 in the dynamic constraints)
    int num_quat = multibody::QuaternionStartIndices(plant_).size();
    std::cout<<"num of after add  quaternion_slack_vars_ 0: "<<num_quat<<std::endl; //目前是零
    quaternion_slack_vars_.push_back(NewContinuousVariables(num_quat * (mode.num_knotpoints() - 1),
                                                              "quat_slack[" + std::to_string(i_mode) + "]"));//0=num_quat
    //  if(i_mode==0)
    //  {
    //       std::cout<<"mode0 decision_variables-16 after add quat decision variable: "<<decision_variables().size()<<std::endl;//=482
    //       std::cout<<"modex decision_variables-17 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
    //       std::cout<<"mode0 decision_variables-18 after add quat decision variable: to_string()"<<decision_variable(0).to_string()<<std::endl;//
    // } 
    // else if(i_mode==1)
    // { 
    //     std::cout<<"mode1 decision_variables-16 after add quat decision variable: "<<decision_variables().size()<<std::endl;//567
    //     std::cout<<"modex decision_variables-17 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
    //     std::cout<<"mode1 decision_variables-18 after add quat decision variable: to_string()"<<decision_variable(0).to_string()<<std::endl;//
    // } 
        // Bound quaternion slack variables to avoid false full rotations
      double slack_bound = 1;
      AddBoundingBoxConstraint(-slack_bound, slack_bound,quaternion_slack_vars_.at(i_mode));//但在mode0中是一个空约束
      // if(i_mode == 0 )
      // {
      //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_quat = GetAllConstraints();
      //     std::cout<<"mode0 constraints-5 after set quat cosntraint: "<<contraint_after_quat.size()<<std::endl;  
      //     for(int i=0;i<contraint_after_quat.size();i++)
      //            std::cout<<"constriant["<<i<<"]: "<<contraint_after_quat[i].to_string()<<std::endl;  
      //     std::cout<<"mode0 constraints-6 contraint_after_quat.to_string(): "<<contraint_after_quat[9].to_string()<<std::endl;
      //     std::cout<<"mode0 constraints-7 contraint_after_quat.to_string(): "<<contraint_after_quat[10].to_string()<<std::endl; 
      //     std::cout<<"mode0 constraints-8 contraint_after_quat.to_string(): "<<contraint_after_quat[11].to_string()<<std::endl; 
      // }
      // else if(i_mode == 1)
      // {
      //   std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_quat = GetAllConstraints();
      //   std::cout<<"mode1 constraints-5 after set quat cosntraint: "<<contraint_after_quat.size()<<std::endl;  //81+1=82
      //   for(int i=0;i<contraint_after_quat.size();i++)
      //     std::cout<<"constriant["<<i<<"]: "<<contraint_after_quat[i].to_string()<<std::endl;  
      //   std::cout<<"mode0 constraints-6 contraint_after_quat.to_string(): "<<contraint_after_quat[9].to_string()<<std::endl;
      //   std::cout<<"mode0 constraints-7 contraint_after_quat.to_string(): "<<contraint_after_quat[10].to_string()<<std::endl; 
      //   std::cout<<"mode0 constraints-8 contraint_after_quat.to_string(): "<<contraint_after_quat[11].to_string()<<std::endl; 
      // }
        //  Post-impact variables. Note: impulse variables declared below. 碰撞后的速度约束
        if (i_mode > 0) 
        {
          v_post_impact_vars_.push_back(NewContinuousVariables(plant_.num_velocities(), "v_p[" + std::to_string(i_mode) + "]"));       
          // if(i_mode==1)
          // {
          //     std::cout<<"mode1 decision_variables-17 after add v_post_impact_vars_: "<<decision_variables().size()<<std::endl;//567+8=575
          //     std::cout<<"modex decision_variables-20 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
          //     std::cout<<"mode0 decision_variables-21 after add quat decision variable: to_string()"<<decision_variable(482).to_string()<<std::endl;//
          //     std::cout<<"mode0 decision_variables-22 after add quat decision variable: to_string()"<<decision_variable(481).to_string()<<std::endl;
          // } 
        }

        // Constraint offset variables for relative constraints
          //make_constriants_relative()这里就是relative方向的足端前进距离的决策变量
        offset_vars_.push_back(NewContinuousVariables(mode.num_relative_constraints(),
                                                      "rel_offset[" + std::to_string(i_mode) + "]"));   
        // std::cout<<mode.evaluators().get_evaluator(1).num_full()<<std::endl;//应当为3
        for(int k=0; k<mode_length(i_mode);k++)                
        {
          floatfoot_collosionavoidence_rel_offset_vars_.push_back(NewContinuousVariables(mode.evaluators().get_evaluator(0).num_full(),
                                        "floatfoot_collosionavoidence_rel_offset[" + std::to_string(i_mode) + "]"+"["+std::to_string(k)));   
        }                          

        // if(i_mode==0)
        // {
        //   std::cout<<"mode0 decision_variables-19 after add offset_vars_: "<<decision_variables().size()<<std::endl;//482+1=483
        //   std::cout<<"modex decision_variables-20 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
        //   std::cout<<"mode0 decision_variables-21 after add quat decision variable: to_string()"<<decision_variable(482).to_string()<<std::endl;//
        //   std::cout<<"mode0 decision_variables-22 after add quat decision variable: to_string()"<<decision_variable(481).to_string()<<std::endl;
        // } 
        // else if(i_mode==1) 
        // {
        //     std::cout<<"mode1 decision_variables-19 after add offset_vars_: "<<decision_variables().size()<<std::endl;//575+1=576
        //     std::cout<<"modex decision_variables-20 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
        //     std::cout<<"mode0 decision_variables-21 after add quat decision variable: to_string()"<<decision_variable(482).to_string()<<std::endl;//
        //     std::cout<<"mode0 decision_variables-22 after add quat decision variable: to_string()"<<decision_variable(481).to_string()<<std::endl;
        // } 
        //
        // Create context elements for knot points
        //为每一个knots创建一个context,每个knots节点都提取其context
        for (int j = 0; j < mode.num_knotpoints(); j++)
        {
          contexts_[i_mode].push_back(std::move(plant_.CreateDefaultContext()));
        }
        // if(i_mode == 0)
        // {
        //     std::cout<<"mode0 context_vector[0]_vector contexts_vector.size:"<<contexts_[0].size()<<std::endl;//10 contexts
        // }
        //     else if(i_mode == 1)
        // {
        // std::cout<<"mode1 context_vector[1]_vector contexts_vector.size:"<<contexts_[1].size()<<std::endl;//10 contexts
        // }

        // We only need an impact if the post-impact constraint set contains
        // anything not already in the pre-impact constraint set.
        //只有当前后两个mode的evaluators(约束计算器)的内容不一样时,说明发生了碰撞,比如mode0左足支撑相,约束左足z坐标为零,
        //mode1右足支撑相,右足z坐标为0
        bool is_impact = false;
        if (i_mode > 0) //且在存在两个以上的mode时才有意义
        {
          auto prev_evaluators = get_mode(i_mode - 1).evaluators().get_evaluators();// a mode-> a set->all evaluators
          for (auto e : mode.evaluators().get_evaluators()) 
          {
            if (std::find(prev_evaluators.begin(), prev_evaluators.end(), e) ==prev_evaluators.end()) 
            {
              is_impact = true;
              break;
            }
          }
          std::cout<<"is_impact is_impact is_impact:"<<is_impact<<std::endl;//10 contexts  
        }

        //
        // Create and add collocation constraints
        //

        // Want to set cache_size > number of decision variables. While we have not
        // declared every decision variable yet (see impulse variables below), the
        // impulse variables do not enter into any dynamics evaluations, so we are
        // safe. Add a small factor (10%) just for safety margin.
       //因为有一些是根据mode设置的
        int cache_size = 1.1 * num_vars();
        cache_.push_back(std::make_unique<DynamicsCache<T>>(mode.evaluators(), cache_size));
        std::cout<<"mode0 cache-0 : "<<cache_.size()<<std::endl;//483
        // if(i_mode==0)
        // {
        //     std::cout<<"mode0 decision_variables-23 : "<<decision_variables().size()<<std::endl;//483
        //     std::cout<<"modex decision_variables-20 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
        //     std::cout<<"mode0 decision_variables-21 after add quat decision variable: to_string()"<<decision_variable(482).to_string()<<std::endl;//
        //     std::cout<<"mode0 decision_variables-22 after add quat decision variable: to_string()"<<decision_variable(481).to_string()<<std::endl;
        //     std::cout<<"mode0 constraints-9 GetAllConstraints().size() "<<GetAllConstraints().size()<<std::endl; //12
        //  }
        // else if(i_mode==1)
        // {
        //     std::cout<<"mode1 decision_variables-23 : "<<decision_variables().size()<<std::endl;//567
        //     std::cout<<"modex decision_variables-20 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
        //     std::cout<<"mode0 decision_variables-21 after add quat decision variable: to_string()"<<decision_variable(482).to_string()<<std::endl;//
        //     std::cout<<"mode0 decision_variables-22 after add quat decision variable: to_string()"<<decision_variable(481).to_string()<<std::endl;
        //     std::cout<<"mode1 constraints-9 GetAllConstraints().size() "<<GetAllConstraints().size()<<std::endl; //82
        // }
        //定义colloca constraints
        for (int j = 0; j < mode.num_knotpoints() - 1; j++) 
        {
          auto constraint = std::make_shared<DirconCollocationConstraint<T>>(plant_, mode.evaluators(), contexts_[i_mode].at(j).get(),
                                                                             contexts_[i_mode].at(j + 1).get(), i_mode, j, cache_[i_mode].get());
          std::cout<<"constraint.get_description()"<<constraint->get_description()<<std::endl;
          const std::unordered_map<int, double>& GetDynamicsScaless = mode.GetDynamicsScale();
          //std::cout<<"GetDynamicsScaless.size()1:"<<GetDynamicsScaless.size()<<std::endl;//always empty currently
          auto it = GetDynamicsScaless.begin();
          while(it!=GetDynamicsScaless.end())
          {
              std::cout<<"mode.GetDynamicsScale():"<<  it->first<<": "<<it->second<<std::endl;
              it++;
          }/**/
          constraint->SetConstraintScaling(mode.GetDynamicsScale());//always empty currently
          // std::cout<<"xxx:"<<timestep(mode_start_[i_mode] + j)<<std::endl;
          // std::cout<<"xxx:"<<state_vars(i_mode, j)<<std::endl;
          // std::cout<<"xxx:"<<state_vars(i_mode, j + 1)<<std::endl;
          // std::cout<<"xxx:"<<input_vars(i_mode, j)<<std::endl;
          // std::cout<<"xxx:"<<input_vars(i_mode, j + 1)<<std::endl;
          // std::cout<<"xxx:"<<force_vars(i_mode, j)<<std::endl;
          // std::cout<<"xxx:"<<force_vars(i_mode, j + 1)<<std::endl;
          //           std::cout<<"xxx:"<<force_vars_.size()<<std::endl;  
          // std::cout<<"xxx:"<<collocation_force_vars(i_mode, j)<<std::endl;
          // std::cout<<"xxx:"<<collocation_slack_vars_.size()<<std::endl;  
          // std::cout<<"xxx:"<<collocation_force_vars_.size()<<std::endl;  
          // auto ssdasdadd = collocation_slack_vars(i_mode, j);
          
          // std::cout<<"xxx:"<< collocation_slack_vars(i_mode, j)<<std::endl;               
          // std::cout<<"xxx:"<< quaternion_slack_vars(i_mode, j)<<std::endl;         

               
          AddConstraint(
              constraint,
              {timestep(mode_start_[i_mode] + j), state_vars(i_mode, j),
               state_vars(i_mode, j + 1), input_vars(i_mode, j),
               input_vars(i_mode, j + 1), force_vars(i_mode, j),
               force_vars(i_mode, j + 1), collocation_force_vars(i_mode, j),
               collocation_slack_vars(i_mode, j),
               quaternion_slack_vars(i_mode, j)}
               );
               
            //std::cout<<"mode.GetDynamicsScale():"<<  it->first<<": "<<it->second<<std::endl;
            if(i_mode == 0 )
            {
              std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode0dcc = GetAllConstraints();
              std::cout<<"mode0 constraints-10 setting position cosntraint.size(): "<<contraint_after_mode0dcc.size()<<std::endl;//21 
              std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[9].to_string()<<std::endl;
              std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[10].to_string()<<std::endl; 
              std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[11].to_string()<<std::endl; 
              //std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode0dcc[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
            }
            else if(i_mode == 1)
            {
                std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode1dcc = GetAllConstraints();
                std::cout<<"mode1 constraints-10 setting position cosntraint.size(): "<<contraint_after_mode1dcc.size()<<std::endl;//91
                if(j==0)//state_vars(i_mode=1, j=0)拼了一个新点,但是没算在规划中的decision variables
                std::cout<<"mode1 state_vars(i_mode, j + 1) "<<state_vars(i_mode, j)<<"--"<<state_vars(i_mode, j + 1)<<std::endl; 
                std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[9].to_string()<<std::endl;
                std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[10].to_string()<<std::endl; 
                std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[11].to_string()<<std::endl; 
                //std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode1dcc[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
            }            
        }

        // Create and add kinematic constraints
        for (int j = 0; j < mode.num_knotpoints(); j++) 
        {
          // Position constraints if type is All kAll==3全体类型
          if (mode.get_constraint_type(j) == KinematicConstraintType::kAll) 
          {
            VectorXd lb = VectorXd::Zero(mode.evaluators().count_active());//设置约束计算器中所有active方向的值的上下界为0
            VectorXd ub = VectorXd::Zero(mode.evaluators().count_active());
            if(mode.get_constraint_type(j) != KinematicConstraintType::kAll)
            std::cout<<"mode.get_constraint_type(j)11:"<<static_cast<int>(mode.get_constraint_type(j)) <<std::endl; 
            // If we are in the first knotpoint, do not constrain the position of
            // any constraints that overlap with the previous mode, as such a
            // constraint would be redundant. We do this here by setting bounds to
            // +/-inf.
             //第二个mode才有效，针对重合节点目前基本没用,对mode0和mode1都没用
            if (j == 0 && i_mode > 0) //本mode的终止节点是下一个mode的起始节点重合节点,此时不限制FindActiveIndicesUnion约束
            {
              std::vector<int> row_indices =get_mode(i_mode - 1).evaluators().FindActiveIndicesUnion(mode.evaluators());
              for (const int row_index : row_indices) 
              {
                lb(row_index) = -std::numeric_limits<double>::infinity();
                ub(row_index) = std::numeric_limits<double>::infinity();
              }
                std::cout<<"row_indices.size(): "<<row_indices.size()<<std::endl;// currently is none for both mode0 and mode1
                std::cout<<"mode.evaluators().size()"<<mode.evaluators().num_evaluators()<<std::endl;// currently is none
                std::cout<<"get_mode(i_mode - 1).evaluators().size()"<<get_mode(i_mode - 1).evaluators().num_evaluators()<<std::endl;
            }


            auto pos_constraint = std::make_shared<KinematicPositionConstraint<T>>(plant_, mode.evaluators(), lb, ub, mode.relative_constraints(),
                                                                                   contexts_[i_mode].at(j).get(),"kinematic_position[" + std::to_string(i_mode) + "][" +std::to_string(j) + "]");


            pos_constraint->SetConstraintScaling(mode.GetKinPositionScale());  //none currently
            AddConstraint(pos_constraint, {state_vars(i_mode, j).head(plant_.num_positions()),offset_vars(i_mode)});
            
            // if(i_mode == 0 )
            // {
            //     std::cout<<"mode0 constraints-12 mode.relative_constraints(): "<<mode.relative_constraints().size()<<std::endl;//21 
            //     std::cout<<"mode0 constraints-13 mode.relative_constraints(): "<<*(mode.relative_constraints().begin())<<std::endl;//21 
            //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_kinematic0 = GetAllConstraints();
            //     std::cout<<"mode0 constraints-12 setting position cosntraint.size(): "<<contraint_after_kinematic0.size()<<std::endl;//21 
            //     for(int k=0; k<contraint_after_kinematic0.size();k++)
            //     std::cout<<"mode0 constraints-13 contraint_position.to_string(): "<<contraint_after_kinematic0[k].to_string()<<std::endl;
            //     std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_kinematic0[10].to_string()<<std::endl; 
            //     std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_kinematic0[11].to_string()<<std::endl; 
            //     std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_kinematic0[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
            // }
            // else  if(i_mode == 1 )
            // {
            //     std::cout<<"mode0 constraints-12 mode.relative_constraints(): "<<mode.relative_constraints().size()<<std::endl;//21 
            //     std::cout<<"mode0 constraints-13 mode.relative_constraints(): "<<*(mode.relative_constraints().begin())<<std::endl;//21 
            //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_kinematic1 = GetAllConstraints();
            //     std::cout<<"mode1 constraints-12 setting position cosntraint.size(): "<<contraint_after_kinematic1.size()<<std::endl;//101 
            //     for(int k=0; k<contraint_after_kinematic1.size();k++)
            //     std::cout<<"mode0 constraints-13 contraint_position.to_string(): "<<contraint_after_kinematic1[k].to_string()<<std::endl;
            //     std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_kinematic1[10].to_string()<<std::endl; 
            //     std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_kinematic1[11].to_string()<<std::endl; 
            //     std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_kinematic1[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
            // }              
          }

          // Velocity constraints if type is not acceleration only. Also skip if
          // this is the first knotpoint following an impact-free mode transition.
          //速度约束
          if (mode.get_constraint_type(j) != KinematicConstraintType::kAccelOnly) 
          {
            // Skip if i_mode > 0 and j == 0 and no impact
            if (i_mode == 0 || j > 0 || is_impact) 
            {
                  auto vel_constraint =std::make_shared<KinematicVelocityConstraint<T>>( plant_, mode.evaluators(),
                                                                                        VectorXd::Zero(mode.evaluators().count_active()),VectorXd::Zero(mode.evaluators().count_active()),
                                                                                        contexts_[i_mode].at(j).get(),"kinematic_velocity[" + std::to_string(i_mode) + "][" + std::to_string(j) + "]");
                  vel_constraint->SetConstraintScaling(mode.GetKinVelocityScale());
                  AddConstraint(vel_constraint, state_vars(i_mode, j));
                //   if(i_mode == 0 )
                //   {
                //       std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode0dcc = GetAllConstraints();
                //       std::cout<<"mode0 constraints-13 setting velocity cosntraint.size(): "<<contraint_after_mode0dcc.size()<<std::endl;//21 
                //       std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[9].to_string()<<std::endl;
                //       std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[10].to_string()<<std::endl; 
                //       std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[11].to_string()<<std::endl; 
                //       std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode0dcc[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
                //   }
                //   else if(i_mode == 1 )
                //   {
                //       std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode1dcc = GetAllConstraints();
                //       std::cout<<"mode1 constraints-13 setting velocity cosntraint.size(): "<<contraint_after_mode1dcc.size()<<std::endl;//111
                //       std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[9].to_string()<<std::endl;
                //       std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[10].to_string()<<std::endl; 
                //       std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[11].to_string()<<std::endl; 
                //       std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode1dcc[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
                // }
            }
          }

          // Acceleration constraints (always)
          auto accel_constraint = std::make_shared<CachedAccelerationConstraint<T>>(plant_, mode.evaluators(), contexts_[i_mode].at(j).get(),
                                                                                    "kinematic_acceleration[" + std::to_string(i_mode) + "][" +std::to_string(j) + "]",
                                                                                    cache_[i_mode].get());
          accel_constraint->SetConstraintScaling(mode.GetKinAccelerationScale());
          AddConstraint(accel_constraint,{state_vars(i_mode, j), input_vars(i_mode, j),force_vars(i_mode, j)});
          // if(i_mode == 0 )
          // {
          //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode0dcc = GetAllConstraints();
          //     std::cout<<"mode0 constraints-14 setting acceleration cosntraint.size(): "<<contraint_after_mode0dcc.size()<<std::endl;//21 
          //     std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[9].to_string()<<std::endl;
          //     std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[10].to_string()<<std::endl; 
          //     std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode0dcc[11].to_string()<<std::endl; 
          //     std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode0dcc[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
          // }
          // else if(i_mode == 1 )
          // {
          //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode1dcc = GetAllConstraints();
          //     std::cout<<"mode1 constraints-14 setting acceleration cosntraint.size(): "<<contraint_after_mode1dcc.size()<<std::endl;//121 
          //     std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[9].to_string()<<std::endl;
          //     std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[10].to_string()<<std::endl; 
          //     std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode1dcc[11].to_string()<<std::endl; 
          //     std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode1dcc[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
          // }           
            if(i_mode==2)
            {
                  VectorXd lb = VectorXd::Zero(mode.relative_constraints().size());//设置约束计算器中所有active方向的值的上下界为0
                  VectorXd ub = VectorXd::Zero(mode.relative_constraints().size());
                  //offset_vars(mode_index)
                  auto foot_location_constraint = std::make_shared<foot_location_Constraint<T>>(plant_, mode.evaluators(), lb, ub, mode.relative_constraints(),
                                                                                  contexts_[i_mode].at(j).get(),"foot_location_Constraint[" + std::to_string(i_mode) + "]" );


                  //pos_constraint->SetConstraintScaling(mode.GetKinPositionScale());  //none currently
                  AddConstraint(foot_location_constraint, {offset_vars(i_mode-2),offset_vars(i_mode-1),offset_vars(i_mode)});
            }

//           //   //floatfoot 的位置约束

            VectorXd lb = VectorXd::Zero(3);
            VectorXd ub = VectorXd::Zero(3);
            if(j==0)
            {
              for (int i=0;i< 3;i++) 
              {
                lb(i) = -std::numeric_limits<double>::infinity(); 
                ub(i) = std::numeric_limits<double>::infinity();
              }
            }
          auto floatfoot_collosionavoidence_constraints = std::make_shared<dairlib::multibody::floatfoot_collosionavoidence_Constraint<T>>(plant_, mode.evaluators(), 
                                                                                lb, ub, mode.relative_constraints(),contexts_[i_mode].at(j).get(),
                                                                                "floatfoot_collosionavoidence_Constraint[" + std::to_string(i_mode) + "]", i_mode, j);

 std::cout<<floatfoot_collosionavoidence_rel_offset_vars(i_mode,j).size() <<std::endl;        
 std::cout<<state_vars(i_mode, j).head(plant_.num_positions()).size() <<std::endl;    
            AddConstraint(floatfoot_collosionavoidence_constraints, {state_vars(i_mode, j).head(plant_.num_positions()),
                                                                                                                                    floatfoot_collosionavoidence_rel_offset_vars(i_mode,j)});
            AddLinearConstraint(floatfoot_collosionavoidence_rel_offset_vars(i_mode,j)[2] <=-0.0001);                                                                                                                        
        }


        //
        // Create and add impact constraints
        //
        if (i_mode > 0)
         {
          int pre_impact_index = mode_length(i_mode - 1) - 1;//10-1=9
          if (is_impact) //通过两个mode的左右运动学计算器内容(约束)不一样判定是否产生碰撞
          {
             impulse_vars_.push_back(NewContinuousVariables(mode.evaluators().count_full(), "impulse[" + std::to_string(i_mode) + "]"));
            //impulse_vars_.push_back(NewContinuousVariables(mode.evaluators().get_evaluator(0).num_full(), "impulse[" + std::to_string(i_mode) + "]"));
            // if(i_mode==1)
            // {  
            //     std::cout<<"mode1 decision_variables-?? : "<<decision_variables().size()<<std::endl;//483
            //     std::cout<<"mode1 decision_variables-20 decision_variables() "<<impulse_vars_[0]<<std::endl;//每个变量都在
            //     std::cout<<"mode1 decision_variables-20 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
            //     std::cout<<"mode0 decision_variables-21 after add quat decision variable: to_string()"<<decision_variable(482).to_string()<<std::endl;//
            //     std::cout<<"mode0 decision_variables-22 after add quat decision variable: to_string()"<<decision_variable(481).to_string()<<std::endl;
            //     std::cout<<"mode0 constraints-9 GetAllConstraints().size() "<<GetAllConstraints().size()<<std::endl; //12
            // }
  
            // Use pre-impact context
            auto impact_constraint = std::make_shared<ImpactConstraint<T>>(plant_, mode.evaluators(), contexts_[i_mode - 1].back().get(),
                                                                           "impact[" + std::to_string(i_mode) + "]");
            impact_constraint->SetConstraintScaling(mode.GetImpactScale());

            AddConstraint(impact_constraint,{state_vars(i_mode - 1, pre_impact_index), impulse_vars(i_mode - 1),post_impact_velocity_vars(i_mode - 1)});
            // if(i_mode == 1 )
            // {
            //     std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_impact = GetAllConstraints();
            //     std::cout<<"mode1 constraints-?? setting contraint_after_impact cosntraint.size(): "<<contraint_after_impact.size()<<std::endl;//122 
            //     std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_impact[9].to_string()<<std::endl;
            //     std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_impact[10].to_string()<<std::endl; 
            //     std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_impact[11].to_string()<<std::endl; 
            //     std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_impact[12].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
            // }        
          }
          else 
          {
            // Add empty decision variables
            impulse_vars_.push_back(NewContinuousVariables(0, ""));
            if(i_mode==1)
            {  
                std::cout<<"mode1 decision_variables-!!: "<<decision_variables().size()<<std::endl;//483
                std::cout<<"modex decision_variables-20 decision_variables() "<<decision_variables()<<std::endl;//每个变量都在
                std::cout<<"mode0 decision_variables-21 after add quat decision variable: to_string()"<<decision_variable(482).to_string()<<std::endl;//
                std::cout<<"mode0 decision_variables-22 after add quat decision variable: to_string()"<<decision_variable(481).to_string()<<std::endl;
                std::cout<<"mode0 constraints-9 GetAllConstraints().size() "<<GetAllConstraints().size()<<std::endl; //12
              }
            // Linear equality constraint on velocity variables
            auto pre_impact_velocity = state_vars(i_mode - 1, pre_impact_index).tail(plant_.num_velocities());
            AddLinearConstraint(pre_impact_velocity ==post_impact_velocity_vars(i_mode - 1));
              if(i_mode == 1 ) 
              {
                 std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_impact = GetAllConstraints();
                 std::cout<<"mode1 constraints-!! setting contraint_after_impact cosntraint.size(): "<<contraint_after_impact.size()<<std::endl;//121 
                 std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_impact[9].to_string()<<std::endl;
                 std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_impact[10].to_string()<<std::endl; 
                 std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_impact[11].to_string()<<std::endl; 
                 std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_impact[12].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
              }        
          }
         }

        //
        // Create and add quaternion constraints
        //目前没有用到 MODE0 mode1 都没有用到
        auto quaternion_constraint = std::make_shared<QuaternionConstraint<T>>();
        for (int j = 0; j < mode.num_knotpoints(); j++) 
        {
          //std::cout<<"sdddsdsdssds"<<mode.IsSkipQuaternionConstraint(j)<<std::endl;  
          if (!mode.IsSkipQuaternionConstraint(j)) 
          {
            auto start_indices = multibody::QuaternionStartIndices(plant_);
            for (auto start_index : start_indices)
            {
              std::cout<<"sdddsdsdssds quaternion_constraint出现验证"<<std::endl;
              AddConstraint(quaternion_constraint,state_vars(i_mode, j).segment(start_index, 4));
            }  
          }
        }
//

        ///
        /// Add friction cone constraints to force variables
        ///
        /// TODO: hard-coding number of frictional faces, but this could be an
        /// option, along with a choice of which friction cone constraint to use.
        int num_faces = 4;
        for (int k = 0; k < mode.evaluators().num_evaluators(); k++) 
        {
            //std::cout<<"mode.evaluators().num_evaluators(): "<<mode.evaluators().num_evaluators()<<std::endl;//1 
          const auto& e = mode.evaluators().get_evaluator(k);
            //CreateLinearFrictionConstraints的调用利用了多态,两个分别是力的限位和摩擦约束
          auto force_constraints_vec = e.CreateLinearFrictionConstraints(num_faces);//force_constraints_vec.size()=2 at mode 0, the  type is srd::vector<shared_ptr<Constraint>> constraints;
          for (auto force_constraint : force_constraints_vec)
          {
            // Add to knot point forces
            for (int j = 0; j < mode.num_knotpoints(); j++) 
            {
              AddConstraint(force_constraint,force_vars(i_mode, j).segment(mode.evaluators().evaluator_full_start(k), e.num_full()));
            //   if(i_mode == 0 )
            //   {
            //       std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode0friction = GetAllConstraints();
            //       std::cout<<"mode0 constraints-15 setting linear friction cosntraint.size(): "<<contraint_after_mode0friction.size()<<std::endl;//71 
            //       std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode0friction[9].to_string()<<std::endl;
            //       std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode0friction[10].to_string()<<std::endl; 
            //       std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode0friction[11].to_string()<<std::endl; 
            //       std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode0friction[12+j].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
            //   }
            //  else if(i_mode == 1 )
            //   {
            //       std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode1friction = GetAllConstraints();
            //       std::cout<<"mode1 constraints-15 setting linear friction cosntraint.size(): "<<contraint_after_mode1friction.size()<<std::endl;//71 
            //       std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode1friction[9].to_string()<<std::endl;
            //       std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode1friction[10].to_string()<<std::endl; 
            //       std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode1friction[11].to_string()<<std::endl; 
            //       std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode1friction[12].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
            //   }                
              std::cout<<"force_vars.size()"<<force_vars(i_mode, j).size()<<std::endl;
            }
            if (i_mode > 0) 
            {
              // Add to impulse variables
              AddConstraint(force_constraint,impulse_vars(i_mode - 1).segment(mode.evaluators().evaluator_full_start(k),e.num_full()));
              if(i_mode == 1 )
              {
              std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_after_mode1friction = GetAllConstraints();
              std::cout<<"mode0 constraints@@ setting impulse linear friction cosntraint.size(): "<<contraint_after_mode1friction.size()<<std::endl;//21 
              std::cout<<"mode0 constraints-6 contraint_before_quat.to_string(): "<<contraint_after_mode1friction[9].to_string()<<std::endl;
              std::cout<<"mode0 constraints-7 contraint_before_quat.to_string(): "<<contraint_after_mode1friction[10].to_string()<<std::endl; 
              std::cout<<"mode0 constraints-8 contraint_before_quat.to_string(): "<<contraint_after_mode1friction[11].to_string()<<std::endl; 
              std::cout<<"mode0 constraints-11 the setting dcc cosntraint "<<contraint_after_mode1friction[12].to_string()<<std::endl; //each dcc constraints and constraint type is bounding
                }            
            }
          }
        }

          //std::cout<<"i_modei_modei_modei_mode: "<<mode.get_force_regularization()<<std::endl; 
        if (mode.get_force_regularization() != 0)//double mode.get_force_regularization() = 1e-10 both mode0 and mode1
        {
             std::cout<<"i_modei_modei_modei_modexxx   ss: "<<mode.get_force_regularization()<<std::endl; 
          // Add regularization cost on force
          {
            int size = force_vars_.at(i_mode).size();
            AddQuadraticCost(mode.get_force_regularization() * MatrixXd::Identity(size, size),VectorXd::Zero(size), force_vars_.at(i_mode));
            // if(i_mode == 0)
            // {
            //     std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_force_regularization = GetAllCosts();
            //     std::cout<<"mode0 costs-2 num of cost after force_regularization: "<<cost_after_force_regularization.size()<<std::endl;   
            //     std::cout<<"mode0 costs-3 num of cost after iforce_regularization: "<<std::endl;   
            //     for(int i=0; i<cost_after_force_regularization.size(); i++)
            //     {
            //        std::cout<<"cost_after force_regularization.to_string(): "<<cost_after_force_regularization[i].to_string()<<std::endl;//
            //         std::cout<<"cost_after force_regularization.variables(): "<<cost_after_force_regularization[i].variables()<<std::endl; 
            //         std::cout<<"cost_after force_regularization.GetNumElements(): "<<cost_after_force_regularization[i].GetNumElements()<<std::endl;     
            //    }
            //  }
            // else if(i_mode == 1)
            // {
            //    std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_force_regularization = GetAllCosts();
            //    std::cout<<"mode1 costs-2 num of cost after force_regularization: "<<cost_after_force_regularization.size()<<std::endl;   
            //    std::cout<<"mode1 costs-3 num of cost after iforce_regularization: "<<std::endl;   
            //    for(int i=0; i<cost_after_force_regularization.size(); i++)
            //     {
            //        std::cout<<"cost_after force_regularization.to_string(): "<<cost_after_force_regularization[i].to_string()<<std::endl;//
            //        std::cout<<"cost_after force_regularization.variables(): "<<cost_after_force_regularization[i].variables()<<std::endl; 
            //        std::cout<<"cost_after force_regularization.GetNumElements(): "<<cost_after_force_regularization[i].GetNumElements()<<std::endl;     
            //     }
            // }             
          }

          {
            int size = collocation_force_vars_.at(i_mode).size();
            AddQuadraticCost(mode.get_force_regularization() * MatrixXd::Identity(size, size),VectorXd::Zero(size), collocation_force_vars_.at(i_mode));
          //   if(i_mode == 0)
          //   {
          //       std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_collocation_force_regularization = GetAllCosts();
          //       std::cout<<"mode0 costs-4 num of cost after cost_after_collocation_force_regularization: "<<cost_after_collocation_force_regularization.size()<<std::endl;   
          //      std::cout<<"mode0 costs-5 num of cost after cost_after_collocation_force_regularization: "<<std::endl;   
          //       for(int i=0; i<cost_after_collocation_force_regularization.size(); i++)
          //      {
          //           std::cout<<"cost_after collocation_force_regularization.to_string(): "<<cost_after_collocation_force_regularization[i].to_string()<<std::endl;//
          //           std::cout<<"cost_after collocation_force_regularization.variables(): "<<cost_after_collocation_force_regularization[i].variables()<<std::endl; 
          //           std::cout<<"cost_after collocation_force_regularization.GetNumElements(): "<<cost_after_collocation_force_regularization[i].GetNumElements()<<std::endl;     
          //       }
          //  }
          //   else if(i_mode == 1)
          //   {
          //       std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_collocation_force_regularization = GetAllCosts();
          //       std::cout<<"mode1 costs-4 num of cost after cost_after_collocation_force_regularization: "<<cost_after_collocation_force_regularization.size()<<std::endl;   
          //       std::cout<<"mode1 costs-5 num of cost after cost_after_collocation_force_regularization: "<<std::endl;   
          //      for(int i=0; i<cost_after_collocation_force_regularization.size(); i++)
          //       {
          //           std::cout<<"cost_after collocation_force_regularization.to_string(): "<<cost_after_collocation_force_regularization[i].to_string()<<std::endl;//
          //           std::cout<<"cost_after collocation_force_regularization.variables(): "<<cost_after_collocation_force_regularization[i].variables()<<std::endl; 
          //            std::cout<<"cost_after collocation_force_regularization.GetNumElements(): "<<cost_after_collocation_force_regularization[i].GetNumElements()<<std::endl;     
          //      }
          //  }                 
          }

          {
            int size = collocation_slack_vars_.at(i_mode).size();
            AddQuadraticCost(mode.get_force_regularization() * MatrixXd::Identity(size, size),VectorXd::Zero(size), collocation_slack_vars_.at(i_mode));
          //   if(i_mode == 0)
          //  {
          //        std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_collocation_slack_vars_regularization = GetAllCosts();
          //        std::cout<<"mode0 costs-6 num of cost after cost_after_collocation_slack_vars_regularization: "<<cost_after_collocation_slack_vars_regularization.size()<<std::endl;   
          //        std::cout<<"mode0 costs-7 num of cost after cost_after_collocation_slack_vars_regularization: "<<std::endl;   
          //       for(int i=0; i<cost_after_collocation_slack_vars_regularization.size(); i++)
          //       {
          //            std::cout<<"cost_after collocation_slack_vars_regularization.to_string(): "<<cost_after_collocation_slack_vars_regularization[i].to_string()<<std::endl;//
          //            std::cout<<"cost_after collocation_slack_vars_regularization.variables(): "<<cost_after_collocation_slack_vars_regularization[i].variables()<<std::endl; 
          //            std::cout<<"cost_after collocation_slack_vars_regularization.GetNumElements(): "<<cost_after_collocation_slack_vars_regularization[i].GetNumElements()<<std::endl;     
          //       }
          //   }   
          //   else if(i_mode == 1)
          //   {
          //       std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_collocation_slack_vars_regularization = GetAllCosts();
          //       std::cout<<"mode1 costs-6 num of cost after cost_after_collocation_slack_vars_regularization: "<<cost_after_collocation_slack_vars_regularization.size()<<std::endl;   
          //       std::cout<<"mode1 costs-7 num of cost after cost_after_collocation_slack_vars_regularization: "<<std::endl;   
          //       for(int i=0; i<cost_after_collocation_slack_vars_regularization.size(); i++)
          //      {
          //          std::cout<<"cost_after collocation_slack_vars_regularization.to_string(): "<<cost_after_collocation_slack_vars_regularization[i].to_string()<<std::endl;//
          //          std::cout<<"cost_after collocation_slack_vars_regularization.variables(): "<<cost_after_collocation_slack_vars_regularization[i].variables()<<std::endl; 
          //          std::cout<<"cost_after collocation_slack_vars_regularization.GetNumElements(): "<<cost_after_collocation_slack_vars_regularization[i].GetNumElements()<<std::endl;     
          //      }
          //   }              
          }

          {
            int size = quaternion_slack_vars_.at(i_mode).size();//空代价函数，但是存在
              std::cout<<"modex costs-xx num of cost after cost_after_quaternion_slack_vars_regularization: "<<size<<std::endl;    //0
            AddQuadraticCost(mode.get_force_regularization() * MatrixXd::Identity(size, size),VectorXd::Zero(size), quaternion_slack_vars_.at(i_mode));
          //  if(i_mode == 0)
          //  {
          //       std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_quaternion_slack_vars_regularization = GetAllCosts();
          //       std::cout<<"mode0 costs-7 num of cost after cost_after_quaternion_slack_vars_regularization: "<<cost_after_quaternion_slack_vars_regularization.size()<<std::endl;   
          //       std::cout<<"mode0 costs-8 num of cost after cost_after_quaternion_slack_vars_regularization: "<<std::endl;   
          //      for(int i=0; i<cost_after_quaternion_slack_vars_regularization.size(); i++)
          //      {
          //          std::cout<<"cost_after quaternion_regularization.to_string(): "<<cost_after_quaternion_slack_vars_regularization[i].to_string()<<std::endl;//
          //          std::cout<<"cost_after quaternion_regularization.variables(): "<<cost_after_quaternion_slack_vars_regularization[i].variables()<<std::endl; 
          //          std::cout<<"cost_after quaternion_regularization.GetNumElements(): "<<cost_after_quaternion_slack_vars_regularization[i].GetNumElements()<<std::endl;     
          //      }
          //    }
          //   else if(i_mode == 1)
          //    {
          //       std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_quaternion_slack_vars_regularization = GetAllCosts();
          //       std::cout<<"mode1 costs-7 num of cost after cost_after_quaternion_slack_vars_regularization: "<<cost_after_quaternion_slack_vars_regularization.size()<<std::endl;   
          //       std::cout<<"mode1 costs-8 num of cost after cost_after_quaternion_slack_vars_regularization: "<<std::endl;   
          //      for(int i=0; i<cost_after_quaternion_slack_vars_regularization.size(); i++)
          //       {
          //           std::cout<<"cost_after quaternion_regularization.to_string(): "<<cost_after_quaternion_slack_vars_regularization[i].to_string()<<std::endl;//
          //           std::cout<<"cost_after quaternion_regularization.variables(): "<<cost_after_quaternion_slack_vars_regularization[i].variables()<<std::endl; 
          //           std::cout<<"cost_after quaternion_regularization.GetNumElements(): "<<cost_after_quaternion_slack_vars_regularization[i].GetNumElements()<<std::endl;     
          //      }
          //  }              
          }
        }
          if(i_mode==0)
          {
              std::cout<<"mode0 decision_variables.size(): "<<decision_variables().size()<<std::endl;//
              std::cout<<"mode0 decision_variables:\n"<<decision_variables()<<std::endl;//每个变量都在

              std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_of_mode0 = GetAllConstraints();
              std::cout<<"mode0 total constraints: "<<contraint_of_mode0.size()<<std::endl;
              for(int i=0; i<contraint_of_mode0.size(); i++)
              {
                  std::cout<<"mode0 total constraints.to_string(): "<<contraint_of_mode0[i].to_string()<<std::endl;
                  std::cout<<std::endl;    
              }
              
                std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_mode0 = GetAllCosts();
                std::cout<<"mode0 costs cost_after_mode0: "<<cost_after_mode0.size()<<std::endl;   
                std::cout<<"mode0 costs cost_after_mode0: "<<std::endl;   
                for(int i=0; i<cost_after_mode0.size(); i++)
                {
                    std::cout<<"cost_after_mode0.to_string(): "<<cost_after_mode0[i].to_string()<<std::endl;//
                    std::cout<<"cost_after_mode0.variables(): "<<cost_after_mode0[i].variables()<<std::endl; 
                    std::cout<<"cost_after_mode0.GetNumElements(): "<<cost_after_mode0[i].GetNumElements()<<std::endl;     
                    std::cout<<std::endl;    
                }
          }   
          else if(i_mode==1)
          {
              std::cout<<"mode1 decision_variables.size(): "<<decision_variables().size()<<std::endl;//
              std::cout<<"mode1 decision_variables:\n"<<decision_variables()<<std::endl;//每个变量都在
              std::cout<<"mode1 decision_variables.to_string()"<<decision_variables()<<std::endl;

              std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_of_mode0 = GetAllConstraints();
              std::cout<<"mode1 total constraints: "<<contraint_of_mode0.size()<<std::endl;
              for(int i=0; i<contraint_of_mode0.size(); i++)
              {
                  std::cout<<"mode1 total constraints.to_string(): "<<contraint_of_mode0[i].to_string()<<std::endl;
                  std::cout<<std::endl;    
              }
              
                std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_mode0 = GetAllCosts();
                std::cout<<"mode1 costs cost_after_mode1: "<<cost_after_mode0.size()<<std::endl;   
                std::cout<<"mode1 costs cost_after_mode1: "<<std::endl;   
                for(int i=0; i<cost_after_mode0.size(); i++)
                {
                    std::cout<<"cost_after_mode1.to_string(): "<<cost_after_mode0[i].to_string()<<std::endl;//
                    std::cout<<"cost_after_mode1.variables(): "<<cost_after_mode0[i].variables()<<std::endl; 
                    std::cout<<"cost_after_mode1.GetNumElements(): "<<cost_after_mode0[i].GetNumElements()<<std::endl;     
                    std::cout<<std::endl;    
                }
          }   
          else if(i_mode==2)
          {
              std::cout<<"mode2 decision_variables.size(): "<<decision_variables().size()<<std::endl;//
              std::cout<<"mode2 decision_variables:\n"<<decision_variables()<<std::endl;//每个变量都在
              std::cout<<"mode2 decision_variables.to_string()"<<decision_variables()<<std::endl;

              std::vector<drake::solvers::Binding<drake::solvers::Constraint> > contraint_of_mode0 = GetAllConstraints();
              std::cout<<"mode2 total constraints: "<<contraint_of_mode0.size()<<std::endl;
              for(int i=0; i<contraint_of_mode0.size(); i++)
              {
                  std::cout<<"mode2 total constraints.to_string(): "<<contraint_of_mode0[i].to_string()<<std::endl;
                  std::cout<<std::endl;    
              }
              
                std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_after_mode0 = GetAllCosts();
                std::cout<<"mode2 costs cost_after_mode2: "<<cost_after_mode0.size()<<std::endl;   
                std::cout<<"mode2 costs cost_after_mode2: "<<std::endl;   
                for(int i=0; i<cost_after_mode0.size(); i++)
                {
                    std::cout<<"cost_after_mode2.to_string(): "<<cost_after_mode0[i].to_string()<<std::endl;//
                    std::cout<<"cost_after_mode2.variables(): "<<cost_after_mode0[i].variables()<<std::endl; 
                    std::cout<<"cost_after_mode2.GetNumElements(): "<<cost_after_mode0[i].GetNumElements()<<std::endl; 
                    std::cout<<std::endl;    
                }
          }               
      }
}

///
/// Getters for decision variables
///lambda
template <typename T>
const VectorXDecisionVariable Dircon<T>::force_vars(int mode_index,
                                                    int knotpoint_index) const {
  const auto& mode = get_mode(mode_index);
  return force_vars_.at(mode_index)
      .segment(knotpoint_index * mode.evaluators().count_full(),
               mode.evaluators().count_full());
  // return force_vars_.at(mode_index)
  //     .segment(knotpoint_index * mode.evaluators().get_evaluator(0).num_full(),
  //              mode.evaluators().get_evaluator(0).num_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::collocation_force_vars(
    int mode_index, int collocation_index) const {
  const auto& mode = get_mode(mode_index);
  return collocation_force_vars_.at(mode_index)
      .segment(collocation_index * mode.evaluators().count_full(),
               mode.evaluators().count_full());
  // return collocation_force_vars_.at(mode_index)
  //     .segment(collocation_index * mode.evaluators().get_evaluator(0).num_full(),
  //              mode.evaluators().get_evaluator(0).num_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::collocation_slack_vars(
    int mode_index, int collocation_index) const {
  const auto& mode = get_mode(mode_index);
  return collocation_slack_vars_.at(mode_index)
      .segment(collocation_index * mode.evaluators().count_full(),
               mode.evaluators().count_full());
  // return collocation_slack_vars_.at(mode_index)
  //     .segment(collocation_index * mode.evaluators().get_evaluator(0).num_full(),
  //              mode.evaluators().get_evaluator(0).num_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::state_vars(int mode_index,
                                                    int knotpoint_index) const {
  // If first knot of a mode after the first, use post impact velocity variables
    //std::cout<<"x_vars()"<<x_vars()<<std::endl;
    //std::cout<<"x_vars()"<<decision_variables()<<std::endl;
    //std::cout<<"x_vars()"<<x_vars()<<std::endl;
  if (knotpoint_index == 0 && mode_index > 0) {
      //std::cout<<"modecccc decision_variables.size(): "<<decision_variables().size()<<std::endl;//
    VectorXDecisionVariable ret(plant_.num_positions() +
                                plant_.num_velocities());
    ret << x_vars().segment(mode_start_[mode_index] * (plant_.num_positions() +
                                                       plant_.num_velocities()), plant_.num_positions()),post_impact_velocity_vars(mode_index - 1);
      //std::cout<<"modevvvv decision_variables.size(): "<<decision_variables().size()<<std::endl;//
    return ret;
  } else {
    return x_vars().segment(
        (mode_start_[mode_index] + knotpoint_index) *
            (plant_.num_positions() + plant_.num_velocities()),
        plant_.num_positions() + plant_.num_velocities());
  }
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::input_vars(int mode_index,
                                                    int knotpoint_index) const {
  return u_vars().segment(
      (mode_start_[mode_index] + knotpoint_index) * plant_.num_actuators(),
      plant_.num_actuators());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::quaternion_slack_vars(
    int mode_index, int collocation_index) const {
  int num_quat = multibody::QuaternionStartIndices(plant_).size();
  return quaternion_slack_vars_.at(mode_index)
      .segment(num_quat * collocation_index, num_quat);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::offset_vars(int mode_index) const {
  return offset_vars_.at(mode_index);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::floatfoot_collosionavoidence_rel_offset_vars(int mode_index, int knot_index) const 
{
      VectorXDecisionVariable ret(3);
      ret <<floatfoot_collosionavoidence_rel_offset_vars_.at(mode_start_[mode_index] +knot_index)[0],
                  floatfoot_collosionavoidence_rel_offset_vars_.at(mode_start_[mode_index] +knot_index)[1],
                  floatfoot_collosionavoidence_rel_offset_vars_.at(mode_start_[mode_index] +knot_index)[2];
      return ret;
}


template <typename T>
const VectorXDecisionVariable Dircon<T>::post_impact_velocity_vars(
    int mode_transition_index) const {
  return v_post_impact_vars_.at(mode_transition_index);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::impulse_vars(
    int mode_transition_index) const {
  return impulse_vars_.at(mode_transition_index);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file, std::vector<unsigned int> poses_per_mode,double alpha, std::string weld_frame_to_world) //poses_per_mode 每个mode所展现的位姿
{
  //std::cout<<"(uint)num_modes():"<<(uint)num_modes()<<std::endl;
  //std::cout<<"poses_per_mode.size() :"<<poses_per_mode.size() <<std::endl;
  DRAKE_DEMAND(!callback_visualizer_);  // Cannot be set twice
  DRAKE_DEMAND(poses_per_mode.size() == (uint)num_modes());

  // Count number of total poses, start and finish of every mode
  int num_poses = num_modes() + 1;
  for (int i = 0; i < num_modes(); i++) {
    DRAKE_DEMAND(poses_per_mode.at(i) == 0 ||
                 (poses_per_mode.at(i) + 2 <= (uint)mode_length(i)));
    num_poses += poses_per_mode.at(i);
  }

  // Assemble variable list
  drake::solvers::VectorXDecisionVariable vars(num_poses *
                                               plant_.num_positions());

  int index = 0;
  for (int i = 0; i < num_modes(); i++) {
    // Set variable block, extracting positions only
    vars.segment(index * plant_.num_positions(), plant_.num_positions()) =
        state_vars(i, 0).head(plant_.num_positions());
    index++;

    for (uint j = 0; j < poses_per_mode.at(i); j++) {
      // The jth element in mode i, counting in between the start/end poses
      int modei_index =
          (j + 1) * (mode_length(i) - 1) / (poses_per_mode.at(i) + 1);

      vars.segment(index * plant_.num_positions(), plant_.num_positions()) =
          state_vars(i, modei_index).head(plant_.num_positions());
      index++;
    }
  }

  // Final state
  auto last_mode = get_mode(num_modes() - 1);
  vars.segment(index * plant_.num_positions(), plant_.num_positions()) =
      state_vars(num_modes() - 1, last_mode.num_knotpoints() - 1)
          .head(plant_.num_positions());

  VectorXd alpha_vec = VectorXd::Constant(num_poses, alpha);
  alpha_vec(0) = 1;
  alpha_vec(num_poses - 1) = 1;

  // Create visualizer
  callback_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
      model_file, num_poses, alpha_vec, weld_frame_to_world);

  // Callback lambda function
  auto my_callback = [this, num_poses](const Eigen::Ref<const VectorXd>& vars) {
    VectorXd vars_copy = vars;
    Eigen::Map<MatrixXd> states(vars_copy.data(), this->plant_.num_positions(),
                                num_poses);

    this->callback_visualizer_->DrawPoses(states);
  };

  AddVisualizationCallback(my_callback, vars);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
                                            unsigned int num_poses,
                                            double alpha,
                                            std::string weld_frame_to_world) {
  // Check that there is an appropriate number of poses
  DRAKE_DEMAND(num_poses >= (uint)num_modes() + 1);
  DRAKE_DEMAND(num_poses <= (uint)N());

  // sum of mode lengths, excluding ends
  int mode_sum = 0;
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    auto mode = get_mode(i_mode);
    if (mode.num_knotpoints() > 2) {
      mode_sum += mode.num_knotpoints() - 2;
    }
  }

  // Split up num_poses per modes, rounding down
  // If NP is the total number of  poses to visualize, excluding ends (interior)
  //   S is mode_sum, the total number of the interior knot points
  //   and N the number of interior knot points for a particular mode, then
  //
  //   poses = (NP * N )/S
  unsigned int num_poses_without_ends = num_poses - num_modes() - 1;
  unsigned int assigned_sum = 0;
  std::vector<unsigned int> num_poses_per_mode;
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    if (mode_length(i_mode) > 2) {
      unsigned int mode_count =
          (num_poses_without_ends * (mode_length(i_mode) - 2)) / mode_sum;
      num_poses_per_mode.push_back(mode_count);
      assigned_sum += mode_count;
    } else {
      num_poses_per_mode.push_back(0);
    }
  }

  // Need to add back in the fractional bits, using the largest fractions
  while (assigned_sum < num_poses_without_ends) {
    int largest_mode = -1;
    double value = 0;
    for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
      double fractional_value =
          num_poses_without_ends * (mode_length(i_mode) - 2) -
          num_poses_per_mode.at(i_mode) * mode_sum;

      if (fractional_value > value) {
        value = fractional_value;
        largest_mode = i_mode;
      }
    }

    num_poses_per_mode.at(largest_mode)++;
    assigned_sum++;
  }

  CreateVisualizationCallback(model_file, num_poses_per_mode, alpha,
                              weld_frame_to_world);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
                                            double alpha,
                                            std::string weld_frame_to_world) {
  CreateVisualizationCallback(model_file, N(), alpha, weld_frame_to_world);
}

template <typename T>
VectorX<Expression> Dircon<T>::SubstitutePlaceholderVariables(
    const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) =
        MultipleShooting::SubstitutePlaceholderVariables(f(i), interval_index);
  }
  return ret;
}

// TODO: need to configure this to handle the hybrid discontinuities properly
template <typename T>
void Dircon<T>::DoAddRunningCost(const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  // Here, we add the cost using symbolic expression. The expression is a
  // polynomial of degree 3 which Drake can handle, although the
  // documentation says it only supports up to second order.

  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, 0) * h_vars()(0) /
          2);
  for (int i = 1; i <= N() - 2; i++) {
    AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, i) *
            (h_vars()(i - 1) + h_vars()(i)) / 2);
  }
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, N() - 1) *
          h_vars()(N() - 2) / 2);
}

template <typename T>
void Dircon<T>::GetStateAndDerivativeSamples(
    const drake::solvers::MathematicalProgramResult& result,
    std::vector<Eigen::MatrixXd>* state_samples,
    std::vector<Eigen::MatrixXd>* derivative_samples,
    std::vector<Eigen::VectorXd>* state_breaks) const 
{
  DRAKE_ASSERT(state_samples->empty());
  DRAKE_ASSERT(derivative_samples->empty());
  DRAKE_ASSERT(state_breaks->empty());
 //std::cout << "a.1"  <<std::endl;
  VectorXd times(GetSampleTimes(result));
 //std::cout << "a.2"  <<std::endl;
    //std::cout << "num_modes:"  <<num_modes()<<std::endl;
  for (int mode = 0; mode < num_modes(); mode++) {
   // std::cout << "mode:"  <<mode<<std::endl;
    MatrixXd states_i(num_states(), mode_length(mode));
      std::cout << "num_states():"  <<num_states()<<std::endl;
      std::cout << "mode_length():"  <<mode_length(mode)<<std::endl;
      ////hereherehere problem
    MatrixXd derivatives_i(num_states(), mode_length(mode));//12*10
      //std::cout << "a.3"  <<std::endl;
    VectorXd times_i(mode_length(mode));
      //std::cout << "a.4"  <<std::endl;
    for (int j = 0; j < mode_length(mode); j++) {
      int k = mode_start_[mode] + j;
        std::cout << "j"  <<j<<std::endl;
     //std::cout << "a.4.1"  <<std::endl;
      VectorX<T> xk = result.GetSolution(state_vars(mode, j));
          //std::cout << "a.4.2"  <<std::endl;
      VectorX<T> uk = result.GetSolution(input_vars(mode, j));
        //std::cout << "a.4.3"  <<std::endl;
      auto context = multibody::createContext<T>(plant_, xk, uk);
        //std::cout << "a.4.4"  <<std::endl;

      states_i.col(j) = drake::math::DiscardGradient(xk);
        //std::cout << "a.4.5"  <<std::endl;
      auto xdot = get_mode(mode).evaluators().CalcTimeDerivativesWithForce(
        context.get(), result.GetSolution(force_vars(mode, j)));
        //std::cout << "a.4.6"  <<std::endl;
        //std::cout << "xdot:"  <<xdot<<std::endl;
        std::cout << "k:"  <<k<<std::endl;
        derivatives_i.col(j) = drake::math::DiscardGradient(xdot);//k这里k换成了j要不然矩阵越界，报错
        //auto temp=drake::math::DiscardGradient(xdot);
        //derivatives_i.col(k) =temp;
       // std::cout << "DiscardGradient(xdot):"  << drake::math::DiscardGradient(xdot)<<std::endl;
        /*int new_k;
        new_k=int(k/2);
        if(new_k<10)
        {
            derivatives_i.col(new_k) = drake::math::DiscardGradient(xdot);
        }
      else if(new_k>=10)
      {
      derivatives_i.col(9) = drake::math::DiscardGradient(xdot);
      }*/
      int r = derivatives_i.rows();
      int c = derivatives_i.cols();
      //std::cout << "rows(cols):"  <<r <<c<<std::endl;
        
        //std::cout << "a.4.7"  <<std::endl;
      times_i(j) = times(k);
        //std::cout << "a.4.8"  <<std::endl;
    }
      //std::cout << "a.5"  <<std::endl;
    state_samples->push_back(states_i);
      //std::cout << "a.6"  <<std::endl;
    derivative_samples->push_back(derivatives_i);
      //std::cout << "a.7"  <<std::endl;
    state_breaks->push_back(times_i);
      //std::cout << "a.8"  <<std::endl;
  }
}

template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructInputTrajectory(
    const MathematicalProgramResult& result) const {
  return PiecewisePolynomial<double>::FirstOrderHold(GetSampleTimes(result),
                                                     GetInputSamples(result));
}

template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructStateTrajectory(
    const MathematicalProgramResult& result) const {
  std::vector<MatrixXd> states;
  std::vector<MatrixXd> derivatives;
  std::vector<VectorXd> times;
   // std::cout << "a"  <<std::endl;
  GetStateAndDerivativeSamples(result, &states, &derivatives, &times);
     //std::cout << "b"  <<std::endl;
  PiecewisePolynomial<double> state_traj =PiecewisePolynomial<double>::CubicHermite(times[0], states[0], derivatives[0]);
      //std::cout << "c"  <<std::endl;
  for (int mode = 1; mode < num_modes(); ++mode) {
    // Cannot form trajectory with only a single break
    if (mode_length(mode) < 2) {
      continue;
    }
    state_traj.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(times[mode], states[mode], derivatives[mode]));
  }
    // std::cout << "d"  <<std::endl;
  return state_traj;
}

template <typename T>
void Dircon<T>::SetInitialForceTrajectory(
    int mode_index, const PiecewisePolynomial<double>& traj_init_l,
    const PiecewisePolynomial<double>& traj_init_lc,
    const PiecewisePolynomial<double>& traj_init_vc) {
  const auto& mode = get_mode(mode_index);
  double start_time = 0;
  double h;
  if (timesteps_are_decision_variables())
        {
    h = GetInitialGuess(h_vars()[0]);
    //std::cout<<"GetInitialGuess of h:"<<h<<std::endl;
    }
  else
  {
    h = fixed_timestep();
  //std::cout<<"GetInitialGuess of h:"<<h<<std::endl;
  }
  
std::cout<<mode.evaluators().num_evaluators()<<std::endl;
std::cout<<mode.evaluators().count_full()<<std::endl;
  int count = 0;
  for (int k=0;k<mode.evaluators().num_evaluators();k++) 
  {
    std::cout<<mode.evaluators().get_evaluator(k).num_full()<<std::endl;
    count += mode.evaluators().get_evaluator(k).num_full();
  }
  VectorXd guess_force(force_vars_[mode_index].size());
  if (traj_init_l.empty()) {
    guess_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints(); ++i) {
      guess_force.segment(mode.evaluators().count_full() * i,
                          mode.evaluators().count_full()) =
          traj_init_l.value(start_time + i * h);
    }
    // for (int i = 0; i < mode.num_knotpoints(); ++i) 
    // {//固定足的evaluator index= 0
    //   guess_force.segment(mode.evaluators().get_evaluator(0).num_full() * i,
    //                       mode.evaluators().get_evaluator(0).num_full() ) =
    //       traj_init_l.value(start_time + i * h);
    // }
  }
  SetInitialGuess(force_vars_[mode_index], guess_force);

  VectorXd guess_collocation_force(collocation_force_vars_[mode_index].size());
  if (traj_init_lc.empty()) {
    guess_collocation_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_force.segment(mode.evaluators().count_full() * i,
                                      mode.evaluators().count_full()) =
          traj_init_lc.value(start_time + (i + 0.5) * h);
    }
    // for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
    //   guess_collocation_force.segment(mode.evaluators().get_evaluator(0).num_full() * i,
    //                                   mode.evaluators().get_evaluator(0).num_full()) =
    //       traj_init_lc.value(start_time + (i + 0.5) * h);
    // }
  }

  SetInitialGuess(collocation_force_vars_[mode_index], guess_collocation_force);

  VectorXd guess_collocation_slack(collocation_slack_vars_[mode_index].size());
  if (traj_init_vc.empty()) {
    guess_collocation_slack.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_slack.segment(mode.evaluators().count_full() * i,
                                      mode.evaluators().count_full()) =
          traj_init_vc.value(start_time + (i + 0.5) * h);
    }
    //   for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
    //   guess_collocation_slack.segment(mode.evaluators().get_evaluator(0).num_full() * i,
    //                                   mode.evaluators().get_evaluator(0).num_full()) =
    //       traj_init_vc.value(start_time + (i + 0.5) * h);
    // }
  }

  // call superclass method
  SetInitialGuess(collocation_slack_vars_[mode_index], guess_collocation_slack);
}

template <typename T>
void Dircon<T>::SetInitialForceTrajectory(
    int mode_index, const PiecewisePolynomial<double>& traj_init_l) {
  const auto& mode = get_mode(mode_index);
  double start_time = 0;
  double h;
  if (timesteps_are_decision_variables())
    {
    h = GetInitialGuess(h_vars()[0]);
    std::cout<<"GetInitialGuess of h:"<<h<<std::endl;
    }
  else
  {
    h = fixed_timestep();
  std::cout<<"GetInitialGuess of h:"<<h<<std::endl;
  }
    
  VectorXd guess_force(force_vars_[mode_index].size());
  VectorXd guess_collocation_force(collocation_force_vars_[mode_index].size());
  if (traj_init_l.empty()) {
    guess_force.fill(0);  // Start with 0 mode.evaluators().get_evaluator(0).num_full()
  } else {
    for (int i = 0; i < mode.num_knotpoints(); ++i) {
      guess_force.segment(mode.evaluators().count_full() * i,
                          mode.evaluators().count_full()) =
          traj_init_l.value(start_time + i * h);
    }
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_force.segment(mode.evaluators().count_full() * i,
                                      mode.evaluators().count_full()) =
          traj_init_l.value(start_time + (i + 0.5) * h);
    }
    //for (int i = 0; i < mode.num_knotpoints(); ++i) {
    //   guess_force.segment(mode.evaluators().get_evaluator(0).num_full() * i,
    //                       mode.evaluators().get_evaluator(0).num_full()) =
    //       traj_init_l.value(start_time + i * h);
    // }
    // for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
    //   guess_collocation_force.segment(mode.evaluators().get_evaluator(0).num_full() * i,
    //                                   mode.evaluators().get_evaluator(0).num_full()) =
    //       traj_init_l.value(start_time + (i + 0.5) * h);
    // }
  }
  SetInitialGuess(force_vars_[mode_index], guess_force);
  SetInitialGuess(collocation_force_vars_[mode_index], guess_collocation_force);
}

template <typename T>
int Dircon<T>::num_modes() const {
  return mode_sequence_.num_modes();
}

template <typename T>
int Dircon<T>::mode_length(int mode_index) const {
  return get_mode(mode_index).num_knotpoints();
}

template <typename T>
void Dircon<T>::ScaleTimeVariables(double scale) {
  for (int i = 0; i < h_vars().size(); i++) {
    this->SetVariableScaling(h_vars()(i), scale);
  }
}

template <typename T>
void Dircon<T>::ScaleQuaternionSlackVariables(double scale) {
  DRAKE_DEMAND(multibody::isQuaternion(plant_));
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    for (int j = 0; j < mode_length(i_mode) - 1; j++) {
      const auto& vars = quaternion_slack_vars(i_mode, j);
      for (int k = 0; k < vars.size(); k++) {
        this->SetVariableScaling(vars(k), scale);
      }
    }
  }
}

template <typename T>
void Dircon<T>::ScaleStateVariable(int state_index, double scale) {
  DRAKE_DEMAND(0 <= state_index &&
               state_index < plant_.num_positions() + plant_.num_velocities());

  // x_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->state(j_knot);
    this->SetVariableScaling(vars(state_index), scale);
  }

  // v_post_impact_vars_
  if (state_index > plant_.num_positions()) {
    for (int mode = 0; mode < num_modes() - 1; mode++) {
      auto vars = post_impact_velocity_vars(mode);
      this->SetVariableScaling(vars(state_index - plant_.num_positions()),
                               scale);
    }
  }
}

template <typename T>
void Dircon<T>::ScaleInputVariable(int input_index, double scale) {
  DRAKE_DEMAND((0 <= input_index) && (input_index < plant_.num_actuators()));

  // u_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->input(j_knot);
    this->SetVariableScaling(vars(input_index), scale);
  }
}

template <typename T>
void Dircon<T>::ScaleForceVariable(int mode_index, int force_index,
                                   double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes()));
  int n_lambda = get_mode(mode_index).evaluators().count_full();
   // int n_lambda = get_mode(mode_index).evaluators().get_evaluator(0).num_full();
  DRAKE_DEMAND((0 <= force_index) && (force_index < n_lambda));

  // Force at knot points
  for (int j = 0; j < mode_length(mode_index); j++) {
    this->SetVariableScaling(force_vars(mode_index, j)(force_index), scale);
  }
  // Force at collocation pints
  for (int j = 0; j < mode_length(mode_index) - 1; j++) {
    this->SetVariableScaling(collocation_force_vars(mode_index, j)(force_index),
                             scale);
  }
}

template <typename T>
void Dircon<T>::ScaleImpulseVariable(int mode_index, int impulse_index,
                                     double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes() - 1));
   int n_impulse = get_mode(mode_index).evaluators().count_full();
  //int n_impulse = get_mode(mode_index).evaluators().get_evaluator(0).num_full();
  DRAKE_DEMAND((0 <= impulse_index) && (impulse_index < n_impulse));

  this->SetVariableScaling(impulse_vars(mode_index)(impulse_index), scale);
}

template <typename T>
void Dircon<T>::ScaleKinConstraintSlackVariable(int mode_index, int slack_index,
                                                double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes() - 1));
   int n_lambda = get_mode(mode_index).evaluators().count_full();
  //int n_lambda = get_mode(mode_index).evaluators().get_evaluator(0).num_full();
  DRAKE_DEMAND(slack_index < n_lambda);

  for (int j = 0; j < mode_length(mode_index) - 1; j++) {
    this->SetVariableScaling(collocation_slack_vars(mode_index, j)(slack_index),
                             scale);
  }
}

template <typename T>
void Dircon<T>::ScaleStateVariables(std::vector<int> index_list, double scale) {
  for (const auto& idx : index_list) {
    ScaleStateVariable(idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleInputVariables(std::vector<int> index_list, double scale) {
  for (const auto& idx : index_list) {
    ScaleInputVariable(idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleForceVariables(int mode, std::vector<int> index_list,
                                    double scale) {
  for (const auto& idx : index_list) {
    ScaleForceVariable(mode, idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleImpulseVariables(int mode, std::vector<int> index_list,
                                      double scale) {
  for (const auto& idx : index_list) {
    ScaleImpulseVariable(mode, idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleKinConstraintSlackVariables(int mode,
                                                 std::vector<int> index_list,
                                                 double scale) {
  for (const auto& idx : index_list) {
    ScaleKinConstraintSlackVariable(mode, idx, scale);
  }
}

template <typename T>
Eigen::MatrixXd Dircon<T>::GetStateSamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd states(num_states(), mode_length(mode));
  for (int i = 0; i < mode_length(mode); i++) {
    states.col(i) = result.GetSolution(state_vars(mode, i));
  }
  return states;
}

template <typename T>
Eigen::MatrixXd Dircon<T>::GetInputSamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd inputs(num_inputs(), mode_length(mode));
  for (int i = 0; i < mode_length(mode); i++) {
    inputs.col(i) = result.GetSolution(input_vars(mode, i));
  }
  return inputs;
}

template <typename T>
Eigen::MatrixXd Dircon<T>::GetForceSamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd forces(get_mode(mode).evaluators().count_full(),
                         mode_length(mode));
  // Eigen::MatrixXd forces(get_mode(mode).evaluators().get_evaluator(0).num_full(),
  //                        mode_length(mode));
                         
  for (int i = 0; i < mode_length(mode); i++) {
    forces.col(i) = result.GetSolution(force_vars(mode, i));//force_vars return VectorXDecisionVariable
  }
  return forces;
}

template <typename T>
Eigen::MatrixXd Dircon<T>::GetForce_C_SamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd collocation_forces(get_mode(mode).evaluators().count_full(),
                         mode_length(mode)-1);
  // Eigen::MatrixXd collocation_forces(get_mode(mode).evaluators().get_evaluator(0).num_full(),
  //                        mode_length(mode)-1);                         
  for (int i = 0; i < mode_length(mode)-1; i++) {
    collocation_forces.col(i) = result.GetSolution(collocation_force_vars(mode, i));//force_vars return VectorXDecisionVariable
  }
  return collocation_forces;
}
    
template <typename T>
Eigen::MatrixXd Dircon<T>::GetGamma_C_SamplesByMode(
    const MathematicalProgramResult& result, int mode) const {
  Eigen::MatrixXd collocation_slack(get_mode(mode).evaluators().count_full(),
                         mode_length(mode)-1);
    // Eigen::MatrixXd collocation_slack(get_mode(mode).evaluators().get_evaluator(0).num_full(),
    //                      mode_length(mode)-1);                       
  for (int i = 0; i < mode_length(mode)-1; i++) {
    collocation_slack.col(i) = result.GetSolution(collocation_slack_vars(mode, i));//force_vars return VectorXDecisionVariable
  }
  return collocation_slack;
}
    
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::Dircon)
