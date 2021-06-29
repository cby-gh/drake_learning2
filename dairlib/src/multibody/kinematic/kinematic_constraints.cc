#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"
#include <drake/multibody/tree/frame.h>

namespace dairlib {
namespace multibody {

using Eigen::VectorXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::VectorX;
using solvers::NonlinearConstraint;

///
///  KinematicPositionConstraint
///
template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : KinematicPositionConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          std::set<int>(), context, description) {}

template <typename T>
KinematicPositionConstraint<T>::KinematicPositionConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::set<int>& full_constraint_relative,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + full_constraint_relative.size(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      full_constraint_relative_(full_constraint_relative) {

  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicPositionConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const {
  const auto& q = vars.head(plant_.num_positions());
  const auto& alpha = vars.tail(full_constraint_relative_.size());
//std::cout<<"alpha.size():"<<alpha.size()<<std::endl;
    // std::cout<<"alpha(i):"<<alpha(0)<<std::endl;
  SetPositionsIfNew<T>(plant_, q, context_);

  *y = evaluators_.EvalActive(*context_);
//std::cout<<"(*y).size():"<<(*y).size()<<std::endl;
    //std::cout<<"(*y)(*y):"<<(*y)<<std::endl;
    //std::cout<<std::endl;
    if((*y)(1)>0.1)
         std::cout<<"ssssdadsadadadasdsdaddfggwftg"<<std::endl;
  // Add relative offsets, looping through the list of relative constraints
  auto it = full_constraint_relative_.begin();
   // std::cout<<"full_constraint_relative_:"<<full_constraint_relative_.size()<<std::endl;
   // std::cout<<"*it:"<<*it<<std::endl;
  for (uint i = 0; i < full_constraint_relative_.size(); i++) {
      //std::cout<<"iiiiiiii:"<<i<<std::endl;
   // std::cout<<"(*y)(*it)0 :"<<(*y)(*it) <<std::endl;  

      
    (*y)(*it) += alpha(i);
    it++;
      //std::cout<<"(*y)(*it)1 :"<<(*y)(*it) <<std::endl;  
      //std::cout <<std::endl;  
    
      
  }
    
}

///
///  KinematicVelocityConstraint
///
template <typename T>
KinematicVelocityConstraint<T>::KinematicVelocityConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : KinematicVelocityConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          context, description) {}

template <typename T>
KinematicVelocityConstraint<T>::KinematicVelocityConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + plant.num_velocities(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators) {
  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicVelocityConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  SetPositionsAndVelocitiesIfNew<T>(plant_, x, context_);

  *y = evaluators_.EvalActiveTimeDerivative(*context_);
}

///
///  KinematicAccelerationConstraint
///
template <typename T>
KinematicAccelerationConstraint<T>::KinematicAccelerationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : KinematicAccelerationConstraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          context, description) {}

template <typename T>
KinematicAccelerationConstraint<T>::KinematicAccelerationConstraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(evaluators.count_active(),
          plant.num_positions() + plant.num_velocities() + plant.num_actuators()
              + evaluators.count_full(),
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators) {
  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}

template <typename T>
void KinematicAccelerationConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, VectorX<T>* y) const {
  const auto& x = vars.head(plant_.num_positions() + plant_.num_velocities());
  const auto& u = vars.segment(plant_.num_positions() + plant_.num_velocities(),
      plant_.num_actuators());
  const auto& lambda = vars.tail(evaluators_.count_full());
  multibody::setContext<T>(plant_, x, u, context_);

  *y = evaluators_.EvalActiveSecondTimeDerivative(context_, lambda);
}

///
///  foot_location_Constraint
///
template <typename T>
foot_location_Constraint<T>::foot_location_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description)
    : foot_location_Constraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_active()),
          VectorXd::Zero(evaluators.count_active()),
          std::set<int>(), context, description) {}

template <typename T>
foot_location_Constraint<T>::foot_location_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::set<int>& full_constraint_relative,
    Context<T>* context, const std::string& description)
    : NonlinearConstraint<T>(full_constraint_relative.size(),
          full_constraint_relative.size()*3,
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      full_constraint_relative_(full_constraint_relative) {

  // Create a new context if one was not provided
  if (context == nullptr) {
    owned_context_ = plant_.CreateDefaultContext();
    context_ = owned_context_.get();
  } else {
    context_ = context;
  }
}
 
template <typename T>
void foot_location_Constraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, drake::VectorX<T>* y) const {
   //std::cout<<"vars.size():"<<vars.size()<<std::endl;   
  DRAKE_DEMAND(vars.size() == 3);
  const auto&  rel0 = vars(0);//rel_x at mode 0
  const auto&  rel1 = vars(1);//rel_x at mode 1
  const auto&  rel2 = vars(2);//rel_x at mode 2
  drake::VectorX<T> vector_rel0(1);
  drake::VectorX<T> vector_rel1(1);  
  drake::VectorX<T> vector_rel2(1);  
  vector_rel0(0)=rel0;
  vector_rel1(0)=rel1;
   vector_rel2(0)=rel2; 
  *y = 0.5*(vector_rel2-vector_rel0)+vector_rel0-vector_rel1;
      
  }

///
///  floatfoot_collosionavoidence_Constraint
///
template <typename T>
floatfoot_collosionavoidence_Constraint<T>::floatfoot_collosionavoidence_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    Context<T>* context, const std::string& description, int mode_index, int knot_index)
    : floatfoot_collosionavoidence_Constraint<T>(plant, evaluators,
          VectorXd::Zero(evaluators.count_full()),
          VectorXd::Zero(evaluators.count_full()),
          std::set<int>(), context, description, mode_index,knot_index) {}

template <typename T>
floatfoot_collosionavoidence_Constraint<T>::floatfoot_collosionavoidence_Constraint(
    const MultibodyPlant<T>& plant,
    const KinematicEvaluatorSet<T>& evaluators,
    const VectorXd& lb, const VectorXd& ub,
    const std::set<int>& full_constraint_relative,
    Context<T>* context, const std::string& description , int mode_index, int knot_index)
    : NonlinearConstraint<T>(full_constraint_relative.size()*3,
          plant.num_positions() + full_constraint_relative.size()*3,
          lb, ub, description),
      plant_(plant), 
      evaluators_(evaluators),
      full_constraint_relative_(full_constraint_relative),
      mode_index_(mode_index),
      knot_index_(knot_index)
{

    // Create a new context if one was not provided
    if (context == nullptr) {
      owned_context_ = plant_.CreateDefaultContext();
      context_ = owned_context_.get();
    } else {
      context_ = context;
    }
}
 
template <typename T>
void floatfoot_collosionavoidence_Constraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& vars, drake::VectorX<T>* y) const
{
  const auto& q = vars.head(plant_.num_positions());
  const auto& alpha = vars.tail(3);
  SetPositionsIfNew<T>(plant_, q, context_);
  const Eigen::Vector3d pt(0.10, 0, -0.29);//以foot作为落足点30//这里需要重新设置,保证精确
  
VectorX<T>pt_target(3);//以foot作为落足点30//这里需要重新设置,保证精确

  SetPositionsIfNew<T>(plant_, q, context_);

    //   *y = evaluators_.EvalFull(*context_);

    // std::cout<<y->size()<<std::endl;
    // *y = *y + alpha;
    const drake::multibody::Frame<T>& left_lower_leg = plant_.GetFrameByName("link_left_foot");//left_lower_leglink_left_toe
    const auto& right_lower_leg = plant_.GetFrameByName("link_right_foot");//right_lower_leglink_right_toe
      
      const drake::multibody::Frame<T>& world = plant_.world_frame();
      const Eigen::Vector3d pt_A_;
      Eigen::MatrixXd m(3,1);
      //m<<pt_A_(0),pt_A_(1),pt_A_(2);
      m.col(0)=pt_A_;
      if(mode_index_==0)
      {
        plant_.CalcPointsPositions(*context_,right_lower_leg,m.template cast<T>(),world,&pt_target);
      }
      else if(mode_index_==1)
      {
        plant_.CalcPointsPositions(*context_,left_lower_leg,m.template cast<T>(),world,&pt_target);
      }
      else if(mode_index_==2)
      {
        plant_.CalcPointsPositions(*context_,right_lower_leg,m.template cast<T>(),world,&pt_target);
      }  
      
        * y = alpha + pt_target; 
}




DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicPositionConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicVelocityConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::KinematicAccelerationConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::foot_location_Constraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::floatfoot_collosionavoidence_Constraint)

}  // namespace multibody
}  // namespace dairlib
