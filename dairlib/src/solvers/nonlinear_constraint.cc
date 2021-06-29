#include "solvers/nonlinear_constraint.h"

#include "drake/common/default_scalars.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace solvers {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::VectorX;
using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
NonlinearConstraint<T>::NonlinearConstraint(int num_constraints, int num_vars,
                                            const VectorXd& lb,
                                            const VectorXd& ub,
                                            const std::string& description,
                                            double eps)
    : Constraint(num_constraints, num_vars, lb, ub, description), eps_(eps) {
		//eps 初始化为1e-7
		//std::cout<<"num_constraintsxxx"<<num_constraints<<std::endl;
		//std::cout<<"num_varsxxx"<<num_vars<<std::endl;
		//std::cout<<"constraint.get_description()"<<get_description()<<std::endl;
																			}

template <typename T>
void NonlinearConstraint<T>::SetConstraintScaling(
    const std::unordered_map<int, double>& map) {
  constraint_scaling_ = map;
}

template <typename T>
template <typename U>
void NonlinearConstraint<T>::ScaleConstraint(VectorX<U>* y) const {
  for (const auto& member : constraint_scaling_) {
    (*y)(member.first) *= member.second;
  }
}

template <>
void NonlinearConstraint<double>::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  EvaluateConstraint(x, y);
	//std::cout<<"1111111111111111111111"<<std::endl;
  this->ScaleConstraint<double>(y);
}

template <>
void NonlinearConstraint<AutoDiffXd>::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  EvaluateConstraint(drake::math::initializeAutoDiff(x), &y_t);
	std::cout<<"2222222222222222222222"<<std::endl;
  *y = drake::math::autoDiffToValueMatrix(y_t);
  this->ScaleConstraint<double>(y);
}

template <typename T>
void NonlinearConstraint<T>::DoEval(
    const Eigen::Ref<const VectorX<drake::symbolic::Variable>>& x,
    VectorX<drake::symbolic::Expression>* y) const {
	std::cout<<"3333333333333333333"<<std::endl;
  throw std::logic_error(
      "NonlinearConstraint does not support symbolic evaluation.");
}

template <>
void NonlinearConstraint<AutoDiffXd>::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
	std::cout<<"444444444444444"<<std::endl;
  EvaluateConstraint(x, y);
  this->ScaleConstraint<AutoDiffXd>(y);
}

template <>
void NonlinearConstraint<double>::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
	//std::cout<<"begin0000000000000"<<std::endl;
	//std::cout<<"before autoDiffToGradientMatrix(x).size(): "<<x.size()<<std::endl;
    MatrixXd original_grad = drake::math::autoDiffToGradientMatrix(x);//从自动微分数据类型(该类型包括数值，微分值，联想计算图中的节点)梯度矩阵
    //std::cout<<"original_grad.shape(): "<<original_grad.rows()<<"x"<<original_grad.cols()<<std::endl;
	// forward differencing
	//std::cout<<"num_constraintsxxx"<<num_constraints()<<std::endl;
	//std::cout<<"num_varsxxx"<<num_vars()<<std::endl;
	//std::cout<<"before autoDiffToValueMatrix.size(): "<<x.size()<<std::endl;
  VectorXd x_val = drake::math::autoDiffToValueMatrix(x);//convert autodiff to VectorXd, currently they are same.//获取autodiff中的函数值
	//std::cout<<"after autoDiffToValueMatrix.size(): "<<x_val.size()<<std::endl;
  VectorXd y0, yi;
  EvaluateConstraint(x_val, &y0);//Rm->Rn的计算过程，即f()
	//std::cout<<"after EvaluateConstrainty0.size(): "<<y0.size()<<std::endl;

  MatrixXd dy = MatrixXd(y0.size(), x_val.size());
  for (int i = 0; i < x_val.size(); i++) {
    x_val(i) += eps_;
    EvaluateConstraint(x_val, &yi);
    x_val(i) -= eps_;
    dy.col(i) = (yi - y0) / eps_;
  }
//std::cout<<"dy.shape(): "<<dy.rows()<<"x"<<dy.cols()<<std::endl;
//std::cout<<"original_grad.shape(): "<<original_grad.rows()<<"x"<<original_grad.cols()<<std::endl;
  // Profiling identified dy * original_grad as a significant runtime event,
  // even though it is almost always the identity matrix.
  if (original_grad.isIdentity(1e-16))//判断在这个精度下是不是梯度矩阵是不是一个单位阵
  {
	 //std::cout<<"*yes"<<std::endl;
    *y = drake::math::initializeAutoDiffGivenGradientMatrix(y0, dy);//用梯度矩阵和函数值生成一个autodiff的y，即该类型包括标量和偏导
	  
	  //当每一维的自变量和函数值是一一对应，比如f1=x1 f2=x2 f3=x3
  } else {
    *y = drake::math::initializeAutoDiffGivenGradientMatrix(y0,
                                                            dy * original_grad);//当每一维的自变量和函数值并不是一一对应，而是混杂，比如f1=(x1*x2) f2=x3(x1+x2)....
	  std::cout<<"*no"<<std::endl;
  }
	//std::cout<<"dy.shape(): "<<dy.rows()<<"x"<<dy.cols()<<std::endl;
	//std::cout<<"dy: "<<dy<<std::endl;
	
	//std::cout<<"y0.size()"<<y0.size()<<std::endl;
	//std::cout<<"y0"<<y0<<std::endl;
	
    //std::cout<<"*y.shape(): "<<(*y).rows()<<"x"<<(*y).cols()<<std::endl;
	if(y0.size()!=(*y).rows())std::cout<<"end00000000000001"<<std::endl;
	//std::cout<<"*y"<<*y<<std::endl;
  this->ScaleConstraint<AutoDiffXd>(y);
	//std::cout<<"end0000000000000"<<std::endl;
}

}  // namespace solvers
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::solvers::NonlinearConstraint)
