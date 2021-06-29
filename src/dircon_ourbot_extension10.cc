#include <ctime>
#include <string>  
#include <iostream>
#include <typeinfo> 
#include <fstream>// c++文件操作
#include <iomanip>// 设置输出格式
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h> 

//#include "periodic_input.h"
//#include "drake/multibody/multibody_utils.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
# include "multibody/multibody_utils.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "common/find_resource.h"
//#include "dairlib/systems/vector_scope.h"// direct import from /home/cby/dairlib, see details in cmakelist 
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph_inspector.h"
#include <gflags/gflags.h>
#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
//#include "drake/multibody/tree/multibody_tree.h" 
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/frame.h"

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/constraint.h>
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"
#include <drake/common/find_resource.h>

//#include "common/find_resource.h"
//#include "systems/trajectory_optimization/dircon/dircon.h"
//#include "multibody/kinematic/world_point_evaluator.h"
//#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "drake/common/drake_assert.h"
#include "extensionx_trajectory/extension10_trajectory.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
DEFINE_double(strideLength, 0.1, "The stride length.");
DEFINE_double(duration, 1.5, "The stride duration");//Tmode1!=Tmode0
DEFINE_bool(autodiff, false, "Double or autodiff version");

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using std::map;
using std::string;
using std::vector;

using drake::multibody::MultibodyPlant;//释放到当前作用域
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
//using drake::math::RigidTransform;
using drake::trajectories::PiecewisePolynomial;
//using drake::systems::DiagramBuilder;
using drake::multibody::JointIndex;
using drake::multibody::FrameIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::multibody::JointActuator;
using drake::multibody::JointActuatorIndex;

void print_localtime() {
  std::time_t result = std::time(nullptr);
  std::cout << std::asctime(std::localtime(&result));
}

bool solve_result=0;
namespace dairlib {
namespace {

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;
std::vector<MatrixXd> result_x;
std::vector<MatrixXd> result_u;   
std::vector<MatrixXd> result_u_prior;   
std::vector<MatrixXd> result_x_prior;    
template <typename T>
void runDircon(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    double stride_length,
    double duration,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_l_traj,
    vector<PiecewisePolynomial<double>> init_lc_traj,
    vector<PiecewisePolynomial<double>> init_vc_traj) {
    auto start1 = std::chrono::high_resolution_clock::now();
    drake::systems::DiagramBuilder<double> builder;
    MultibodyPlant<T>& plant = *plant_ptr;
    //std::move means take the owner ship && AddSystem return a bare pointer.
    drake::geometry::SceneGraph<double>& scene_graph =*builder.AddSystem(std::move(scene_graph_ptr));

    /// Construct a map between joint names and position indices
    ///     <name,index> such that q(index) has the given name 
    /// return a map: std::map<string, int>
    std::unique_ptr<Context<T>> context = plant.CreateDefaultContext();
   
    drake::VectorX<T> positions = plant.GetPositions(*context, ModelInstanceIndex(2));//在multibody sysyem中MII0给world,MII1给了drake自建,后续的
    drake::VectorX<T> velocities = plant.GetVelocities(*context, ModelInstanceIndex(2));//才会给urdf引入的instance
    std::cout<<"positions.rows()"<<positions.rows()<<std::endl;//8 x z lh rh lk rk la ra 
    std::cout<<"positions.cols()"<<positions.cols()<<std::endl;
    std::cout<<"positions.size()"<<positions.size()<<std::endl;
    std::cout<<"velocities.rows()"<<velocities.rows()<<std::endl;//8 x z lh rh lk rk la ra 
    std::cout<<"velocities.cols()"<<velocities.cols()<<std::endl;
    std::cout<<"velocities.size()"<<velocities.size()<<std::endl;
    std::vector<JointIndex> joint_indices0 = plant.GetJointIndices(ModelInstanceIndex(0));
    std::cout<<"Model world has"<<joint_indices0.size()<<" joints"<<std::endl;
    std::vector<JointIndex> joint_indices1 = plant.GetJointIndices(ModelInstanceIndex(1));
    std::cout<<"Model default has"<<joint_indices1.size()<<" joints"<<std::endl;
    std::vector<JointIndex> joint_indices2 = plant.GetJointIndices(ModelInstanceIndex(2));
    std::cout<<"Model robot has "<<joint_indices2.size()<<" joints"<<std::endl;
    //N.B. we have add a weld joint between base and world, so there is additional one joint, so the size is 12, 11 previous
    //can also check the position index is the same order in joint order.
    for(int ii=0; ii<joint_indices2.size();ii++)//12
    {
        const drake::multibody::Joint<T>&  joint = plant.get_joint(joint_indices2[ii]);
        std::cout<<"joint_indices2["<<ii<<"]: "<<joint_indices2[ii]<<"->"<<joint.name()<<std::endl;
    }
    std::cout<<std::endl;

  std::cout<<"to_stringto_stringto_stringto_stringto_string0:"<<(*context).to_string()<<std::endl;//8 x z lh rh lk rk la ra 
  drake::VectorX<T> test(16);
  test<<0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15;
  context->SetContinuousState(test);
  std::cout<<"to_stringto_stringto_stringto_stringto_string1:"<<(*context).to_string()<<std::endl;//8 x z lh rh lk rk la ra 
    auto positions_map = multibody::makeNameToPositionsMap(plant);//return a std::map<std::string, int>
    auto velocities_map = multibody::makeNameToVelocitiesMap(plant);//且state顺序并没有改变
    //first = k, second = value
  std::cout<<"to_stringto_stringto_stringto_stringto_string2:"<<(*context).to_string()<<std::endl;//8 x z lh rh lk rk la ra 
    for (auto const& element : positions_map)
       cout << "positions_map:" <<element.first << " = " << element.second << endl;
    std::cout<<std::endl;
    for (auto const& element : velocities_map)
       cout << "velocities_map:"  << element.first << " = " << element.second << endl;
    std::cout<<std::endl;
    
    //红左蓝右，这里的frame是material frame，为该body的根坐标系，也是body_frame相当于ros中的父joint坐标系的位置。
    const drake::multibody::Frame<T>& left_lower_leg = plant.GetFrameByName("link_left_foot");//left_lower_leglink_left_toe
    const auto& right_lower_leg = plant.GetFrameByName("link_right_foot");//right_lower_leglink_right_toe
    
    //pt_A the contact point on the body与地面接触的点
    //足端点相对于关节膝关节joint的位置/问题是初始位置还是摆动位置，这两个位置相对与膝关节是一个位置，这是关于膝关节关节坐标系讲的
    //问题2：如果是欠驱动怎么办
    //Vector3d pt(0, 0, -0.5);
    //Vector3d pt(0, 0, -0.27);
    Vector3d pt(0, 0, -0.28);//以foot作为落足点30
    //Vector3d pt(0, 0, 0);//以toe作为落足点
    Vector3d ones(1,0,0);
    double mu = 3;
    //左落足点相关计算类,相关运动学计算器，也蕴含了各种约束，包括了地面接触时的碰撞点的判定，即xyz位置信息。
    //但是在这一步还没有进行各种约束的选择，只是设定了被激活的方向和初始化
    //应当是计算落足点在世界坐标系下的位姿
    auto left_foot_eval = multibody::WorldPointEvaluator<T>(plant, pt,
                                                            left_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 2});//0 1 2对应所有xyz方向， 激活方向0 2 = x z
    left_foot_eval.set_frictional(); //is_frictional_ = true; 启用接触摩擦模型
    left_foot_eval.set_mu(mu);//mu_=mu=1

  auto right_foot_eval = multibody::WorldPointEvaluator<T>(plant, pt, 
                                                           right_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 2});
  right_foot_eval.set_frictional();//meaens     constraints.push_back(solvers::CreateConicFrictionConstraint(this->mu(), 2));
  right_foot_eval.set_mu(mu);
    
  auto evaluators_left = multibody::KinematicEvaluatorSet<T>(plant);//初始化一个运动学(约束)集合
  int left_foot_eval_index =  evaluators_left.add_evaluator(&left_foot_eval);//加入左脚的运动学计算类，并返回当前运动学计算器在vector中的索引
  cout << "left_foot_eval_index: " << " = " << left_foot_eval_index << endl;//左足端点计算器的索引 0 
  auto evaluators_right = multibody::KinematicEvaluatorSet<T>(plant);
  int right_foot_eval_index =  evaluators_right.add_evaluator(&right_foot_eval);
  cout << "right_foot_eval_index: " << " = " << right_foot_eval_index << endl;//右足端点计算器的索引 0 

  int num_knotpoints_mode0 = 10;
  int num_knotpoints_mode1 = 20;
 //double min_T = .1;//The minimum total duration of the mode (default 0)
  //double max_T = 3;//The maximum total duration of the mode (default 0)
  double  min_T_mode0 = 0.5;//在这里直接修改左右相时间相等或者在类里添加时间相等约束
  double max_T_mode0 = 0.5;//
  double  min_T_mode1 = 1;//在这里直接修改左右相时间相等或者在类里添加时间相等约束
  double max_T_mode1 = 1;//
//Each DirconMode object refers to a single hybrid mode of the optimization  
  auto mode_left = DirconMode<T>(evaluators_left, num_knotpoints_mode0, min_T_mode0, max_T_mode0);//定义dircon的左腿mode,该初始化仅定义了一些成员变量，没有额外的操作
  mode_left.MakeConstraintRelative(0, 0);  // x-coordinate在特定的腿的运动学计算器中的激活的方向选中更加激活的方向(relative direction)，x方向足端要向
  //前前进rel_offset的具体,所以x方向比active方向更特殊,标记为relative, 第一个arg为运动学计算器,实际上就是计算第一个约束,第二个为该约束里比active
  //更特殊的方向
  auto mode_right = DirconMode<T>(evaluators_right, num_knotpoints_mode1, min_T_mode1, max_T_mode1);//定义dircon的右腿mode
  mode_right.MakeConstraintRelative(0, 0);

    
    //左足支撑相(先迈右脚，蓝色)和右足支撑相
  auto sequence = DirconModeSequence<T>(plant);//初始化一个空的序列
  sequence.AddMode(&mode_left);//在表示序列的vector中插入指向mode的指针
  sequence.AddMode(&mode_right);

  auto trajopt = Dircon<T>(sequence);  
  std::cout<< "trajopt.num_vars()after create dircon: "<<  trajopt.num_vars()<<std::endl;//579
  std::cout<< "trajopt.GetAllConstraints().size() after create dircon:"<<  trajopt.GetAllConstraints().size()<<std::endl;//144
  std::cout<< "trajopt.GetAllCosts().size() after create dircon:"<<  trajopt.GetAllCosts().size()<<std::endl;//8
//Adds a duration constraint on the total duration of the trajectory. 
 trajopt.AddDurationBounds(duration, duration);//duration = 1.5s for 2 mode duration of the trajectory. 
  
//set solver
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),"Print file", "../snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),"Major iterations limit", 200);//200
// std::cout<< " drake::solvers::SnoptSolver::id().name()"<< drake::solvers::SnoptSolver::id().name()<<std::endl; now is SNOPT
//Set the initial guess for the trajectory decision variables. 
  for (int j = 0; j < sequence.num_modes(); j++) 
  {
    cout << "num_modes:" << " = " << sequence.num_modes() << endl;//当前的modes数目 2
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::SetInitialTrajectory(init_u_traj, init_x_traj);//40 knot points
       /// Set the initial guess for the force variables for a specific mode
        /// traj_init_l contact forces lambda (interpreted at knot points)
         ///traj_init_lc contact forces (interpreted at collocation points)
  /// traj_init_vc velocity constraint slack variables (at collocation)
    trajopt.SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j], init_vc_traj[j]);
      cout << "trajopt.SetInitialForceTrajectory rows:" << " = " << init_l_traj[j].rows() << endl;//各个mode下约束力lammda大小
      cout << "trajopt.SetInitialForceTrajectory cols:" << " = " << init_l_traj[j].cols() << endl;//各个mode下约束力lammda大小
  }

// Periodicity constraints
// la = 7
// ra = 6
// lk = 5
// rk = 4
// lh = 3
// rh = 2
// planar_x = 0
// planar_z = 1

    //标准约束原始约束    
  auto x0 = trajopt.initial_state();
  auto xf = trajopt.final_state();
  auto x_exact=trajopt.state();//state(index)
   // const drake::solvers::VectorXDecisionVariable state_vars(
    //  int mode_index, int knotpoint_index) const;
  int nq = plant.num_positions();
  trajopt.AddLinearConstraint(x0(positions_map["planar_z"]) == xf(positions_map["planar_z"]));
  trajopt.AddLinearConstraint(x0(positions_map["joint_left_hip"]) ==0.0);
  trajopt.AddLinearConstraint(x0(positions_map["joint_right_hip"]) ==0.0);
  trajopt.AddLinearConstraint(x0(positions_map["joint_left_knee"]) ==0.0);
  trajopt.AddLinearConstraint(x0(positions_map["joint_right_knee"]) ==0.0);
  trajopt.AddLinearConstraint(xf(positions_map["joint_left_knee"]) ==-0.3);
  trajopt.AddLinearConstraint(xf(positions_map["joint_right_knee"]) ==0.0);//当前测试中可能需要删掉
  trajopt.AddLinearConstraint(xf(positions_map["joint_left_hip"]) ==-0.3);
  trajopt.AddLinearConstraint(xf(positions_map["joint_right_hip"]) ==0.0);  

  //下面四行修改后
 // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_hipdot"])  ==xf(nq + velocities_map["joint_left_hipdot"]));
 // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_hipdot"])  ==xf(nq + velocities_map["joint_right_hipdot"]));
 // trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_kneedot"]) ==xf(nq + velocities_map["joint_left_kneedot"]));
  //trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_kneedot"]) ==xf(nq + velocities_map["joint_right_kneedot"]));  
  trajopt.AddLinearConstraint(xf(nq + velocities_map["joint_right_hipdot"])  ==0.0);
  trajopt.AddLinearConstraint(xf(nq + velocities_map["joint_left_hipdot"])  ==0.0 );
  trajopt.AddLinearConstraint(xf(nq + velocities_map["joint_right_kneedot"])  ==0.0 );
  trajopt.AddLinearConstraint(xf(nq + velocities_map["joint_left_kneedot"])  ==0.0 );
  trajopt.AddLinearConstraint(xf(nq + velocities_map["joint_left_ankledot"]) ==0.0);


  //trajopt.AddLinearConstraint(xf(positions_map["joint_left_ankle"]) ==x0(positions_map["joint_left_ankle"]));
 // trajopt.AddLinearConstraint(xf(positions_map["joint_right_ankle"]) ==x0(positions_map["joint_right_ankle"]));
  //trajopt.AddLinearConstraint(xf(positions_map["joint_left_ankle"]) ==x0(positions_map["joint_right_ankle"]) );
 // trajopt.AddLinearConstraint(xf(positions_map["joint_right_ankle"]) ==x0(positions_map["joint_left_ankle"]) );
  //trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_left_ankledot"]) ==xf(nq + velocities_map["joint_right_ankledot"]));
  //trajopt.AddLinearConstraint(x0(nq + velocities_map["joint_right_ankledot"]) ==xf(nq + velocities_map["joint_left_ankledot"]));  


  trajopt.AddLinearConstraint(x0(nq + velocities_map["planar_zdot"]) ==xf(nq + velocities_map["planar_zdot"]));
  trajopt.AddLinearConstraint(x0(nq + velocities_map["planar_xdot"]) ==xf(nq + velocities_map["planar_xdot"])); 


  auto x = trajopt.state();
  std::cout<< "trajopt.state():"<< x<<std::endl;//8   
  trajopt.AddConstraintToAllKnotPoints(x(nq + velocities_map["planar_xdot"])  >= 0);//19knots  
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["planar_z"])  >= 0);  

  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_hip"]) >= -0.8);//joint_left_knee
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_hip"]) >= -0.8);//joint_right_knee    
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_hip"]) <= 0.8);  //0.8
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_hip"]) <= 0.8);  //0.8
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_knee"]) <= 0.4);  
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_knee"]) <= 0.4);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_knee"]) >= -0.4);  
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_knee"]) >= -0.4); 
  //ankle 
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_ankle"]) >= -0.2);//-0.02
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_right_ankle"]) <= 0.2);  //0.02
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_ankle"]) >= -0.2);//-0.02
  trajopt.AddConstraintToAllKnotPoints(x(positions_map["joint_left_ankle"]) <= 0.2); //0.02

  trajopt.AddLinearConstraint(x0(positions_map["planar_x"]) == 0);
  trajopt.AddLinearConstraint(xf(positions_map["planar_x"]) == 2*stride_length);
   
// planarwalker trajopt.force_vars(0, i) 是3 对应lambdax lambday lambdax 
 for (int i = 0; i < 10; i++) {
//force_vars是每个knot point处的力变量，设置每个knot处的y方向的约束力为0，因为只有平面   
    trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(0, i)(1));
    cout << "trajopt.force_vars(0, i)(1):" << " = " << trajopt.force_vars(0, i).size() << endl;
    trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(1, i)(1));
    trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(1, 10+i)(1));
    cout << "trajopt.force_vars(1, i)(1):" << " = " << trajopt.force_vars(1, i).size() << endl;//右足端点计算器的索引 0 
  }

    //printf("here");

  auto u = trajopt.input();
  drake::solvers::VectorXDecisionVariable x_left_hip(29);
  drake::solvers::VectorXDecisionVariable x_dot_left_hip(29);
  drake::solvers::VectorXDecisionVariable x_right_hip(29);
  drake::solvers::VectorXDecisionVariable x_dot_right_hip(29);
  drake::solvers::VectorXDecisionVariable x_left_knee(29);
  drake::solvers::VectorXDecisionVariable x_dot_left_knee(29);
  drake::solvers::VectorXDecisionVariable x_right_knee(29);
  drake::solvers::VectorXDecisionVariable x_dot_right_knee(29);
  drake::solvers::VectorXDecisionVariable x_right_ankle(29);
  drake::solvers::VectorXDecisionVariable x_dot_right_ankle(29);
  drake::solvers::VectorXDecisionVariable x_left_ankle(29);
  drake::solvers::VectorXDecisionVariable x_dot_left_ankle(29);
  for(int i=0;i<29;i++)
  {
    x_left_hip[i]=trajopt.state(i)[3];//x_vars()是project的,不能在class instace中使用
    x_right_hip[i]=trajopt.state(i)[2];//x_vars()是project的,不能在class instace中使用
    x_left_knee[i]=trajopt.state(i)[5];//x_vars()是project的,不能在class instace中使用
    x_right_knee[i]=trajopt.state(i)[4];//x_vars()是project的,不能在class instace中使用
    x_right_ankle[i]=trajopt.state(i)[6];//x_vars()是project的,不能在class instace中使用
    x_left_ankle[i]=trajopt.state(i)[7];//x_vars()是project的,不能在class instace中使用

    x_dot_left_hip[i]=trajopt.state(i)[11];//x_vars()是project的,不能在class instace中使用
    x_dot_right_hip[i]=trajopt.state(i)[10];//x_vars()是project的,不能在class instace中使用
    x_dot_left_knee[i]=trajopt.state(i)[13];//x_vars()是project的,不能在class instace中使用
    x_dot_right_knee[i]=trajopt.state(i)[12];//x_vars()是project的,不能在class instace中使用
    x_dot_right_ankle[i]=trajopt.state(i)[14];//x_vars()是project的,不能在class instace中使用
    x_dot_left_ankle[i]=trajopt.state(i)[15];//x_vars()是project的,不能在class instace中使用     
  }

  const double R = 4;  // Cost on input 
  const double Q = 4;  // Cost on total state 

  const double L = 10;  // x_left_hip
  const double M = 8;  // x_right_hip
  const double N = 10;  // x_left_knee
  const double O = 8;  // x_right_knee

  const double P = 8;  // x_dot_left_hip
  const double S = 8;  // x_dot_right_hip
    const double V = 10;  // x_dot_left_knee
      const double U = 8;  // x_dot_right_knee
  //添加手工代价
  //trajopt.AddRunningCost(theta_rk.transpose()*N*theta_rk);
  trajopt.AddRunningCost(u.transpose()*R*u);
  //trajopt.AddRunningCost(x(3)*L*x(3));//lh
  //trajopt.AddRunningCost(x(2)*L*x(2));//rh
  //trajopt.AddRunningCost(x(10)*M*x(10));//rhdot
  //trajopt.AddRunningCost(x(11)*M*x(11));//lhdot
  trajopt.AddRunningCost(x.transpose()*Q*x);
  //trajopt.AddRunningCost(trajopt.state().transpose()*M*trajopt.state());
  trajopt.AddRunningCost(x_left_hip.transpose()*L*x_left_hip);//good,关键
  trajopt.AddRunningCost(x_right_hip.transpose()*M*x_right_hip);//good,关键
  trajopt.AddRunningCost(x_left_knee.transpose()*N*x_left_knee);//开始晃了不加hip的约束
  trajopt.AddRunningCost(x_right_knee.transpose()*O*x_right_knee);//开始晃了不加hip的约束

  trajopt.AddRunningCost(x_dot_left_hip.transpose()*P*x_dot_left_hip);//less useful
  trajopt.AddRunningCost(x_dot_right_hip.transpose()*S*x_dot_right_hip);//less useful
  trajopt.AddRunningCost(x_dot_left_knee.transpose()*V*x_dot_left_knee);//less useful
  trajopt.AddRunningCost(x_dot_right_knee.transpose()*U*x_dot_right_knee);//less useful
  std::cout<< "trajopt.num_vars()after manually setting: "<<  trajopt.num_vars()<<std::endl;//579
  std::cout<< "trajopt.GetAllConstraints().size() after manually settingss:"<<  trajopt.GetAllConstraints().size()<<std::endl;//144
  std::cout<< "trajopt.GetAllCosts().size() after manually setting:"<<  trajopt.GetAllCosts().size()<<std::endl;//8   
  std::vector<unsigned int> visualizer_poses;
  visualizer_poses.push_back(3);
  visualizer_poses.push_back(3);
//std::cout << "222" << std::endl;//finding_path2 <<std::endl;//drake/examples/PlanarWalker/PlanarWalker2.urdf
  trajopt.CreateVisualizationCallback(
      drake::FindResourceOrThrow("drake/examples/v10/v10_drake_extension4_82.urdf"),//e
      visualizer_poses, 0.2, "base");//something happen here as i can't use FindResourceOrThrow from dairlib, please check here
  //std::string finding_path2=dairlib::GetResourceSearchPaths();
  //std::cout << "222" << std::endl;//finding_path2 <<std::endl;
  std::cout<<"trajopt.GetAllConstraints().size():"<<trajopt.GetAllConstraints().size()<<std::endl;
  std::cout<<"trajopt.decision_variables().size():"<<trajopt.decision_variables().size()<<std::endl;
  auto finish1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_parse = finish1 - start1;//求解时长
  std::cout << "Parse1 time:" << elapsed_parse.count() <<std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  //std::cout<<"在solve()函数中具体实施某些动力学约束的实现"<<std::endl;
    std::cout<<"print_localtime"<<std::endl;
    print_localtime();
  const auto result = Solve(trajopt, trajopt.initial_guess());//规划器求解结果 trajopt.initial_guess():Getter for the initial guess. 
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;//求解时长
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  std::cout << "herehere"  <<std::endl;
  std::cout <<"result.is_success():"<<result.is_success()<<std::endl;
  std::cout << "result.get_solver_id().name()"<<result.get_solver_id().name()<<std::endl;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;  
  // visualizer
    //GetStateAndDerivativeSamples这个函数存在维度错误
  
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =trajopt.ReconstructStateTrajectory(result);
  const drake::trajectories::PiecewisePolynomial<double> pp_utraj =trajopt.ReconstructInputTrajectory(result);
  std::cout<< "trajopt.num_vars()trajopt.num_vars(): "<<  trajopt.num_vars()<<std::endl;//579
  std::cout<< "trajopt.decision_variables()trajopt.decision_variables(): "<<  trajopt.decision_variables()<<std::endl;
  Eigen::MatrixXd force_value_mode0=trajopt.GetForceSamplesByMode(result,0);
  Eigen::MatrixXd force_value_mode1=trajopt.GetForceSamplesByMode(result,1);
  Eigen::MatrixXd force_c_value_mode0=trajopt.GetForce_C_SamplesByMode(result,0);
  Eigen::MatrixXd force_c_value_mode1=trajopt.GetForce_C_SamplesByMode(result,1);
  Eigen::MatrixXd slack_value_mode0=trajopt.GetGamma_C_SamplesByMode(result,0);
  Eigen::MatrixXd slack_value_mode1=trajopt.GetGamma_C_SamplesByMode(result,1);
  std::cout<< "force_value_mode0.size(): "<<  force_value_mode0.rows()<<"x"<<force_value_mode0.cols()<<std::endl;//应当是3x10
  std::cout<< "force_value_mode0.data(): "<<  force_value_mode0<<std::endl;//3x10
  std::cout<< "force_value_mode1.size(): "<<  force_value_mode1.rows()<<"x"<<force_value_mode1.cols()<<std::endl;//3x20
  std::cout<< "force_value_mode1.data(): "<<  force_value_mode1<<std::endl;//3x20
  
  std::cout<< "force_c_value_mode0.size(): "<<  force_c_value_mode0.rows()<<"x"<<force_c_value_mode0.cols()<<std::endl;//3x9
  std::cout<< "force_c_value_mode0.data(): "<<  force_c_value_mode0<<std::endl;
  std::cout<< "force_c_value_mode1.size(): "<<  force_c_value_mode1.rows()<<"x"<<force_c_value_mode1.cols()<<std::endl;//3x19
  std::cout<< "force_c_value_mode1.data(): "<<  force_c_value_mode1<<std::endl;
  
  std::cout<< "slack_value_mode0.size(): "<<  slack_value_mode0.rows()<<"x"<<slack_value_mode0.cols()<<std::endl;//3x9
  std::cout<< "slack_value_mode0.data(): "<<  slack_value_mode0<<std::endl;
  std::cout<< "slack_value_mode1.size(): "<<  slack_value_mode1.rows()<<"x"<<slack_value_mode1.cols()<<std::endl;//3x19
  std::cout<< "slack_value_mode1.data(): "<<  slack_value_mode1<<std::endl;
  
  //std::cout<<"trajopt.decision_variable(0).to_string()"<<trajopt.decision_variable(0).to_string()<<std::endl;//398   
  //std::cout<<"trajopt.decision_variable(578).to_string()"<<trajopt.decision_variable(578).to_string()<<std::endl;//398   
  std::cout<<"trajopt.decision_variable_values:"<<result.GetSolution()<<std::endl;//579个值lammda0
  auto decision_variables_value = result.GetSolution();
  std::cout<<"decision_variables_value.size()"<<decision_variables_value.size()<<std::endl;

    //读取第一mode中的lammda lammda_c gamma_c
    std::fstream out_lammda_data_file0;
    std::fstream out_lammda_c_data_file0;
    std::fstream out_gamma_c_data_file0;
    out_lammda_data_file0.open("/home/cby/drake_learning/src/drake_learning1/src/lammda_planed_outsereach.txt", std::ios::out | std::ios::trunc);//输出且写入清空
    out_lammda_c_data_file0.open("/home/cby/drake_learning/src/drake_learning1/src/lammda_c_planed_outsereach.txt", std::ios::out | std::ios::trunc);
    out_gamma_c_data_file0.open("/home/cby/drake_learning/src/drake_learning1/src/gammda_c_planed_outsereach.txt", std::ios::out | std::ios::trunc);
    out_lammda_data_file0 << std::fixed;
    out_lammda_c_data_file0 << std::fixed;//不用科学计数法
    out_gamma_c_data_file0 << std::fixed;//不用科学计数法

  Eigen::MatrixXd lammda_value_mode0(3,10);
  Eigen::MatrixXd lammda_c_value_mode0(3,9);
  Eigen::MatrixXd gamma_c_value_mode0(3,9);
  Eigen::MatrixXd lammda_value_mode1(3,20);
  Eigen::MatrixXd lammda_c_value_mode1(3,19);
  Eigen::MatrixXd gamma_c_value_mode1(3,19);
  int lammda_index0 = 398;
  for(int i=0;i<10;i++)
  {
    for(int j=0;j<3;j++)
    {
      lammda_value_mode0(j,i) = decision_variables_value[lammda_index0];
      out_lammda_data_file0 << std::setprecision(4) << lammda_value_mode0(j,i)<< std::endl;//四位有效数字
      lammda_index0++;
      std::cout<<"lammda_value_mode0(j,i)"<<lammda_value_mode0(j,i)<<std::endl;
    }
  }

  out_lammda_data_file0.close();  
  //判断从decision variables来的值是不是等于其他method来的,应当yes
  if(lammda_value_mode0==force_value_mode0)
  std::cout<<"yes lammda_value_mode0==force_value_mode0"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no lammda_value_mode0!=force_value_mode0"<<std::endl;//579个值lammda0   

   int lammda_c_index0 = 428;
  for(int i=0;i<9;i++)
  {
    for(int j=0;j<3;j++)
    {
      lammda_c_value_mode0(j,i) = decision_variables_value[lammda_c_index0];
      out_lammda_c_data_file0 << std::setprecision(4) << lammda_c_value_mode0(j,i)<<std::endl;//四位有效数字
      lammda_c_index0++;
    }
  }
  lammda_c_index0=lammda_c_index0-3;
      for(int j=0;j<3;j++)
    {
      out_lammda_c_data_file0 << std::setprecision(4) << decision_variables_value[lammda_c_index0]<<std::endl;//四位有效数字
      lammda_c_index0++;
    }
  out_lammda_c_data_file0.close();
  if(lammda_c_value_mode0==force_c_value_mode0)
  std::cout<<"yes lammda_c_value_mode0==force_c_value_mode0"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no lammda_c_value_mode0!=force_c_value_mode0"<<std::endl;//579个值lammda0   

   int gamma_c_index0 = 455;
  for(int i=0;i<9;i++)
  {
    for(int j=0;j<3;j++)
    {
      gamma_c_value_mode0(j,i) = decision_variables_value[gamma_c_index0];
      out_gamma_c_data_file0 << std::setprecision(4) << gamma_c_value_mode0(j,i)<< std::endl;//四位有效数字
      gamma_c_index0++;
    }
  }
  gamma_c_index0=gamma_c_index0-3;
      for(int j=0;j<3;j++)
    {
      out_gamma_c_data_file0 << std::setprecision(4) << decision_variables_value[gamma_c_index0]<<std::endl;//四位有效数字
      gamma_c_index0++;
    }
  out_gamma_c_data_file0.close();  
  if(gamma_c_value_mode0==slack_value_mode0)
  std::cout<<"yes gamma_c_value_mode0==slack_value_mode0"<<std::endl;//579个值lammda0  
  else
  std::cout<<"no gamma_c_value_mode0!=slack_value_mode0"<<std::endl;//579个值lammda0   
  //读取mode1中的lammda lammda_c gamma_c
    std::fstream out_lammda_data_file1;
    std::fstream out_lammda_c_data_file1;
    std::fstream out_gamma_c_data_file1;  
    out_lammda_data_file1.open("/home/cby/drake_learning/src/drake_learning1/src/lammda_planed_outsereach.txt", std::ios::out | std::ios::app);
    out_lammda_c_data_file1.open("/home/cby/drake_learning/src/drake_learning1/src/lammda_c_planed_outsereach.txt", std::ios::out | std::ios::app);
    out_gamma_c_data_file1.open("/home/cby/drake_learning/src/drake_learning1/src/gammda_c_planed_outsereach.txt", std::ios::out | std::ios::app);
    out_lammda_data_file1 << std::fixed;
    out_lammda_c_data_file1 << std::fixed;//不用科学计数法
    out_gamma_c_data_file1 << std::fixed;//不用科学计数法
   int lammda_index1 = 483;
    for(int i=0;i<20;i++)
    {
      for(int j=0;j<3;j++)
      {
        lammda_value_mode1(j,i) = decision_variables_value[lammda_index1];
        out_lammda_data_file1 << std::setprecision(4) << lammda_value_mode1(j,i)<<std::endl;//四位有效数字
        lammda_index1++;
        std::cout<<"lammda_value_mode1(j,i)"<<lammda_value_mode1(j,i)<<std::endl;
      }
    }
    out_lammda_data_file1.close();
    if(lammda_value_mode1==force_value_mode1)
    std::cout<<"yes lammda_value_mode1==force_value_mode1"<<std::endl;//579个值lammda0  
    else
    std::cout<<"no lammda_value_mode1!=force_value_mode1"<<std::endl;//579个值lammda0   

   int lammda_c_index1 = 513;//变了,需要重新计算
  for(int i=0;i<9;i++)
  {
    for(int j=0;j<3;j++)
    {
      lammda_c_value_mode1(j,i) = decision_variables_value[lammda_c_index1];
      out_lammda_c_data_file1 << std::setprecision(4) << lammda_c_value_mode1(j,i)<<std::endl;//四位有效数字
      lammda_c_index1++;
    }
  }
    lammda_c_index1=lammda_c_index1-3;
      for(int j=0;j<3;j++)
    {
      out_lammda_c_data_file1 << std::setprecision(4) << decision_variables_value[lammda_c_index1]<<std::endl;//四位有效数字
      lammda_c_index1++;
    }
    out_lammda_c_data_file1.close();
  if(lammda_c_value_mode1==force_c_value_mode1)
    std::cout<<"yes lammda_c_value_mode1==force_c_value_mode1"<<std::endl;//579个值lammda0  
  else
    std::cout<<"no lammda_c_value_mode1!=force_c_value_mode1"<<std::endl;//579个值lammda0   

   int gamma_c_index1 = 540;
    for(int i=0;i<9;i++)
    {
      for(int j=0;j<3;j++)
      {
        gamma_c_value_mode1(j,i) = decision_variables_value[gamma_c_index1];
        out_gamma_c_data_file1 << std::setprecision(4) << gamma_c_value_mode1(j,i)<<std::endl;//四位有效数字
        gamma_c_index1++;
      }
    }
      gamma_c_index1=gamma_c_index1-3;
      for(int j=0;j<3;j++)
    {
      out_gamma_c_data_file1 << std::setprecision(4) << decision_variables_value[gamma_c_index1]<<std::endl;//四位有效数字
      gamma_c_index1++;
    }
    out_gamma_c_data_file1.close();
    if(gamma_c_value_mode1==slack_value_mode1)
    std::cout<<"yes gamma_c_value_mode1==slack_value_mode1"<<std::endl;//579个值lammda0  
    else
    std::cout<<"no gamma_c_value_mode1!=slack_value_mode1"<<std::endl;//579个值lammda0   

 //multibody::connectTrajectoryVisualizer(plant_double_ptr,&builder, &scene_graph, pp_xtraj);
//目前未知,不算上面这个函数,仅注册了scene_graph系统,diagram中仅有一个

auto traj_source_new = builder.AddSystem<drake::dairlib::cby::extension6_trajectory<double>>(pp_xtraj, plant_double_ptr);//<double>
auto to_pose =builder.AddSystem<drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(*plant_double_ptr);
builder.Connect(traj_source_new->get_output_port(), to_pose->get_input_port());
builder.Connect(to_pose->get_output_port(),  scene_graph.get_source_pose_port(plant_double_ptr->get_source_id().value()));
drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
std::cout<<"hao many system now:"<<builder.GetMutableSystems().size()<<std::endl;
for(int i = 0; i<builder.GetMutableSystems().size();i++)
{
  std::cout << "system name:"  <<builder.GetMutableSystems()[i]->GetMemoryObjectName()<<std::endl;
}
    //std::cout << "3"  <<std::endl;
 auto diagram = builder.Build();

    
   double trajectory_time_u =  pp_utraj.end_time()-pp_utraj.start_time();
   double trajectory_time_x =  pp_xtraj.end_time()-pp_xtraj.start_time();
    // auto trajectory_duration =  pp_utraj.duration(1);
     //const std::vector<double>& trajectory_breaks=  pp_utraj.breaks();
    std::cout << "trajectory_time_u:"  <<trajectory_time_u<<std::endl;
    std::cout << "trajectory_time_x:"  <<trajectory_time_x<<std::endl;
    //100Hz均匀取值
    double times = 0.0;
    for(int m = 0; m<150;m++)
    {
        result_u.push_back(pp_utraj.value(times));
        result_x.push_back(pp_xtraj.value(times));
        times += 0.01;
    }   
    //读取每个knotpoint值,共29个
    times = 0.0;
    for(int m = 0; m<29;m++)
    {
        result_u_prior.push_back(pp_utraj.value(times));
        result_x_prior.push_back(pp_xtraj.value(times));
        times += 0.05263;
    } 
  
    std::fstream out_x_data_file;
    std::fstream out_u_data_file;
    out_u_data_file.open("/home/cby/drake_learning/src/drake_learning1/src/u_planed_outsearch.txt", std::ios::out | std::ios::trunc);
    out_x_data_file.open("/home/cby/drake_learning/src/drake_learning1/src/x_planed_outsearch.txt", std::ios::out | std::ios::trunc);
    out_x_data_file << std::fixed;
    out_u_data_file << std::fixed;//不用科学计数法
    for(int i=0;i<result_u_prior.size();i++)
  {
        out_u_data_file << std::setprecision(4) << result_u_prior[i]<< std::endl;//四位有效数字
        out_x_data_file << std::setprecision(4) << result_x_prior[i]<< std::endl;//std::setprecision(4)显示4位
  }
    out_x_data_file.close();  
    out_u_data_file.close();
    std::cout << "result_u.size():"  <<result_u.size()<<std::endl;//4*150      完整一步时是4*200
    std::cout << "result_x.size():"  <<result_x.size()<<std::endl;//16*150   完整一步时是20*200
    std::cout << "result_u[0].size():"  <<result_u[0].size()<<std::endl;
    std::cout << "result_x[0].size():"  <<result_x[0].size()<<std::endl;
   
      for(int t = 0;t<150;t++)
    {

        std::cout << "tttttttttt: "  <<t<<std::endl;
        std::cout << "result_x:left_leg_hipdot: "  <<result_x[t](11)<<std::endl;
        std::cout << "result_x:right_hip_pindot: "  <<result_x[t](10)<<std::endl;
         std::cout << "result_x:left_knee_pindot: "  <<result_x[t](13)<<std::endl;
         std::cout << "result_x:right_knee_pindot: "  <<result_x[t](12)<<std::endl;
    } 
         std::cout << "rhdot0: "  <<pp_xtraj.value(0)(10,0)<<std::endl;        
         std::cout << "lhdot0: "  <<pp_xtraj.value(0)(11,0)<<std::endl;      
         std::cout << "rkdot0: "  <<pp_xtraj.value(0)(12,0)<<std::endl;        
         std::cout << "lkdot0: "  <<pp_xtraj.value(0)(13,0)<<std::endl;    
         std::cout << "rhdot1: "  <<pp_xtraj.value(1.5)(10,0)<<std::endl;        
         std::cout << "lhdot1: "  <<pp_xtraj.value(1.5)(11,0)<<std::endl;      
         std::cout << "rkdot1: "  <<pp_xtraj.value(1.5)(12,0)<<std::endl;        
         std::cout << "lkdot1: "  <<pp_xtraj.value(1.5)(13,0)<<std::endl;    
         
    /* for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:left_leg_hip:"  <<  t<<":"<<result_u[t](0)<<std::endl;
    }    */
    
      /*  for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:right_leg_hip:" <<  t<<":" <<result_u[t](1)<<std::endl;
            
    }   */

     /*     for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:hip_pin: t"<<  t<<":"<<result_u[t](2)<<std::endl;
    }    
       for(int t = 0;t<100;t++)
    {
        std::cout << "result_u:right_knee_pin:" <<  t<<":" <<result_u[t](3)<<std::endl;
    }     */
     std::cout <<"result.is_success():"<<result.is_success()<<std::endl;
     std::cout << "result.get_solver_id().name()"<<result.get_solver_id().name()<<std::endl;
     std::cout << "Solve time:" << elapsed.count() <<std::endl;  
      if(result.is_success()==0)
      {
        solve_result=0;
        std::cout << "InfeasibleConstraints.size():" << result.GetInfeasibleConstraintNames(trajopt).size() <<std::endl;  
        std::cout << "InfeasibleConstraints.size():" << result.GetInfeasibleConstraints(trajopt).size() <<std::endl;  
        for(int i=0;i<result.GetInfeasibleConstraintNames(trajopt).size();i++)
        {
          std::cout << "InfeasibleConstraints name:" << result.GetInfeasibleConstraintNames(trajopt)[i] <<std::endl;
          std::cout << "InfeasibleConstraints:" << result.GetInfeasibleConstraints(trajopt)[i].to_string() <<std::endl;  
          std::cout << "InfeasibleConstraints num:" << result.GetInfeasibleConstraints(trajopt)[i].evaluator()->num_constraints() <<std::endl;  
        }
      }
  else if(result.is_success()==1)
  {
      solve_result=1;
      while (true) 
      {
        drake::systems::Simulator<double> simulator(*diagram);
        simulator.set_target_realtime_rate(0.3);
        simulator.Initialize();
        simulator.AdvanceTo(10);//pp_xtraj.end_time()
    }  
  }
}

}  // namespace
}  // namespace dairlib



std::string get_greet(const std::string& who) {
  return "Hello " + who;
}




int main(int argc, char* argv[]) 
{
    auto start0 = std::chrono::high_resolution_clock::now();
    std::string words = get_greet("CBY project developer!!!");
    std::cout <<words<<std::endl;
    std::cout<<"Current local time: ";
    print_localtime();
    gflags::ParseCommandLineFlags(&argc, &argv, true);  
    std::srand(time(0));  
    auto plant = std::make_unique<drake::multibody::MultibodyPlant<double>>(0.0);
    auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
    auto scene_graph = std::make_unique<SceneGraph<double>>();

    Parser parser(plant.get()); //unique_ptr get classical pointer且没有给scenegraph因为plant对象仅用来规划
    Parser parser_vis(plant_vis.get(), scene_graph.get()); 
  
    std::string full_name = drake::FindResourceOrThrow("drake/examples/v10/v10_drake_extension4_82.urdf");//
    drake::FindResourceResult ResourceResult = drake::FindResource("drake/examples/v10/v10_drake_extension4_82.urdf");//
    std::optional<std::string> full_name_same =  ResourceResult.get_absolute_path();   
    if(full_name_same)
    {
        std::cout<<"full path & name: "<<*full_name_same<<std::endl;//.. means another one
        std::cout << "Success to find urdf file!"  <<std::endl;
        std::string our_urdf_path = "/opt/drake/share/drake/examples";
        std::cout <<"LOCATION: " <<"please add our model here: " << our_urdf_path <<std::endl;    
        std::cout << "We have Ourbot now!"  <<std::endl;
        std::cout <<std::endl;
    }
    else
    {
       std::cout<<"NOT FOUND ERROR: "<<*ResourceResult.get_error_message()<<"\n"<<std::endl;
       std::cout <<std::endl;
    } 
  
    //Parses the SDF or URDF file named in arguement and adds one model to plant. 
    ModelInstanceIndex  mii_p = parser.AddModelFromFile(full_name);
    ModelInstanceIndex  mii_pv =  parser_vis.AddModelFromFile(full_name);
    parser_vis.AddModelFromFile(drake::FindResourceOrThrow("drake/examples/v10/warehouse.sdf"));
    //modelindex in multibody plant system
    auto num_model_instances = plant->num_model_instances();
    std::cout << "Baisc-info1 Multibodyplant 'plant' has "  << num_model_instances<< " modles"<<std::endl;  
    std::cout<<"mii_plant is: "<<mii_p<<std::endl;
    
    auto num_model_instances2 = plant_vis->num_model_instances();
    std::cout << "Baisc-info1 Multibodyplant 'plant_vis' has "  << num_model_instances2<< " modles"<<std::endl;    
    std::cout<<"mii_plant_vis is: "<<mii_pv<<std::endl;
    std::cout<<"Baisc-info2: "<<mii_pv<<std::endl;
    for (ModelInstanceIndex i(0); i<num_model_instances;i++)
    {
        const std::string&  ModelInstance_name = plant->GetModelInstanceName(i);
        ModelInstanceIndex  ModelInstance = plant->GetModelInstanceByName(ModelInstance_name);
        std::cout << "\t model_name:"  <<ModelInstance_name<< std::endl; 
        bool yes=plant->HasModelInstanceNamed("v10");
        std::cout << "\t model_index"<< ModelInstance_name <<"-v10 has been installed ?: "  <<yes<< std::endl; 
    }
    std::cout <<std::endl;
    //joints 11
    auto num_joints = plant->num_joints();
    std::cout << "Baisc-info3 v10 has "  << num_joints << " Joints"<<std::endl;  
    for(JointIndex i(0); i<num_joints;i++)
    {
        const drake::multibody::Joint<double>&  joint = plant->get_joint(i);
        std::cout << "\tJoints:"  <<joint.name()<< std::endl;         
    }
    std::cout << std::endl;   
    
    //frames 24(1world+12body frame+11 joint frame)
    auto num_frames = plant->num_frames();
    //前12个是每个link算一个
     std::cout << "Baisc-info3 v10 has "  << num_frames << " frames"<<std::endl;  
     for(FrameIndex i(0); i<plant->num_frames();i++)
     {
         const drake::multibody::Frame<double>& each_frame= plant->get_frame(i);
         std::cout << "\tFrame:"  <<each_frame.name()<< std::endl;  
     }
     std::cout << std::endl;   
    
    //base相当于一个地板(基座)，固定在世界坐标系下
    //将地板(基座) 与世界坐标系合一 
    const drake::multibody::BodyFrame<double>& world_frame = plant->world_frame();
    const drake::multibody::Frame<double>& base_frame= plant->GetFrameByName("base");
    const drake::multibody::BodyFrame<double>& world_frame_vis = plant_vis->world_frame();
    const drake::multibody::Frame<double>& base_frame_vis= plant_vis->GetFrameByName("base");
    plant->WeldFrames(world_frame, base_frame, drake::math::RigidTransform<double>());
    plant_vis->WeldFrames(world_frame_vis, base_frame_vis, drake::math::RigidTransform<double>());  
    //after all elements in the model (joints, bodies, force elements, constraints, etc.) , 
    plant->Finalize();
    plant_vis->Finalize();
    std::cout << "Topological information configuration complete: plant, plant_vis " <<std::endl;  
    std::cout <<std::endl;         
    
    //state q+q_dot and initialize to 0
    int configuration_q = plant->num_positions();
    int configuration_q_dot = plant->num_velocities();
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(configuration_q + configuration_q_dot);  //2n
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(plant->num_actuators());  //2n
    std::cout << "plant->num_actuators: "  <<plant->num_actuators()<<std::endl;  //4
    std::cout << "plant->num_actuated_dofs: "  <<plant->num_actuated_dofs()<<std::endl;  //4
    std::cout << "Baisc-info4 State dimensions: "  <<(configuration_q + configuration_q_dot)<<std::endl;  //16
    std::cout <<std::endl;

    //4,lh,rh,lk,rk for ourbot
    int nu = plant->num_actuators();
    std::cout << "Baisc-info5 Actuators num: "  << nu <<std::endl;  
    for (JointActuatorIndex i(0); i<nu;i++){
          const JointActuator<double>&  joint_actuators = plant->get_joint_actuator(i);
          std::cout << "\t actuator_index:"  <<i<< std::endl; 
          std::cout << "\t actuator_name:"  <<joint_actuators.name()<< std::endl; 
          const drake::multibody::Joint<double>&  actuator_joint = (plant->get_joint_actuator(i)).joint();
          std::cout << "\t joint_name:"  <<actuator_joint.name()<< std::endl; 
      }
   
//    std::unique_ptr<Context<double>>  contextnow = plant->CreateDefaultContext();
//   std::cout <<" plant->get_actuation_input_port().get_index()"<<plant->get_actuation_input_port().get_index()<<std::endl;  
//    std::cout <<" plant->get_actuation_input_port().GetFullDescription()"<<plant->get_actuation_input_port().GetFullDescription()<<std::endl;     
//        std::cout <<" plant->get_actuation_input_port().HasValue()"<<plant->get_actuation_input_port().HasValue(*contextnow)<<std::endl;     

    //dimensions of state    
    int nx = plant->num_positions() + plant->num_velocities();
    int N = 10;
    int M = 20;//对与第二个mode给他20个点,因为他的时长是mode0的两倍
    int i = 0;
    //GRF information 地面接触力，xyz三个方向 
    //一个初始向量，给力初始samples用的
    Eigen::VectorXd init_l_vec(3);
    init_l_vec << 0, 0, 8.93*9.81;//20质量 initial lammda10
    //for(int i=0; i<init_l_vec.size();i++)
   // {
  //      std::cout<<"Debug:"<<init_l_vec[i]<<std::endl;  
 //   }    
    //矩阵向量
    //初始的状态啊轨迹和输入轨迹
    std::vector<MatrixXd> init_x;
    std::vector<MatrixXd> init_u;
    std::vector<drake::trajectories::PiecewisePolynomial<double>> init_l_traj;//地面接触力，xyz三个方向 traj_init_l contact forces lambda (interpreted at knot points)
    std::vector<PiecewisePolynomial<double>> init_lc_traj;//traj_init_lc contact forces constraint slack variables(interpreted at collocation points)
    std::vector<PiecewisePolynomial<double>> init_vc_traj;  //traj_init_vc velocity constraint slack variables (at collocation)
    // std::cout << "Data type of init_vc_traj: "  <<typeid(init_vc_traj).name()<< std::endl; //i is the alias of int, c=char
    // Initialize state trajectory 0-18:19个值
    //定义break points and knot points. N.B. 这二十个点是两个mode组成的sequence的完整体
    //N.B.as it use a ZeroOrderHold to synthesis an initial x trajectory and u trajectory, so it need 19 point values
    std::vector<double> init_time;
    std::fstream out_ramdom_x_file;//输出获得先验轨迹的初始随机轨迹
    std::fstream out_ramdom_u_file;
    out_ramdom_x_file.open("/home/cby/drake_learning/src/drake_learning1/src/random_x_search.txt", std::ios::out | std::ios::trunc);
    out_ramdom_u_file.open("/home/cby/drake_learning/src/drake_learning1/src/random_u_search.txt", std::ios::out | std::ios::trunc);
    out_ramdom_x_file << std::fixed;
    out_ramdom_u_file << std::fixed;//不用科学计数法
    for (int i = 0; i < N+M-1; i++)//29:0-28
    {
        init_time.push_back(i*0.05263);//breaks
        //std::cout<<"init_time: "<<init_time[i]<<std::endl;
        init_x.push_back(x0 + .1*VectorXd::Random(nx));//初始状态轨迹随机初始化
        init_u.push_back(VectorXd::Random(nu));//u[i]
        //init_x.push_back(x0);//初始状态轨迹0初始化 x0已经是0无解
        //init_u.push_back(u0);//初始输入轨迹0初始化 u0已经是0无解
        //std::cout<<"init_x[i]:"<<init_x[i]<<std::endl;
        for(int j=0;j<nx;j++)
        { 
            out_ramdom_x_file << std::setprecision(4) << init_x[i](j)<<std::endl;//四位有效数字
            //std::cout<<"init_x[i](j):"<<init_x[i](j)<<std::endl;
        }
        //std::cout<<"init_u[i]:"<<init_u[i]<<std::endl;
        for(int j=0;j<nu;j++)
        { 
            out_ramdom_u_file << std::setprecision(4) << init_u[i](j)<<std::endl;//四位有效数字
            //std::cout<<"init_u[i](j):"<<init_u[i](j)<<std::endl;
        }
    }
        out_ramdom_x_file.close();
        out_ramdom_u_file.close();

    std::cout<<"init_time.size(): "<<init_time.size()<<std::endl;
   int result1 = access("/home/cby/drake_learning/src/drake_learning1/src/x_planed_outsearch_select0.txtxxx", 0);//加入先验条件时,需要撤去文件名称的xxx
   int result2 = access("/home/cby/drake_learning/src/drake_learning1/src/u_planed_outsearch_select0.txtxxx", 0);
  if(result1==0&&result2==0)
  {
      std::cout<<"exist prior files xu"<<std::endl;
      std::fstream in_x_txt_file;
      std::fstream in_u_txt_file;
      in_x_txt_file.open("/home/cby/drake_learning/src/drake_learning1/src/x_planed_outsearch_select0.txt", std::ios::out | std::ios::in);
      in_u_txt_file.open("/home/cby/drake_learning/src/drake_learning1/src/u_planed_outsearch_select0.txt", std::ios::out | std::ios::in);
      in_x_txt_file << std::fixed; 
      in_u_txt_file << std::fixed;    /**/
      double data_x; 
      double data_u;
      for(int i=0;i<init_x.size();i++)
      {
        for(int j=0;j<16;j++)
        {
          in_x_txt_file>>data_x;
          init_x[i](j)= data_x;
          std::cout<<"init_x[i](j): "<<init_x[i](j)<<std::endl;
        }
         for(int j=0;j<4;j++)
        {
          in_u_txt_file>>data_u;
          init_u[i](j)= data_u;
          std::cout<<"init_u[i](j): "<<init_u[i](j)<<std::endl;
        }
      }
        in_x_txt_file.close();
        in_u_txt_file.close();
  }
  
    std::cout << "Baisc-info6 segments of  init_time trajectory:"<<init_time.size()<<std::endl;
    std::cout << "Baisc-info6 Dimensions of init_x at  each knot point:"<<init_x[i].size()<<std::endl;
    std::cout << "Baisc-info6 segments of   init_x trajectory:"<<init_x.size()<<std::endl;
    std::cout << "Baisc-info6 Dimensions of init_u at  each knot point:"<<init_u[0].size()<<std::endl;
    std::cout << "Baisc-info6 segments of init_u trajectory:"<<init_u.size()<<std::endl;
    std::cout <<"TIME BREAK POINT~X KNOT POINT~U KNOT POINT"<<std::endl; 
    std::cout <<std::endl;  
    
    //reminder: x is cubic spline and u is first-order 
    std::cout << "The initial time sequence(break points) and x init trajectory and u init trajectory (knot points on x(t), u(t) )has set."<<std::endl;
  
    auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
    auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

    //在一个breakpoint处多项式的输出的值的形状
    Eigen::Index  row_x = init_x_traj.rows();
    Eigen::Index  col_x = init_x_traj.cols();
    Eigen::Index  row_u = init_u_traj.rows();
    Eigen::Index  col_u = init_u_traj.cols();
    std::cout << "Baisc-info7 Output shape(rows x cols) of init_x_traj at break points: "  <<row_x<<" x "<<col_x<<std::endl;
    std::cout << "Baisc-info7 Output shape(rows x cols) of init_u_traj at break points: "  <<row_u<<" x "<<col_u<<std::endl; 
    std::cout << "Baisc-info8 segments  of init_u_traj at break points: "  <<init_u_traj.get_number_of_segments()<<std::endl;
    std::cout << "Baisc-info8 segments  of init_x_traj at break points: "  <<init_x_traj.get_number_of_segments()<<std::endl;

    int lamda_ptr=227;
    int lamda_c_ptr=225;
    int gamma_c_ptr=226;
    for (int j = 0; j < 2; j++) 
    {
        if(j==0)
        {
           lamda_ptr=0;
           lamda_c_ptr=0;
           gamma_c_ptr=0;
        }
        else
        {
           lamda_ptr=227;//227
           lamda_c_ptr=225;//223
           gamma_c_ptr=221;//223
        }
        
        std::vector<MatrixXd> init_l_j;//lammda contraint force
        std::vector<MatrixXd> init_lc_j;//lammda_c force correction
        std::vector<MatrixXd> init_vc_j;//gamma velocity correction
        std::vector<double> init_time_j;//breaks
        for (int i = 0; i < N; i++) 
        {
           // init_time_j.push_back(i*.2);05263
            init_time_j.push_back(i*0.05263);
            init_l_j.push_back(init_l_vec);//0 0 8.9*9.81
            init_lc_j.push_back(init_l_vec);//0 0 8.9*9.81
            init_vc_j.push_back(VectorXd::Zero(3));//000
        }
        // std::cout << "init_l_j:init_lc_j:init_vc_j.size() "  <<init_l_j.size()<<init_lc_j.size()<<init_vc_j.size()<<std::endl;
        int result3 = access("/home/cby/drake_learning/src/drake_learning1/src/lammda_planed_outsereach_select0xxx.txt", 0);//加入先验条件时,需要撤去文件名称的xxx
        int result4 = access("/home/cby/drake_learning/src/drake_learning1/src/lammda_c_planed_outsereach_select0xxx.txt", 0);
        int result5 = access("/home/cby/drake_learning/src/drake_learning1/src/gammda_c_planed_outsereach_selsect0xxx.txt", 0);
       if(result3==0&&result4==0&&result5==0)
       {
          std::cout<<"exist prior files l lc rc"<<std::endl;
          std::fstream in_lammda_data_file;
          std::fstream in_lammda_c_data_file;
          std::fstream in_gamma_c_data_file;

          in_lammda_data_file.open("/home/cby/drake_learning/src/drake_learning1/src/lammda_planed_outsereach_select0.txt", std::ios::out | std::ios::in);
          in_lammda_c_data_file.open("/home/cby/drake_learning/src/drake_learning1/src/lammda_c_planed_outsereach_select0.txt", std::ios::out | std::ios::in);
          in_gamma_c_data_file.open("/home/cby/drake_learning/src/drake_learning1/src/gammda_c_planed_outsereach_selsect0.txt", std::ios::out | std::ios::in);
          in_lammda_data_file << std::fixed;
          in_lammda_c_data_file << std::fixed;    
          in_gamma_c_data_file << std::fixed;    
          in_lammda_data_file.seekg(lamda_ptr,std::ios::beg);
          in_lammda_c_data_file.seekg(lamda_c_ptr,std::ios::beg);
          in_gamma_c_data_file.seekg(gamma_c_ptr,std::ios::beg);
          double data_lammda;
          double data_lammda_c;
          double data_gamma_c;
            /*     
              if(j==0)
                  {
                      for(int m=0;m<init_l_j.size();m++)
                      {
                          for(int n=0;n<3;n++)
                          {
                              in_lammda_data_file>>data_lammda;
                              init_l_j[m](n)= data_lammda;
                              std::cout<<"init_l_j[m](n): "<<init_l_j[m](n)<<std::endl;
              std::cout<<"in_lammda_data_file.tellg()"<<in_lammda_data_file.tellg()<<std::endl;          
                          }
                      }
                  }
                  else if(j==1)
                  {
                      for(int m=0;m<init_l_j.size()*2;m++)
                      {
                          for(int n=0;n<3;n++)
                          {
                            if(m<init_l_j.size())
                            {
                              in_lammda_data_file>>data_lammda;
                              std::cout<<"data_lammda: "<<data_lammda<<std::endl;
                              std::cout<<"in_lammda_data_file.tellg()"<<in_lammda_data_file.tellg()<<std::endl;     
                            }
                              else if(m>=init_l_j.size())
                              {
                                  in_lammda_data_file>>data_lammda;
                                  init_l_j[m-init_l_j.size()](n)= data_lammda;
                                  std::cout<<"init_l_j[m](n): "<<init_l_j[m-init_l_j.size()](n)<<std::endl;
                                  std::cout<<"in_lammda_data_file.tellg()"<<in_lammda_data_file.tellg()<<std::endl;          
                              }
                          }
                      }
                  }
            */

          for(int m=0;m<init_l_j.size();m++)
          {
              for(int n=0;n<3;n++)
              {
                  in_lammda_data_file>>data_lammda;
                  init_l_j[m](n)= data_lammda;
                  std::cout<<"init_l_j[m](n): "<<init_l_j[m](n)<<std::endl;
              }
          }
          std::cout<<"in_lammda_data_file.tellg()"<<in_lammda_data_file.tellg()<<std::endl;          
          for(int m=0;m<init_lc_j.size();m++)
          {
            for(int n=0;n<3;n++)
            {
              in_lammda_c_data_file>>data_lammda_c;
              init_lc_j[m](n)= data_lammda_c;
              std::cout<<"init_lc_j[m](n): "<<init_lc_j[m](n)<<std::endl;
            }
          }
          std::cout<<"in_lammda_c_data_file.tellg()"<<in_lammda_c_data_file.tellg()<<std::endl;   
          for(int m=0;m<init_vc_j.size();m++)
          {
            for(int n=0;n<3;n++)
            {
              in_gamma_c_data_file>>data_gamma_c;
              init_vc_j[m](n)= data_gamma_c;
              std::cout<<"init_vc_j[m](n): "<<init_vc_j[m](n)<<std::endl;
            }
          }
          std::cout<<"in_gamma_c_data_file.tellg()"<<in_gamma_c_data_file.tellg()<<std::endl;   
          in_gamma_c_data_file.close();
          in_lammda_c_data_file.close();
          in_lammda_data_file.close();
       }

      std::cout << "Baisc-info11111 size of  init_time_j "  <<init_time_j.size()<<std::endl;  
      std::cout << "Baisc-info9 size of init_l_j[0] "  <<init_l_j[0].size()<<std::endl;  
      std::cout << "Baisc-info9 size of init_lc_j[0] "  <<init_lc_j[0].size()<<std::endl;  
      std::cout << "Baisc-info9 size of init_vc_j[0] "  <<init_vc_j[0].size()<<std::endl; 
      //零阶力轨迹 生成轨迹
      // traj_init_l contact forces λ  (interpreted at knot points)
      //traj_init_lc contact λ'  (interpreted at collocation points)
      //traj_init_vc velocity constraint slack variables  γ (at collocation)
      auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);//9+19段
      auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
      auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

      //三维的轨迹束，共两段，两个多维的轨迹放在一个向量里
      init_l_traj.push_back(init_l_traj_j);//两个mode下，9+19段，
      init_lc_traj.push_back(init_lc_traj_j);
      init_vc_traj.push_back(init_vc_traj_j);
      std::cout << "Baisc-info10 Num_modes of init_l_traj  "  << init_l_traj.size()<<std::endl;  
      std::cout << "Baisc-info10 Output shape of init_l_traj_j: nx1 n="<<init_l_traj_j.rows()<<std::endl;         
      std::cout << "Baisc-info10 segments  of init_l_traj at break points: "  <<init_l_traj[j].get_number_of_segments()<<std::endl;
      std::cout << "Baisc-info11 Num_modes of init_lc_traj  "  << init_lc_traj.size()<<std::endl;  
      std::cout << "Baisc-info11 Output shape of init_lc_traj_j: nx1 n="<<init_l_traj_j.rows()<<std::endl;         
      std::cout << "Baisc-info11 segments  of init_lc_traj at break points: "  <<init_lc_traj[j].get_number_of_segments()<<std::endl;
      std::cout << "Baisc-info12 Num_modes of init_vc_traj  "  << init_vc_traj.size()<<std::endl;  
      std::cout << "Baisc-info12 Output shape of init_vc_traj_j: nx1 n="<<init_vc_traj_j.rows()<<std::endl;         
      std::cout << "Baisc-info12 segments  of init_vc_traj at break points: "  <<init_vc_traj[j].get_number_of_segments()<<std::endl;
          std::cout<<"init_l_j.size():"<<init_l_j.size()<<std::endl;     
    }
    std::cout<<std::endl;         

    if (FLAGS_autodiff)
     {
      std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(*plant);
      dairlib::runDircon<drake::AutoDiffXd>(
      std::move(plant_autodiff), plant_vis.get(), std::move(scene_graph),
      FLAGS_strideLength, FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
      } 
  else 
  {
        // FLAGS_duration = 1 FLAGS_strideLength = 0.1
      auto finish0 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_parse0 = finish0 - start0;//解析时长
      std::cout << "Parse0 time:" << elapsed_parse0.count() <<std::endl;
      dairlib::runDircon<double>(
          std::move(plant), plant_vis.get(), std::move(scene_graph),
          1*FLAGS_strideLength, 1*FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
          init_lc_traj, init_vc_traj);
  }
  while(solve_result==0)
  {
        for (int i = 0; i < M+N-1; i++)
        {
            init_x[i]=(x0 + .1*VectorXd::Random(nx));//初始状态轨迹在knotpoint上的值x[i]
            init_u[i]=(VectorXd::Random(nu));//u[i]
       } 
    
        auto init_x_traj_new = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
        auto init_u_traj_new = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);
        for (int j = 0; j < 2; j++) 
        {
            std::vector<MatrixXd> init_l_j;//lammda contraint force
            std::vector<MatrixXd> init_lc_j;//lammda_c force correction
            std::vector<MatrixXd> init_vc_j;//gamma velocity correction
            std::vector<double> init_time_j;//breaks
            int nP=0;
            if(j==0)
            nP=10;
            else if(j==1)
            nP=20;
            for (int i = 0; i < nP; i++) 
            {
               // init_time_j.push_back(i*.2);05263
                init_time_j.push_back(i*0.05263);
                init_l_j.push_back(init_l_vec);//0 0 20*9.81
                init_lc_j.push_back(init_l_vec);//0 0 20*9.81
                init_vc_j.push_back(VectorXd::Zero(3));//000
            }
            auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
            auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
            auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);
            init_l_traj.push_back(init_l_traj_j);//两个mode下，各十段，共二十段，
            init_lc_traj.push_back(init_lc_traj_j);//两个mode下，各十段，
            init_vc_traj.push_back(init_vc_traj_j);//两个mode下，
       }    
           dairlib::runDircon<double>(std::move(plant), plant_vis.get(), std::move(scene_graph),1*FLAGS_strideLength,
                                      1*FLAGS_duration, init_x_traj_new, init_u_traj_new, init_l_traj,init_lc_traj, init_vc_traj);
  }
}/**/