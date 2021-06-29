cmake_minimum_required(VERSION 2.8.3)
project(dairlib)
message("PROJECT_NAME!!!=${PROJECT_NAME}") 
SET(LIBRARY_OUTPUT_PATH "/home/cby/drake_learning/src/drake_learning2/lib")
message("LIBRARY_OUTPUT_PATH=${LIBRARY_OUTPUT_PATH}")
SET(LIBRARY_OUTPUT_PATH "/home/cby/drake_learning/src/drake_learning2/build/lib")#为什么LIBRARY_OUTPUT_PATH位置不同作用不同
 set(multibodyutils_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/multibody/multibody_utils.cc")
 set(world_point_evaluator_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/multibody/kinematic/world_point_evaluator.cc")
 set(kinematic_evaluator_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/multibody/kinematic/kinematic_evaluator.cc")
 set(constraint_factory_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/solvers/constraint_factory.cc")
 set(nonlinear_constraint_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/solvers/nonlinear_constraint.cc")
 set(kinematic_evaluator_set_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/multibody/kinematic/kinematic_evaluator_set.cc")
 set(kinematic_constraints_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/multibody/kinematic/kinematic_constraints.cc")
 set(dircon_mode_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/systems/trajectory_optimization/dircon/dircon_mode.cc")
 set(multipose_visualizer_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/multibody/multipose_visualizer.cc")
 set(dynamics_cache_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/systems/trajectory_optimization/dircon/dynamics_cache.cc")
 set(dircon_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/systems/trajectory_optimization/dircon/dircon.cc")
 set(dircon_opt_constraints_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/systems/trajectory_optimization/dircon/dircon_opt_constraints.cc")
 set(find_resource_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/common/find_resource.cc")
 set(spruce_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/third_party/spruce/spruce.cc")
 set(visualization_utils_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/multibody/visualization_utils.cc")
 set(subvector_pass_through_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/systems/primitives/subvector_pass_through.cc")
 set(file_utils_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/common/file_utils.cc")
 set(cassie_utils_path "/home/cby/drake_learning/src/drake_learning2/examples/Cassie/cassie_utils.cc")
 set(sim_cassie_sensor_aggregator_path "/home/cby/drake_learning/src/drake_learning2/examples/Cassie/sim_cassie_sensor_aggregator.cc")
 set(timestamped_vector_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/systems/framework/timestamped_vector.cc")
 set(output_vector_path "/home/cby/drake_learning/src/drake_learning2/dairlib/src/systems/framework/output_vector.cc")

message("PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")
message("PROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/multibody")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/multibody/kinematic")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/multibody/solvers")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/systems/trajectory_optimization/dircon")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/common")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/third_party/spruce")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/systems/primitives")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/examples/Cassie")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/systems/framework")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("/home/cby/drake_learning/src/drake_learning2/dairlib/include/systems/framework")
INCLUDE_DIRECTORIES("/home/cby/drake_learning/src/drake_learning2/examples/Cassie")

find_package(drake REQUIRED )#PATHS ${drake_DIR}
message("drake_FOUND=${drake_FOUND}")
message("drake_DIR=${drake_DIR}")
set(ros_drake_LIBRARIES ${drake_LIBRARIES})
message("ros_drake_LIBRARIESxxxxxx=${ros_drake_LIBRARIES}")
include_directories(
  ${ros_drake_LIBRARIES}
)

#ADD_LIBRARY(multibodyutils SHARED ${DAIRlib_src} ) 
ADD_LIBRARY(multibodyutils2 SHARED ${multibodyutils_path} )
target_link_libraries(multibodyutils2
    ${ros_drake_LIBRARIES}
 )

ADD_LIBRARY(kinematicevaluator2 SHARED ${kinematic_evaluator_path} )
target_link_libraries(kinematicevaluator2
    ${ros_drake_LIBRARIES}
 )


ADD_LIBRARY(worldpointevaluator3 SHARED ${world_point_evaluator_path} )
target_link_libraries(worldpointevaluator3
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluator2.so
 )
add_dependencies(worldpointevaluator3 kinematicevaluator2)



ADD_LIBRARY(constraintfactory2 SHARED ${constraint_factory_path} )
target_link_libraries(constraintfactory2
    ${ros_drake_LIBRARIES}
 )

ADD_LIBRARY(kinematicevaluatorset2 SHARED ${kinematic_evaluator_set_path} )
target_link_libraries(kinematicevaluatorset2
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluator2.so
 )
add_dependencies(kinematicevaluatorset2 kinematicevaluator2)

ADD_LIBRARY(dirconmode2 SHARED ${dircon_mode_path} )
target_link_libraries(dirconmode2
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluatorset2.so
 )
add_dependencies(dirconmode2 kinematicevaluatorset2)


ADD_LIBRARY(multiposevisualizer2 SHARED ${multipose_visualizer_path} )
target_link_libraries(multiposevisualizer2
    ${ros_drake_LIBRARIES}
 )


ADD_LIBRARY(dynamicscache2 SHARED ${dynamics_cache_path} )
target_link_libraries(dynamicscache2
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluatorset2.so
 )
add_dependencies(dynamicscache2 kinematicevaluatorset2)

ADD_LIBRARY(dircon2 SHARED ${dircon_path} )
target_link_libraries(dircon2
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libdirconmode2.so 
${PROJECT_SOURCE_DIR}/lib/libdynamicscache2.so 
${PROJECT_SOURCE_DIR}/lib/libmultiposevisualizer2.so
 )
add_dependencies(dircon2 dirconmode2 dynamicscache2 multiposevisualizer2)

ADD_LIBRARY(kinematicconstraints2 SHARED ${kinematic_constraints_path} )
target_link_libraries(kinematicconstraints2
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libnonlinearconstraint2.so
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluatorset2.so
 )


ADD_LIBRARY(nonlinearconstraint2 SHARED ${nonlinear_constraint_path} )
target_link_libraries(nonlinearconstraint2
    ${ros_drake_LIBRARIES}
 )

ADD_LIBRARY(dirconoptconstraints2 SHARED ${dircon_opt_constraints_path} )
target_link_libraries(dirconoptconstraints2
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libnonlinearconstraint2.so
${PROJECT_SOURCE_DIR}/lib/libdynamicscache2.so  
${PROJECT_SOURCE_DIR}/lib/libdirconmode2.so   
 )
add_dependencies(dirconoptconstraints2 dirconmode2)

ADD_LIBRARY(spruce2 SHARED ${spruce_path} )
target_link_libraries(spruce2
    ${ros_drake_LIBRARIES}  
 )

ADD_LIBRARY(findresource2 SHARED ${find_resource_path} )
target_link_libraries(findresource2
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libspruce2.so   
 )
add_dependencies(findresource2 spruce2)
ADD_LIBRARY(subvectorpassthrough2 SHARED ${subvector_pass_through_path} )
target_link_libraries(subvectorpassthrough2
    ${ros_drake_LIBRARIES} 
 )


ADD_LIBRARY(visualizationutils2 SHARED ${visualization_utils_path} )
target_link_libraries(visualizationutils2
    ${ros_drake_LIBRARIES} 
${PROJECT_SOURCE_DIR}/lib/libsubvectorpassthrough2.so
 )
add_dependencies(visualizationutils2 subvectorpassthrough2)

ADD_LIBRARY(fileutils2 SHARED ${file_utils_path} )
target_link_libraries(fileutils2
    ${ros_drake_LIBRARIES} 
 )

ADD_LIBRARY(timestampedvector2 SHARED ${timestamped_vector_path} )
target_link_libraries(timestampedvector2
    ${ros_drake_LIBRARIES} 
 )
##编译cassie project失败，找不到一些hpp文件
##ADD_LIBRARY(simcassiesensoraggregator SHARED ${sim_cassie_sensor_aggregator_path} )
##target_link_libraries(simcassiesensoraggregator
##   ${ros_drake_LIBRARIES} 
##${PROJECT_SOURCE_DIR}/build/lib/libtimestampedvector.so
##${PROJECT_SOURCE_DIR}/build/lib/liboutputvector.so
## )



ADD_LIBRARY(outputvector2 SHARED ${output_vector_path} )
target_link_libraries(outputvector2
    ${ros_drake_LIBRARIES} 
 )

#ADD_LIBRARY(cassieutils SHARED ${cassie_utils_path} )
#target_link_libraries(cassieutils
#    ${ros_drake_LIBRARIES} 
# )


