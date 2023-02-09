# slam-navigation notebook
#1.坐标变换与坐标系变换
#2.TEB参数调整
TebLocalPlannerROS:
  odom_topic: odom
  map_frame: map

  # Trajectory
  teb_autosize: True
  dt_ref: 0.3
  dt_hysteresis: 0.1
  min_samples: 3
  global_plan_overwrite_orientation: True
  global_plan_viapoint_sep: 0.3 # negative, do not use viapoints. positive, use them. the actual value does not matter
  max_global_plan_lookahead_dist: 1.5
  global_plan_prune_distance: 0.6
  force_reinit_new_goal_dist: 1.0
  feasibility_check_no_poses: 3
  publish_feedback: false
  allow_init_with_backwards_motion: true
  exact_arc_length: false
  shrink_horizon_backup: true
  shrink_horizon_min_duration: 10
其中“Trajectory”部分的参数用于调整轨迹，
“global_plan_viapoint_sep”参数调整全局路径上选取的航迹点的间隔，应根据机器人的尺寸大小调整，我们机器人的长宽为0.8*0.5(米)，这里修改为0.3，使航迹点更为紧凑；
“max_global_plan_lookahead_dist”参数在TebLocalPlannerROS::transformGlobalPlan()函数中被使用，决定局部规划初始轨迹的最大长度，实际调试发现此参数无需过大，因为局部轨迹在每个控制周期都被更新，实际执行的指令仅是轨迹上第一个点的速度值，这里设置为1.5即可，过长也可能导致优化结果无法有效收敛；
“global_plan_prune_distance”参数在TebLocalPlannerROS::pruneGlobalPlan()函数中被使用，因为全局路径是从全局起始点到全局目标点的一条轨迹，而初始的局部路径仅是从机器人当前位置到局部目标点的一小段路径，全局路径裁剪其中一部分即局部路径，该参数决定了从机器人当前位置的后面一定距离开始裁剪；
“feasibility_check_no_poses”参数在判断生成的轨迹是否冲突时使用，此时设置为3，即从轨迹起点开始逐个检查轨迹上的3个点，若3个点均不发生碰撞，则认为本次轨迹有效，由于teb优化并非硬约束，这里相当于是轨迹生成之后的一层保障，这个参数因根据机器人的速度和环境复杂程度调整，否则极有可能出现在狭窄环境中走走停停的情况；



  # Robot
  max_vel_x: 1.0
  max_vel_x_backwards: 0.5
  max_vel_theta: 3.14
  max_vel_y: 0.2
  acc_lim_y: 0.5
  acc_lim_x: 0.5
  acc_lim_theta: 1.57
  min_turning_radius: 0.0
  wheelbase: 0.0 # not used, is differential
  cmd_angle_instead_rotvel: false # not used, is differential
  footprint_model: # types: "point", "circular", "two_circles", "line", "polygon"
#    type: "circular"
#    radius: 0.5 # for type "circular"
#    type: "line"
#    line_start: [-0.0545, 0.0] # for type "line"
#    line_end: [0.0545, 0.0] # for type "line"
    #front_offset: 0.2 # for type "two_circles"
    #front_radius: 0.2 # for type "two_circles"
    #rear_offset: 0.2 # for type "two_circles"
    #rear_radius: 0.2 # for type "two_circles"
    type: "polygon"
    vertices: [ [0.4, -0.25], [0.5, 0.0], [0.4, 0.25], [-0.4, 0.25], [-0.4, -0.25] ] # for type "polygon"
“Robot”部分的参数主要是根据机器人的实际情况配置最大速度和最大加速度等，不同模型的机器人（差分驱动、全向移动、阿克曼模型）有不同的配置方法。
我们使用全向移动机器人（完整模型），所以需要配置y方向的速度与加速度，并且“min_turning_radius”最小转弯半径设置为0；
另外“footprint_model”参数用于配置在优化过程中使用的机器人模型（主要是在计算障碍物距离的过程中），有"point", “circular”, “two_circles”, “line”, "polygon"这几种可选，针对每一种模型都有不同的障碍物距离计算方法，其中"point"模型是最简单的，但准确度也最低，"polygon"多边形模型最复杂，完全考虑到机器人的轮廓形状，计算准确度最高。我们这里选择"polygon"模型，然后需要设置多边形的几何参数（多边形的每一个顶点坐标），与move_base中costmap使用的几何参数一致。

  # GoalTolerance
  xy_goal_tolerance: 0.1
  yaw_goal_tolerance: 0.05
  free_goal_vel: False

“GoalTolerance”部分的参数设置机器人停止运行的容差，根据实际情况调整。其中“free_goal_vel”参数设置机器人在目标点速度的情况，Fasle为默认最终速度为0，即到目标位置的时候应该是保持静止状态。

  # Obstacles
  min_obstacle_dist: 0.05 # minimum distance to obstacle: it depends on the footprint_model
  inflation_dist: 0.0 # greater than min_obstacle_dist to take effect
  include_costmap_obstacles: True # use the local costmap
  costmap_obstacles_behind_robot_dist: 1.0 # distance at which obstacles behind the robot are taken into account
  legacy_obstacle_association: false
  obstacle_poses_affected: 30 # unused if legacy_obstacle_association is false
  obstacle_association_force_inclusion_factor: 10.0 # the obstacles that will be taken into account are those closer than min_obstacle_dist*factor, if legacy is false
  obstacle_association_cutoff_factor: 40.0 # the obstacles that are further than min_obstacle_dist * factor will not be taken into account, if legacy is false
#  costmap_converter_plugin: "costmap_converter::CostmapToPolygonsDBSMCCH"
  #costmap_converter_plugin: "costmap_converter::CostmapToLinesDBSRANSAC"
  #costmap_converter_plugin: "costmap_converter::CostmapToLinesDBSMCCH"
#  costmap_converter_plugin: "costmap_converter::CostmapToPolygonsDBSConcaveHull"
  costmap_converter_plugin: "" # deactivate plugin
  costmap_converter_spin_thread: True
  costmap_converter_rate: 10

“Obstacles”部分的参数用于对环境中障碍物的处理方式，体现在轨迹优化阶段。
首先是“costmap_converter_plugin”这个参数，它决定是否使用costmap_converter插件，原始costmap中障碍物全部以“点”来表示，计算机器人到障碍物的距离实际需要计算机器人到每一个“障碍物点”的距离，当环境非常复杂时计算代价会非常大。costmap_converter插件的作用是将障碍物预先表示成线段或多边形的形式，可以在一定程度上减轻后续计算距离的压力，具体介绍可见ROS wiki页面：http://wiki.ros.org/costmap_converter或中文介绍https://www.ncnynl.com/archives/201809/2604.html。
但同时这种预处理的方法也会耗费资源，需要根据实际环境的情况来判断是否启用。如果使用的话"costmap_converter::CostmapToPolygonsDBSMCCH"是一个较为精确的方法，它将环境转换为非凸多边形；在将障碍物距离加入g2o优化框架中（障碍物距离是目标函数之一，描述为超图的边），“min_obstacle_dist”参数限制机器人与障碍物的最小距离，实际还配合“obstacle_association_force_inclusion_factor”和“obstacle_association_cutoff_factor”这两个参数生效，参考*TebOptimalPlanner::AddEdgesObstacles()*函数中的如下代码：
距离小于min_obstacle_dist * obstacle_association_force_inclusion_factor值的“障碍物点”，被强制加入优化框架中，
距离大于min_obstacle_dist * obstacle_association_cutoff_factor的“障碍物点”被直接抛弃不再考虑，
然后在剩余的障碍物点中计算机器人左侧最小距离和右侧最小距离。这三个参数的设置非常重要，需要根据机器人的外形尺寸小心调整，否则极易出现狭窄空间机器人无法通过或优化不收敛的情况。

  # Optimization
  no_inner_iterations: 5
  no_outer_iterations: 4
  optimization_activate: True # optimize
  optimization_verbose: False
  penalty_epsilon: 0.1
  weight_max_vel_x: 2
  weight_max_vel_y: 1
  weight_max_vel_theta: 1
  weight_acc_lim_x: 1
  weight_acc_lim_y: 1
  weight_acc_lim_theta: 1
  weight_kinematics_nh: 1 # is a holonomic robot
  weight_kinematics_forward_drive: 10 # prefer forward driving, for differential
  weight_kinematics_turning_radius: 0 # prefer turns that respect the min_turning_radius, not used if differential (min_turning_radius = 0)
  weight_optimaltime: 1.0 # prefer trajectories with less transition time
  weight_obstacle: 50.0 # prefer trajectories that respect the min_obstacle_dist
  weight_inflation: 0.1 # prefer trajectories that respect the inflation of the obstacles
  #weight_dynamic_obstacle: 10 # not in use yet
  weight_viapoint: 1.0 # prefer trajectories that respect the viapoints in the global path
  weight_adapt_factor: 2 # factor to multiply some weights (currently only weight_obstacle) at each iteration (gives better results than a huge value for the weight)
“Optimization”部分的参数主要是设置优化框架中各部分的权重大小，其中“weight_kinematics_nh”参数应设置较小值，因为我们是完整约束机器人无需限制其运动学约束。

  # Homotopy Class Planner
  enable_homotopy_class_planning: False # currently not used

