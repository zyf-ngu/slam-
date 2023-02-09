# slam-navigation notebook
# 1.坐标变换与坐标系变换
   在SLAM中经常用到空间点的坐标变换。假设已获得相机某一位置在世界坐标系下的位姿poseT，pose包括相机坐标系相对于世界坐标系的旋转R和平移t，此时若已知某点p在相机坐标系下的坐标为Pc，计算点P在世界坐标系下的坐标Pw，可使用

Pw=R*Pc+t＝T*Pc

开始的时候我有这样的疑问：T表示世界坐标到相机坐标系的变换，而计算的是点坐标从相机坐标系到世界坐标系的变换结果，为什么可以直接乘呢？而不是反向的旋转的平移？其实是因为点的运动和坐标系的运动是正好相反的，所以点坐标乘上反向的变换矩阵是正好可以计算的。


（1）source、target frame是在进行坐标变换时的概念，source是坐标变换的源坐标系，target是目标坐标系。这个时候，这个变换代表的是坐标变换
（2）parent、child frame是在描述坐标系变换时的概念，parent是原坐标系，child是变换后的坐标系，这个时候这个变换描述的是坐标系变换，也是child坐标系在parent坐标系下的描述。
（3）a frame到b frame的坐标系变换（frame transform），也表示了b frame在a frame的描述，也代表了把一个点在b frame里坐标变换成在a frame里坐标的坐标变换。
（4）从parent到child的坐标系变换（frame transform）等同于把一个点从child坐标系向parent坐标系的坐标变换，等于child坐标系在parent frame坐标系的姿态描述。



# 2.move_base参数配置
## 2.1 costmap_common_params.yaml
该文件被global_costmap、local_costmap共同使用。
#---standard pioneer footprint---
#---(in meters)---
#robot_radius: 0.3#如果机器人是圆形，则指定机器人的半径；
footprint: [[-0.1,-0.10],[0.5,-0.10],[0.5,0.10],[-0.1,0.10]] #如果机器人非圆形，则指定机器人的轮廓；坐标原点为驱动电机处？

transform_tolerance: 0.2
map_type: costmap    #地图类型；
#障碍物层的参数配置
obstacle_layer:
 enabled: true   #使能障碍物层；
 obstacle_range: 15.0 # the range limit of 2d lasers# 添加障碍物范围，一方面考虑激光范围，另外范围越大越耗计算资源；
 raytrace_range: 15.0#清除障碍物范围；
 inflation_radius: 0.1
 track_unknown_space: true #true 禁止全局路径规划穿越未知区域；
 combination_method: 1
#max_obstacle_height:  0.6 #考虑的最大障碍物高度；

 observation_sources: laser_scan_sensor#数据源；
 laser_scan_sensor: {
  data_type: LaserScan, #scan数据类型；
  topic: merged_scan,#scan的话题名称；
  clearing: true, #是否根据scan清除障碍物；
  #observation_persistence: 0.0,
  inf_is_valid: true#scan的无穷远数据是否有效；
  }
#point_cloud2_sensor: {data_type: PointCloud2, topic: rslidar_points, marking: true, clearing: true, max_obstacle_height: 1.8, min_obstacle_height: 0.5}

inflation_layer:
  enabled:              true #是否使能膨胀层
  cost_scaling_factor:  10.0  # exponential rate at which the obstacle cost drops off (default: 10)#膨胀层的指数衰减速度，值越小衰减越慢(default: 10)；表示远离障碍物时，代价值的衰减系数
  inflation_radius:     0.1  # max. distance from an obstacle at which costs are incurred for planning paths.#最大有效膨胀半径，即安装指数衰减扩张的最大半径，计算障碍物cos函数时使用。

static_layer:
  enabled:              true#是否使用静态层；
  map_topic:            "/map"

## 2.2 local_costmap_params.yaml
local_costmap:
  global_frame: map     #全局坐标系
  robot_base_frame: base_link    #机器人坐标系
  update_frequency: 5.0    #局部代价地图的更新频率（内部计算使用）
  publish_frequency: 5.0   #局部代价地图的发布频率 （Rviz显示使用）；
  static_map: false # if using only static map# 是否为静态地图属性，代价地图为动态，设置为否；
  rolling_window: true   #是否位滚动窗口（随着机器人移动而滑动）；
  width: 10.0  #宽度；
  height: 10.0  #高度（长度）；
  resolution: 0.1   #栅格地图分辨率；
  transform_tolerance: 0.5    #0.5  订阅tf时的时间差冗余量；
  # footprint: [[-0.28,-0.33],[1.12,-0.33],[1.12,0.33],[-0.28,0.33]]
  
  plugins:  #局部代价地图使用的地图插件（顺序颠倒会影响效果）
   - {name: static_layer,        type: "costmap_2d::StaticLayer"}
   - {name: obstacle_layer,      type: "costmap_2d::ObstacleLayer"}
   - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}

## 2.3 global_costmap_params.yaml
global_costmap:
  global_frame: map   #全局坐标系
  robot_base_frame: base_link    #机器人基准坐标系
  update_frequency: 1.0  #更新频率（内部计算用）；
  publish_frequency: 0.25   #发布频率（Rviz显示用）；
  static_map: true  #是否位静态地图
 
  transform_tolerance: 1.0  #0.5  订阅tf时的时间差冗余量；
  plugins:#全局代价地图使用的地图插件
    - {name: static_layer,            type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer,          type: "costmap_2d::VoxelLayer"}
    - {name: inflation_layer,         type: "costmap_2d::InflationLayer"}

## 2.4  dwa-local-planner-para.yaml
DWAPlannerROS: 
 
### Robot Configuration Parameters - Kobuki 机器人配置参数，这里为Kobuki底座
  max_vel_x: 0.5  # 0.55 
  #x方向最大线速度绝对值，单位:米/秒
  min_vel_x: 0.0  
  #x方向最小线速度绝对值，单位:米/秒。如果为负值表示可以后退.
 
  max_vel_y: 0.0  # diff drive robot  
  #y方向最大线速度绝对值，单位:米/秒。turtlebot为差分驱动机器人，所以为0
  min_vel_y: 0.0  # diff drive robot  
  #y方向最小线速度绝对值，单位:米/秒。turtlebot为差分驱动机器人，所以为0
 
  max_trans_vel: 0.5 # choose slightly less than the base's capability 
  #机器人最大平移速度的绝对值，单位为 m/s
  min_trans_vel: 0.1  # this is the min trans velocity when there is negligible rotational velocity 
  #机器人最小平移速度的绝对值，单位为 m/s
  trans_stopped_vel: 0.1 
  #机器人被认属于“停止”状态时的平移速度。如果机器人的速度低于该值，则认为机器人已停止。单位为 m/s
  #Warning!
  #do not set min_trans_vel to 0.0 otherwise dwa will always think translational velocities
  #are non-negligible and small in place rotational velocities will be created.
  #注意不要将min_trans_vel设置为0，否则DWA认为平移速度不可忽略，将创建较小的旋转速度。
  max_rot_vel: 5.0  # choose slightly less than the base's capability #机器人的最大旋转角速度的绝对值，单位为 rad/s 
  min_rot_vel: 0.4  # this is the min angular velocity when there is negligible translational velocity #机器人的最小旋转角速度的绝对值，单位为 rad/s
  rot_stopped_vel: 0.4 #机器人被认属于“停止”状态时的旋转速度。单位为 rad/s
   
  acc_lim_x: 1.0 # maximum is theoretically 2.0, but we  机器人在x方向的极限加速度，单位为 meters/sec^2
  acc_lim_theta: 2.0 机器人的极限旋转加速度，单位为 rad/sec^2
  acc_lim_y: 0.0      # diff drive robot 机器人在y方向的极限加速度，对于差分机器人来说当然是0
 
### Goal Tolerance Parameters 目标距离公差参数
  yaw_goal_tolerance: 0.3  # 0.05 
  #到达目标点时，控制器在偏航/旋转时的弧度容差(tolerance)。即：到达目标点时偏行角允许的误差，单位弧度
  xy_goal_tolerance: 0.15  # 0.10 
  #到到目标点时，控制器在x和y方向上的容差（tolerence）（米）。即：到达目标点时,在xy平面内与目标点的距离误差
  #latch_xy_goal_tolerance: false 
  #设置为true时表示：如果到达容错距离内,机器人就会原地旋转；即使转动是会跑出容错距离外。
#注：这三个参数的设置及影响讨论请参考《ROS导航功能调优指南》
 
### Forward Simulation Parameters 前向模拟参数
  sim_time: 1.0       # 1.7 
  #前向模拟轨迹的时间，单位为s(seconds) 
  vx_samples: 6       # 3  
  #x方向速度空间的采样点数.
  vy_samples: 1       # diff drive robot, there is only one sample
  #y方向速度空间采样点数.。Tutulebot为差分驱动机器人，所以y方向永远只有1个值（0.0）
  vtheta_samples: 20  # 20 
  #旋转方向的速度空间采样点数.
#注：参数的设置及影响讨论请参考《ROS导航功能调优指南》
 
### Trajectory Scoring Parameters 轨迹评分参数
  path_distance_bias: 64.0      # 32.0   - weighting for how much it should stick to the global path plan
  #控制器与给定路径接近程度的权重
  
  goal_distance_bias: 24.0      # 24.0   - weighting for how much it should attempt to reach its goal
  #控制器与局部目标点的接近程度的权重，也用于速度控制
  
  occdist_scale: 0.5            # 0.01   - weighting for how much the controller should avoid obstacles
  #控制器躲避障碍物的程度
  
  forward_point_distance: 0.325 # 0.325  - how far along to place an additional scoring point
  #以机器人为中心，额外放置一个计分点的距离
  
  stop_time_buffer: 0.2         # 0.2    - amount of time a robot must stop in before colliding for a valid traj.
  #机器人在碰撞发生前必须拥有的最少时间量。该时间内所采用的轨迹仍视为有效。即：为防止碰撞,机器人必须提前停止的时间长度
 
  scaling_speed: 0.25           # 0.25   - absolute velocity at which to start scaling the robot's footprint
  #开始缩放机器人足迹时的速度的绝对值，单位为m/s。
  #在进行对轨迹各个点计算footprintCost之前，会先计算缩放因子。如果当前平移速度小于scaling_speed，则缩放因子为1.0，否则，缩放因子为(vmag - scaling_speed) / (max_trans_vel - scaling_speed) * max_scaling_factor + 1.0。然后，该缩放因子会被用于计算轨迹中各个点的footprintCost。
  #参考：https://www.cnblogs.com/sakabatou/p/8297479.html
  #亦可简单理解为：启动机器人底盘的速度.(Ref.: https://www.corvin.cn/858.html)
  
  max_scaling_factor: 0.2       # 0.2    - how much to scale the robot's footprint when at speed.
  #最大缩放因子。max_scaling_factor为上式的值的大小。
 
### Oscillation Prevention Parameters 振荡预防参数
  oscillation_reset_dist: 0.05  # 0.05   - how far to travel before resetting oscillation flags
  #机器人必须运动多少米远后才能复位震荡标记(机器人运动多远距离才会重置振荡标记)
 
### Global Plan Parameters
  #prune_plan: false
  #机器人前进是否清除身后1m外的轨迹.
  
### Debugging 调试参数
  publish_traj_pc : true #将规划的轨迹在RVIZ上进行可视化
  publish_cost_grid_pc: true 
  #将代价值进行可视化显示
  #是否发布规划器在规划路径时的代价网格.如果设置为true,那么就会在~/cost_cloud话题上发布sensor_msgs/PointCloud2类型消息.
  global_frame_id: odom #全局参考坐标系为odom
 
 
### Differential-drive robot configuration - necessary? 差分机器人配置参数
###  holonomic_robot: false 
   #是否为全向机器人。 值为false时为差分机器人； 为true时表示全向机器人

# 3.TEB参数调整
https://blog.csdn.net/zz123456zzss/article/details/104692548
https://blog.csdn.net/weixin_44917390/article/details/107568507
TebLocalPlannerROS:
  odom_topic: odom
  map_frame: map

  ##  Trajectory
  teb_autosize: True#优化期间允许改变轨迹的时域长度
  dt_ref: 0.3#局部路径规划的解析度，参考轨迹的离散间隔
  dt_hysteresis: 0.1#允许改变的时域解析度的浮动范围， 一般为 dt_ref 的 10% 左右; 
  min_samples: 3
  global_plan_overwrite_orientation: True#覆盖全局路径中局部路径点的朝向
  global_plan_viapoint_sep: 0.3 # negative, do not use viapoints. positive, use them. the actual value does not matter
  max_global_plan_lookahead_dist: 1.5
  global_plan_prune_distance: 0.6
  force_reinit_new_goal_dist: 1.0#如果上一个目标的间隔超过指定的米数（跳过热启动），则强制规划器重新初始化轨迹
  feasibility_check_no_poses: 3
  publish_feedback: false#发布包含完整轨迹和活动障碍物列表的规划器反馈
  allow_init_with_backwards_motion: true#允许在开始时先后退来执行轨迹
  exact_arc_length: false#如果为真，规划器在速度、加速度和转弯率计算中使用精确的弧长[->增加的CPU时间]，否则使用欧几里德近似
  shrink_horizon_backup: true
  shrink_horizon_min_duration: 10
其中“Trajectory”部分的参数用于调整轨迹，
“global_plan_viapoint_sep”参数调整全局路径上选取的航迹点的间隔，应根据机器人的尺寸大小调整，我们机器人的长宽为0.8*0.5(米)，这里修改为0.3，使航迹点更为紧凑；
“max_global_plan_lookahead_dist”参数在TebLocalPlannerROS::transformGlobalPlan()函数中被使用，决定局部规划初始轨迹的最大长度，实际调试发现此参数无需过大，因为局部轨迹在每个控制周期都被更新，实际执行的指令仅是轨迹上第一个点的速度值，这里设置为1.5即可，过长也可能导致优化结果无法有效收敛；
“global_plan_prune_distance”参数在TebLocalPlannerROS::pruneGlobalPlan()函数中被使用，因为全局路径是从全局起始点到全局目标点的一条轨迹，而初始的局部路径仅是从机器人当前位置到局部目标点的一小段路径，全局路径裁剪其中一部分即局部路径，该参数决定了从机器人当前位置的后面一定距离开始裁剪；
“feasibility_check_no_poses”参数在判断生成的轨迹是否冲突时使用，此时设置为3，即从轨迹起点开始逐个检查轨迹上的3个点，若3个点均不发生碰撞，则认为本次轨迹有效，由于teb优化并非硬约束，这里相当于是轨迹生成之后的一层保障，这个参数因根据机器人的速度和环境复杂程度调整，否则极有可能出现在狭窄环境中走走停停的情况；



  ##  Robot
  max_vel_x: 1.0
  max_vel_x_backwards: 0.5
  max_vel_theta: 3.14
  max_vel_y: 0.2
  acc_lim_y: 0.5
  acc_lim_x: 0.5
  acc_lim_theta: 1.57
  min_turning_radius: 0.0#车类机器人的最小转弯半径
  wheelbase: 0.0 # not used, is differential
  cmd_angle_instead_rotvel: false # not used, is differential
  footprint_model: # types: "point", "circular", "two_circles", "line", "polygon"
#type: "circular"
#radius: 0.5 # for type "circular"
#type: "line"
#line_start: [-0.0545, 0.0] # for type "line"
#line_end: [0.0545, 0.0] # for type "line"
    #front_offset: 0.2 # for type "two_circles"
    #front_radius: 0.2 # for type "two_circles"
    #rear_offset: 0.2 # for type "two_circles"
    #rear_radius: 0.2 # for type "two_circles"
    type: "polygon"
    vertices: [ [0.4, -0.25], [0.5, 0.0], [0.4, 0.25], [-0.4, 0.25], [-0.4, -0.25] ] # for type "polygon"
“Robot”部分的参数主要是根据机器人的实际情况配置最大速度和最大加速度等，不同模型的机器人（差分驱动、全向移动、阿克曼模型）有不同的配置方法。
我们使用全向移动机器人（完整模型），所以需要配置y方向的速度与加速度，并且“min_turning_radius”最小转弯半径设置为0；
另外“footprint_model”参数用于配置在优化过程中使用的机器人模型（主要是在计算障碍物距离的过程中），有"point", “circular”, “two_circles”, “line”, "polygon"这几种可选，针对每一种模型都有不同的障碍物距离计算方法，其中"point"模型是最简单的，但准确度也最低，"polygon"多边形模型最复杂，完全考虑到机器人的轮廓形状，计算准确度最高。我们这里选择"polygon"模型，然后需要设置多边形的几何参数（多边形的每一个顶点坐标），与move_base中costmap使用的几何参数一致。
footprint：每一个坐标代表机器人上的一点，设置机器人的中心为[0,0]，根据机器人不同的形状，找到机器人各凸出的坐标点即可，具体可参考下图来设置(如果是圆形底盘机器人，直接设置半径大小即可：例如 robot_radius: 0.5)；如果是长方形小车：宽736，长1590

顺时针：footprint:[[0.795, 0.368], [0.795, -0.368], [-0.795, -0.368], [-0.795, 0.368]]

五边形：footprint:[[0.795, 0.368], [0.95, 0], [0.795, -0.368], [-0.795, -0.368], [-0.795, 0.368]]

![Uploading 图片.png…]()

  ##  GoalTolerance
  xy_goal_tolerance: 0.1#目标 xy 偏移容忍度
  yaw_goal_tolerance: 0.05#目标 角度 偏移容忍度
  free_goal_vel: False

“GoalTolerance”部分的参数设置机器人停止运行的容差，根据实际情况调整。其中“free_goal_vel”参数设置机器人在目标点速度的情况，Fasle为默认最终速度为0，即到目标位置的时候应该是保持静止状态。

  ##  Obstacles
  min_obstacle_dist: 0.05 # minimum distance to obstacle: it depends on the footprint_model和障碍物最小距离
  inflation_dist: 0.0 # greater than min_obstacle_dist to take effect障碍物膨胀距离
  include_costmap_obstacles: True # use the local costmap
  costmap_obstacles_behind_robot_dist: 1.0 # distance at which obstacles behind the robot are taken into account
  legacy_obstacle_association: false
  obstacle_poses_affected: 30 # unused if legacy_obstacle_association is false
  obstacle_association_force_inclusion_factor: 10.0 # the obstacles that will be taken into account are those closer than min_obstacle_dist*factor, if legacy is false
  obstacle_association_cutoff_factor: 40.0 # the obstacles that are further than min_obstacle_dist * factor will not be taken into account, if legacy is false
  #costmap_converter_plugin: "costmap_converter::CostmapToPolygonsDBSMCCH"
  #costmap_converter_plugin: "costmap_converter::CostmapToLinesDBSRANSAC"
  #costmap_converter_plugin: "costmap_converter::CostmapToLinesDBSMCCH"
  #costmap_converter_plugin: "costmap_converter::CostmapToPolygonsDBSConcaveHull"
  costmap_converter_plugin: "" # deactivate plugin
  costmap_converter_spin_thread: True
  costmap_converter_rate: 10

“Obstacles”部分的参数用于对环境中障碍物的处理方式，体现在轨迹优化阶段。
首先是“costmap_converter_plugin”这个参数，它决定是否使用costmap_converter插件，原始costmap中障碍物全部以“点”来表示，计算机器人到障碍物的距离实际需要计算机器人到每一个“障碍物点”的距离，当环境非常复杂时计算代价会非常大。costmap_converter插件的作用是将障碍物预先表示成线段或多边形的形式，可以在一定程度上减轻后续计算距离的压力，具体介绍可见ROS wiki页面：http://wiki.ros.org/costmap_converter或中文介绍https://www.ncnynl.com/archives/201809/2604.html。
但同时这种预处理的方法也会耗费资源，需要根据实际环境的情况来判断是否启用。如果使用的话"costmap_converter::CostmapToPolygonsDBSMCCH"是一个较为精确的方法，它将环境转换为非凸多边形；在将障碍物距离加入g2o优化框架中（障碍物距离是目标函数之一，描述为超图的边），“min_obstacle_dist”参数限制机器人与障碍物的最小距离，实际还配合“obstacle_association_force_inclusion_factor”和“obstacle_association_cutoff_factor”这两个参数生效，参考*TebOptimalPlanner::AddEdgesObstacles()*函数中的如下代码：
距离小于min_obstacle_dist * obstacle_association_force_inclusion_factor值的“障碍物点”，被强制加入优化框架中，
距离大于min_obstacle_dist * obstacle_association_cutoff_factor的“障碍物点”被直接抛弃不再考虑，
然后在剩余的障碍物点中计算机器人左侧最小距离和右侧最小距离。这三个参数的设置非常重要，需要根据机器人的外形尺寸小心调整，否则极易出现狭窄空间机器人无法通过或优化不收敛的情况。

  ##  Optimization
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

  ##  Homotopy Class Planner
  enable_homotopy_class_planning: False # currently not used

