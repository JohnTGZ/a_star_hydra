#include <csignal>

#include <global_planner/a_star.h>
#include <global_planner/jps_wrapper.h>
#include <grid_map/grid_map.h> 
#include <geometry_msgs/Pose.h> 
#include <std_msgs/Empty.h> 

#include <visualization_msgs/Marker.h> 

template<typename ... Args>
std::string str_fmt( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class PlanRequest
{
public:

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    front_end_publisher_map_["front_end/closed_list"] = nh.advertise<visualization_msgs::Marker>("closed_list_viz", 10);
    front_end_plan_viz_pub_ = nh.advertise<visualization_msgs::Marker>("plan_viz", 1);

    // ROS Params
    pnh.param("front_end/a_star/max_iterations", astar_params_.max_iterations, -1);
    pnh.param("front_end/a_star/debug_viz", astar_params_.debug_viz, false);

    pnh.param("front_end/a_star/tie_breaker", astar_params_.tie_breaker, -1.0);
    pnh.param("front_end/a_star/cost_function_type", astar_params_.cost_function_type, 2);

    // Initialize map
    map_.reset(new GridMap);
    map_->initMapROS(nh, pnh);

    // Initialize front end planner
    front_end_planner_ = std::make_unique<AStarPlanner>(map_, astar_params_);
    front_end_planner_->addPublishers(front_end_publisher_map_);

    debug_start_sub_ = pnh.subscribe("debug/plan_start", 5, &PlanRequest::debugStartCB, this);
    debug_goal_sub_ = pnh.subscribe("debug/plan_goal", 5, &PlanRequest::debugGoalCB, this);
    plan_on_demand_sub_ = pnh.subscribe("plan_on_demand", 5, &PlanRequest::planOnDemandCB, this);
  }

  /**
   * @brief callback to debug topic for setting starting point of plan
   * 
   * @param msg 
   */
  void debugStartCB(const geometry_msgs::PoseConstPtr &msg)
  {
    logInfo(str_fmt("Received debug start (%f, %f, %f)", 
          msg->position.x,
          msg->position.y,
          msg->position.z));
    start_pos = Eigen::Vector3d{
          msg->position.x,
          msg->position.y,
          msg->position.z};
  }

  /**
   * @brief callback to debug goal topic for setting starting point of plan
   * 
   * @param msg 
   */
  void debugGoalCB(const geometry_msgs::PoseConstPtr &msg)
  {
    logInfo(str_fmt("Received debug goal (%f, %f, %f)", 
          msg->position.x,
          msg->position.y,
          msg->position.z));

    goal_pos = Eigen::Vector3d{
          msg->position.x,
          msg->position.y,
          msg->position.z};
  }

  /**
   * @brief callback to debug goal topic for setting starting point of plan
   * 
   * @param msg 
   */
  void planOnDemandCB(const std_msgs::EmptyConstPtr &msg)
  {
    logInfo(str_fmt("Planning on demand triggered! from (%f, %f, %f) to (%f, %f, %f)",
      start_pos(0), start_pos(1), start_pos(2),
      goal_pos(0), goal_pos(1), goal_pos(2))
    );

    requestPlan();
  }

  /**
   * @brief Request for front-end plan
   * 
   */
  bool requestPlan(){
    logInfo(str_fmt("Requesting front-end plan from (%f, %f, %f) to (%f, %f, %f)",
      start_pos(0), start_pos(1), start_pos(2),
      goal_pos(0), goal_pos(1), goal_pos(2))
    );

    // TODO: Change start_pos to subscribed drone current pose
    if (!front_end_planner_->generatePlan(start_pos, goal_pos)){
      logInfo(str_fmt("FRONT END FAILED!!!! front_end_planner_->generatePlan() from (%f, %f, %f) to (%f, %f, %f)",
        start_pos(0), start_pos(1), start_pos(2),
        goal_pos(0), goal_pos(1), goal_pos(2))
      );

      // publishClosedList(front_end_planner_->getClosedList(), "world", closed_list_viz_pub_);
      return false;
    }

    // TODO: Put front-end path in ros message and send to Hydra node.
    std::vector<Eigen::Vector3d> front_end_path = front_end_planner_->getPathPosRaw();

    // Publish front end plan
    publishFrontEndPath(front_end_path, "world", front_end_plan_viz_pub_) ;

    return true;
  }

  inline void publishFrontEndPath(const std::vector<Eigen::Vector3d>& path, const std::string& frame_id, ros::Publisher& publisher) {
    visualization_msgs::Marker wp_sphere_list, path_line_strip;
    visualization_msgs::Marker start_sphere, goal_sphere;
    double radius = 0.15;
    double alpha = 0.8; 

    geometry_msgs::Point pt;

    /* Start/goal sphere*/
    start_sphere.header.frame_id = goal_sphere.header.frame_id = frame_id;
    start_sphere.header.stamp = goal_sphere.header.stamp = ros::Time::now();
    start_sphere.ns = goal_sphere.ns = "start_end_points";
    start_sphere.type = goal_sphere.type = visualization_msgs::Marker::SPHERE;
    start_sphere.action = goal_sphere.action = visualization_msgs::Marker::ADD;
    start_sphere.id = 1;
    goal_sphere.id = 2; 
    start_sphere.pose.orientation.w = goal_sphere.pose.orientation.w = 1.0;

    start_sphere.color.r = 1.0; 
    start_sphere.color.g = 1.0; 
    start_sphere.color.b = 0.0; 
    start_sphere.color.a = goal_sphere.color.a = alpha;

    goal_sphere.color.r = 0.0;
    goal_sphere.color.g = 1.0;
    goal_sphere.color.b = 0.0;

    start_sphere.scale.x = goal_sphere.scale.x = radius;
    start_sphere.scale.y = goal_sphere.scale.y = radius;
    start_sphere.scale.z = goal_sphere.scale.z = radius;

    /* wp_sphere_list: Sphere list (Waypoints) */
    wp_sphere_list.header.frame_id = frame_id;
    wp_sphere_list.header.stamp = ros::Time::now();
    wp_sphere_list.ns = "front_end_sphere_list"; 
    wp_sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    wp_sphere_list.action = visualization_msgs::Marker::ADD;
    wp_sphere_list.id = 1; 
    wp_sphere_list.pose.orientation.w = 1.0;

    wp_sphere_list.color.r = 1.0;
    wp_sphere_list.color.g = 0.5;
    wp_sphere_list.color.b = 0.0;
    wp_sphere_list.color.a = alpha;

    wp_sphere_list.scale.x = radius;
    wp_sphere_list.scale.y = radius;
    wp_sphere_list.scale.z = radius;

    /* path_line_strip: Line strips (Connecting waypoints) */
    path_line_strip.header.frame_id = frame_id;
    path_line_strip.header.stamp = ros::Time::now();
    path_line_strip.ns = "front_end_path_lines"; 
    path_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_strip.action = visualization_msgs::Marker::ADD;
    path_line_strip.id = 1;
    path_line_strip.pose.orientation.w = 1.0;

    path_line_strip.color.r = 1.0;
    path_line_strip.color.g = 0.5;
    path_line_strip.color.b = 0.0;
    path_line_strip.color.a = alpha * 0.75;

    path_line_strip.scale.x = radius * 0.5;

    start_sphere.pose.position.x = path[0](0);
    start_sphere.pose.position.y = path[0](1);
    start_sphere.pose.position.z = path[0](2);

    pt.x = path[0](0);
    pt.y = path[0](1);
    pt.z = path[0](2);
    path_line_strip.points.push_back(pt);

    for (size_t i = 1; i < path.size()-1; i++){
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);

      wp_sphere_list.points.push_back(pt);
      path_line_strip.points.push_back(pt);
    }

    pt.x = path.back()(0);
    pt.y = path.back()(1);
    pt.z = path.back()(2);
    path_line_strip.points.push_back(pt);

    goal_sphere.pose.position.x = path.back()(0);
    goal_sphere.pose.position.y = path.back()(1);
    goal_sphere.pose.position.z = path.back()(2);

    publisher.publish(start_sphere);
    publisher.publish(goal_sphere);
    publisher.publish(wp_sphere_list);
    publisher.publish(path_line_strip);
  }

private:
  std::string node_name_{"hydra_front_end"};
  int drone_id_{0};
  Eigen::Vector3d start_pos, goal_pos;
  std::unique_ptr<PlannerBase> front_end_planner_; // Front-end planner
  std::shared_ptr<GridMap> map_;

  std::unordered_map<std::string, ros::Publisher> front_end_publisher_map_;

  JPSWrapper::JPSParams jps_params_; 
  AStarPlanner::AStarParams astar_params_; 

  /* Subscribers */
  ros::Subscriber debug_start_sub_; // DEBUG: Subscriber to user-defined start point
  ros::Subscriber debug_goal_sub_; // DEBUG: Subscriber to user-defined goal point
  ros::Subscriber plan_on_demand_sub_; // DEBUG: Subscriber to trigger planning on demand

  ros::Publisher front_end_plan_viz_pub_;
 
  void logInfo(const std::string& str){
    ROS_INFO_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logWarn(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logError(const std::string& str){
    ROS_ERROR_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logFatal(const std::string& str){
    ROS_FATAL_NAMED(node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logInfoThrottled(const std::string& str, double period){
    ROS_INFO_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logWarnThrottled(const std::string& str, double period){
    ROS_WARN_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logErrorThrottled(const std::string& str, double period){
    ROS_ERROR_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

  void logFatalThrottled(const std::string& str, double period){
    ROS_FATAL_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      drone_id_, str.c_str());
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hydra_front_end");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PlanRequest plan_req;
  plan_req.init(nh, pnh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
