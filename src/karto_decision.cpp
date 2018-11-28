#include <thread>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "nav_msgs/Path.h"

#include "messages/GlobalPlannerAction.h"
#include "messages/LocalPlannerAction.h"

using namespace std;


enum ErrorCode{
  OK = 0,
  Error = 1,
/************************HARDWARE********************/



/***********************SOFTWARRE********************/
  /***************DRIVER******************/
    //camera
    CAMERA_ERROR = 10000,

    //lidar
    LIDAR_ERROR = 10100,


  /**************PERCEPTION***************/
    //mapping
    MAPPING_ERROR = 11000,

    //map
    MAP_ERROR = 12000,

    //localization
    LOCALIZATION_INIT_ERROR = 12100,

    //detection
    DETECTION_INIT_ERROR = 12200,


  /**************DECISION*****************/
    //decision
    DECISION_ERROR = 13000,


  /**************PLANNING*****************/

    //global planner
    GP_INITILIZATION_ERROR = 14000,
    GP_GET_POSE_ERROR,
    GP_POSE_TRANSFORM_ERROR,
    GP_GOAL_INVALID_ERROR,
    GP_PATH_SEARCH_ERROR,
    GP_MOVE_COST_ERROR,
    GP_MAX_RETRIES_FAILURE,
    GP_TIME_OUT_ERROR,


    //local planner
    LP_PLANNING_ERROR = 14100,
    LP_INITILIZATION_ERROR = 14101,
    LP_ALGORITHM_INITILIZATION_ERROR = 14102,
    LP_ALGORITHM_TRAJECTORY_ERROR= 14103,
    LP_ALGORITHM_GOAL_REACHED= 14104,
    LP_MAX_ERROR_FAILURE = 14105,
    LP_PLANTRANSFORM_ERROR = 14106,
    LP_OPTIMAL_ERROR = 14107,
    LP_VELOCITY_ERROR = 14108,
    LP_OSCILLATION_ERROR


  /**************CONTROL******************/
};

enum NodeState{
  IDLE,
  RUNNING,
  PAUSE,
  SUCCESS,
  FAILURE
};

class DecisionNode{
public:
 DecisionNode() : global_planner_actionlib_client_("global_planner_node_action", true),
                   local_planner_actionlib_client_("local_planner_node_action", true),
                   rviz_goal_(false), new_path_(false),
                   decision_state_(IDLE) {
    //point mode
    ros::NodeHandle rviz_nh("move_base_simple");
    goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              &DecisionNode::GoalCallback, this);


    global_planner_actionlib_client_.waitForServer();
    cout << "Global planner module has been connected!";

    local_planner_actionlib_client_.waitForServer();
    cout << "Local planner module has been connected!";

    thread_ = std::thread(&DecisionNode::Execution, this);
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal){
    global_planner_goal_.goal = *goal;
    rviz_goal_ = true;
  }

  void Execution(){

    while(ros::ok()) {
      if (rviz_goal_) {

        // global_planner_actionlib_client_.sendGoal(global_planner_goal_,
        //                                           boost::bind(&DecisionNode::GlobalPlannerDoneCallback, this, _1, _2),
        //                                           boost::bind(&DecisionNode::GlobalPlannerActiveCallback, this),
        //                                           boost::bind(&DecisionNode::GlobalPlannerFeedbackCallback, this, _1)
        // );
        // decision_state_ = RUNNING;
        
        path_.header.frame_id = "map";
        path_.header.stamp = ros::Time::now();
        std::vector<geometry_msgs::PoseStamped> poses;
        poses.push_back(global_planner_goal_.goal);
        path_.poses = poses;
        local_planner_goal_.route = path_;
        new_path_=true;

        rviz_goal_ = false;

      }
      //local goal
      if (new_path_) {

        local_planner_actionlib_client_.sendGoal(local_planner_goal_,
                                                 boost::bind(&DecisionNode::LocalPlannerDoneCallback, this, _1, _2),
                                                 actionlib::SimpleActionClient<messages::LocalPlannerAction>::SimpleActiveCallback(),
                                                 actionlib::SimpleActionClient<messages::LocalPlannerAction>::SimpleFeedbackCallback());
        new_path_ = false;

      }

    usleep(1);
    }

  }

  // Global Planner
  void GlobalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const messages::GlobalPlannerResultConstPtr& result){
    cout<<"Global planner "<<state.toString().c_str()<<"!"<<endl;
    decision_state_ = IDLE;
  }
  void GlobalPlannerActiveCallback(){
    cout<<"Global planner server has recived the goal!"<<endl;
  }
  void GlobalPlannerFeedbackCallback(const messages::GlobalPlannerFeedbackConstPtr& feedback){
    if (feedback->error_code != ErrorCode::OK) {
      cout<<"Global planner: "<<feedback->error_msg<<endl;
    }
    if (!feedback->path.poses.empty()) {
      local_planner_goal_.route = feedback->path;
      new_path_=true;
    }
  }

  // Local Planner
  void LocalPlannerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const messages::LocalPlannerResultConstPtr& result){
    cout<<"Local planner "<<state.toString().c_str()<<"!"<<endl;
  }

  ~DecisionNode(){

    if(thread_.joinable()){
      thread_.join();
    }

  }

private:

    actionlib::SimpleActionClient<messages::GlobalPlannerAction> global_planner_actionlib_client_;
    actionlib::SimpleActionClient<messages::LocalPlannerAction> local_planner_actionlib_client_;

    messages::GlobalPlannerGoal global_planner_goal_;
    messages::LocalPlannerGoal local_planner_goal_;

    bool rviz_goal_;
    bool new_path_;
    std::thread thread_;
    NodeState decision_state_;

    ros::Subscriber goal_sub_;
    nav_msgs::Path path_;

}


int main(int argc, char const *argv[])
{
    ros::init(argc, argv, "karto_decision");


    ros::spin();
    return 0;
}
