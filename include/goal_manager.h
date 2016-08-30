#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <queue>

// maybe I have to add a namespace here

struct Point2D {
  Point2D() : x_(0), y_(0), th_(0) {
  };
  Point2D(double x, double y, double th)
    : x_(x), y_(y), th_(th) {
  };
  double x_;
  double y_;
  double th_;
};

class GoalManager {
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;
 public:
  GoalManager(ros::NodeHandle n);
  ~GoalManager() {
    ReleaseGoalVectors();
    // release actionlib
    delete action_client_;
  };

  void GoalSending();

  // test functions
  void ParamGoalVectorPrintTest();

 private:
  void NewGoalStampedSubCbk(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void NewGoalSubCbk(const geometry_msgs::Pose::ConstPtr& goal);
  void CancelGoalSubCbk(const std_msgs::String::ConstPtr& cancel);
  void ReleaseGoalVectors();
  bool IsGoalVectorsEmpty();
  ros::NodeHandle nh_;

  // test
  int ind_;
  bool is_doing_topic_goal;
  std::queue<geometry_msgs::PoseStamped> goal_vector_;
  std::vector<Point2D> param_goal_vector_;
  static const int kSleepTime_;
  static const std::string kNewGoalSubName_;
  static const std::string kNewGoalStampedSubName_;
  static const std::string kCancelGoalSubName_;
  static const std::string kGoalSequenceKey_;
  static const std::string kActionLibServername_;
  static const std::string kGoalFrameId_;

  boost::shared_ptr<boost::thread> GoalSendingThread_;
  boost::mutex mtx_;
  boost::mutex mtx_notify_;
  boost::condition_variable_any cond_;

  ActionClient* action_client_;
  ros::Subscriber new_goal_sub_;
  ros::Subscriber new_goal_stamped_sub_;
  ros::Subscriber cancel_goal_sub_;
};
