#include <goal_manager.h>

// maybe I have to add a namespace here

// TODO : add defined name here
const std::string GoalManager::kCancelGoalSubName_ = "cancel_goal";
const std::string GoalManager::kNewGoalSubName_ = "new_goal";
const std::string GoalManager::kNewGoalStampedSubName_ = "new_goal_stamped";
const std::string GoalManager::kGoalSequenceKey_ = "goal_sequence";
const std::string GoalManager::kActionLibServername_ = "move_base";
const int GoalManager::kSleepTime_ = 100000; // u sec

GoalManager::GoalManager(ros::NodeHandle n)
  : nh_(n), ind_(1) {
  ROS_INFO_STREAM("Goal Manager Init...");
  new_goal_stamped_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
    kNewGoalStampedSubName_, 1, boost::bind(&GoalManager::NewGoalStampedSubCbk, this, _1));
  new_goal_sub_ = nh_.subscribe<geometry_msgs::Pose>(
    kNewGoalSubName_, 1, boost::bind(&GoalManager::NewGoalSubCbk, this, _1));
  cancel_goal_sub_ = nh_.subscribe<std_msgs::String>(
    kCancelGoalSubName_, 1, boost::bind(&GoalManager::CancelGoalSubCbk, this, _1));

  ROS_INFO_STREAM("Start ActionLib");
  // Connect to the move_base action server
  action_client_ = new ActionClient(kActionLibServername_, true); // create a thread to handle subscriptions.
  action_client_->waitForServer();
  ROS_INFO("Server OK");

  GoalSendingThread_.reset(
    new boost::thread(boost::bind(&GoalManager::GoalSending, this)) );

  XmlRpc::XmlRpcValue yml;
  if (!nh_.getParam(kGoalSequenceKey_, yml)) {
    ROS_ERROR_STREAM("get " << kGoalSequenceKey_ << " error");
    return;
  }
  // push form backward so that it can pop from the front
  for (int i = yml.size() - 1; i >= 0; i--) {
    Point2D pose_tmp(yml[i][0], yml[i][1], yml[i][2]);
    param_goal_vector_.push_back(pose_tmp);
  }
  cond_.notify_all();
  ROS_INFO_STREAM("Goal Manager Init...OK...");
}

  // usage functions
void GoalManager::NewGoalStampedSubCbk(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header = goal->header;
  pose_tmp.pose = goal->pose;
  ROS_INFO_STREAM("rviz came" << goal->pose.position.x);
  mtx_.lock();
  goal_vector_.push_back(pose_tmp);
  mtx_.unlock();
  cond_.notify_all();
}

void GoalManager::NewGoalSubCbk(const geometry_msgs::Pose::ConstPtr& goal) {
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header.stamp = ros::Time::now();
  pose_tmp.pose.position = goal->position;
  pose_tmp.pose.orientation = goal->orientation;
  mtx_.lock();
  goal_vector_.push_back(pose_tmp);
  mtx_.unlock();
  cond_.notify_all();
}

bool GoalManager::IsGoalVectorsEmpty() {
  int size = 0;
  size = param_goal_vector_.size();
  mtx_.lock();
  size += goal_vector_.size();
  mtx_.unlock();
  if (size == 0)
    return true;
  else
    return false;
}

void GoalManager::ReleaseGoalVectors() {
    // release goal vector
    param_goal_vector_.clear();
    goal_vector_.clear();
    std::vector<geometry_msgs::PoseStamped>().swap(goal_vector_);
    std::vector<Point2D>().swap(param_goal_vector_);
}

void GoalManager::CancelGoalSubCbk(const std_msgs::String::ConstPtr& cancel) {
  ReleaseGoalVectors();
  action_client_->cancelAllGoals();
}

void GoalManager::GoalSending() {
  while(1) {
    boost::unique_lock<boost::mutex> lock{mtx_notify_};
    if (IsGoalVectorsEmpty()) {
      ROS_INFO_STREAM("wait");
      cond_.wait(mtx_notify_);
    }
    Point2D point_tmp;
    move_base_msgs::MoveBaseGoal goal_tmp;
    goal_tmp.target_pose.header.frame_id = "map";
    // set goal
    point_tmp = param_goal_vector_.back();
    param_goal_vector_.pop_back();
    goal_tmp.target_pose.pose.position.x = point_tmp.x_;
    goal_tmp.target_pose.pose.position.y = point_tmp.y_;
    goal_tmp.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(point_tmp.th_);
    goal_tmp.target_pose.header.stamp = ros::Time::now();
    // the piority of  goal_vector is higher than param_goal_vector

    // send goal
    action_client_->sendGoal(goal_tmp);
    ROS_INFO_STREAM("Sending Goal: " << point_tmp.x_ << ", " << point_tmp.y_ << ", " << point_tmp.th_);
    // wait for result
    // It will get out the loop when not able to get a plan.
    while (action_client_->waitForResult(ros::Duration(1, 0)) == false) {
      if (!nh_.ok()) // exit if ros node is closed. (by pressing ctrl+c)
        exit(0);
    }
    // if don't cancel all the goals, the program will go to next goal after
    // reach the current goal
    if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Goal Reached!!");
    } else {
      ROS_INFO("Goal Timout!!");
    }
    usleep(kSleepTime_);
  }
}

  // test functions
void GoalManager::ParamGoalVectorPrintTest() {
  for (int i = 0; i < param_goal_vector_.size(); i++) {
    Point2D _tmp;
    _tmp = param_goal_vector_[i];
    ROS_INFO_STREAM("x: " << _tmp.x_ << ", y: " << _tmp.y_ << ", th: " << _tmp.th_);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_manager");
  ros::NodeHandle nh;
  GoalManager gm(nh);
  ros::spin();
  return 0;
}
