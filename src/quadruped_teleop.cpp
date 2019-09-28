#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <free_gait_msgs/ExecuteStepsGoal.h>
#include <free_gait_msgs/BaseTarget.h>
#include <free_gait_msgs/BaseAuto.h>
#include <free_gait_msgs/Footstep.h>
#include <free_gait_msgs/EndEffectorTarget.h>
#include <free_gait_msgs/ExecuteStepsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <kindr_ros/kindr_ros.hpp>
#include <tf/transform_listener.h>
#include <kindr/Core>

class QuadrupedTeleOp
{
public:
    QuadrupedTeleOp(ros::NodeHandle& nodehandle);
    void publishCommand();
    void publishDirectJointCommand();
    int directJointFlag_;
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void commandUpdate();

  ros::NodeHandle nh_;

  boost::thread commandUpdateThread_;

  int eStopFlag_, setInitialStateFlag_, legMoveFlag_, trotFlag_, paceFlag_, standFlag_, count_;
  geometry_msgs::Pose target_pose_;
  geometry_msgs::Point target_position_, target_euler_, lf_pos_, rf_pos_, rh_pos_, lh_pos_;
  double l_scale_, a_scale_;
  std::string actionServerTopic_;
  geometry_msgs::Twist twist, legTwist;

  free_gait_msgs::ExecuteStepsGoal step_goal;
  float leftStickLR ,leftStickUD,LT,rightStickLR ,rightStickUD, RT, crossKeyLR, crossKeyUD;//1/-1
  int keyA, keyB, keyX, keyY, LB, RB, keyBack, keyStart, keyPower;//0/1

  ros::Publisher vel_pub_, eStopPublisher_, legMovePub_, cartesianDiffPub_,jointCommandPub_, homingCommandPub_, emergencyStopPub_, resetCommandPub_;
  ros::Subscriber joy_sub_;
  //! Step action client.
  std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>> stepActionClient_;

  tf::TransformListener baseTFListener_;

  tf::StampedTransform baseToOdomTransform_, footprintToOdomTransform_;
  kindr::HomTransformQuatD base_to_odom_, footprint_to_odom_;
};


QuadrupedTeleOp::QuadrupedTeleOp(ros::NodeHandle& nodehandle):
  directJointFlag_(0),
  nh_(nodehandle),
  eStopFlag_(0),
  legMoveFlag_(0),
  setInitialStateFlag_(0)
{

  trotFlag_ = 0;
  paceFlag_ = 0;
  standFlag_ = 0;

  lh_pos_.x = -0.42;
  lh_pos_.y = 0.25;
  lh_pos_.z = 0.1;

  rh_pos_.x = -0.42;
  rh_pos_.y = -0.25;
  rh_pos_.z = 0.1;

  lf_pos_.x = 0.42;
  lf_pos_.y = 0.25;
  lf_pos_.z = 0.1;

  rf_pos_.x = 0.42;
  rf_pos_.y = -0.25;
  rf_pos_.z = 0.1;

  nh_.param("scale_angular", a_scale_, 1.0);
  nh_.param("scale_linear", l_scale_, 1.0);
  nh_.param("action_server_topic", actionServerTopic_,
                    std::string("/free_gait/action_server"));
  ROS_INFO("augular scale is %f, linear scale is %f",a_scale_,l_scale_);

  stepActionClient_ = std::unique_ptr<
      actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>>(
      new actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>(
          actionServerTopic_, true));
  eStopPublisher_ = nh_.advertise<std_msgs::Bool>("/e_stop", 1);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &QuadrupedTeleOp::joyCallback, this);

  commandUpdateThread_ = boost::thread(boost::bind(&QuadrupedTeleOp::commandUpdate, this));
}

void QuadrupedTeleOp::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  free_gait_msgs::ExecuteStepsGoal step_goal;
  leftStickLR = joy->axes[0];//contiunus 1~-1
  leftStickUD = joy->axes[1];//contiunus 1~-1
  LT = 1 - joy->axes[2];//contiunus 0~2
  rightStickLR = joy->axes[3];//contiunus 1~-1
  rightStickUD = joy->axes[4];//contiunus 1~-1
  RT = 1 - joy->axes[5];//contiunus 0~2
  crossKeyLR = joy->axes[6];//1/-1
  crossKeyUD = joy->axes[7];//1/-1
  keyA = joy->buttons[0];//0/1
  keyB = joy->buttons[1];//0/1
  keyX = joy->buttons[2];//0/1
  keyY = joy->buttons[3];//0/1
  LB = joy->buttons[4];//0/1
  RB = joy->buttons[5];//0/1
  keyBack = joy->buttons[6];//0/1
  keyStart = joy->buttons[7];//0/1
  keyPower = joy->buttons[8];//0/1

  float velocityScale = l_scale_*RT +1;
  float angularScale = a_scale_*LT +1;

  if(keyStart == 1)
  {
      ROS_INFO("Reset Emergency Stop");
      std_msgs::Bool e_stop_msg;
      e_stop_msg.data = false;

      eStopPublisher_.publish(e_stop_msg);

      eStopFlag_ = 0;
  }
  if(keyPower == 1)
  {
      ROS_INFO("Emergency Stop");
      std_msgs::Bool e_stop_msg;
      e_stop_msg.data = true;

      eStopPublisher_.publish(e_stop_msg);
      eStopFlag_ = 1;
      return;
  }


}


void QuadrupedTeleOp::commandUpdate()
{
  ros::Rate rate(10);
  count_ = 0;
  while (!eStopFlag_ && ros::ok()) {

      try {
        baseTFListener_.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.5));
        baseTFListener_.lookupTransform("/odom", "/base_link", ros::Time(0), baseToOdomTransform_);
        baseTFListener_.waitForTransform("/odom", "/foot_print", ros::Time(0), ros::Duration(0.5));
        baseTFListener_.lookupTransform("/odom", "/foot_print", ros::Time(0), footprintToOdomTransform_);
        base_to_odom_ = kindr::HomTransformQuatD(kindr::Position3D(baseToOdomTransform_.getOrigin().getX(),
                                                          baseToOdomTransform_.getOrigin().getY(),
                                                          baseToOdomTransform_.getOrigin().getZ()),
                                                 kindr::RotationQuaternionD(baseToOdomTransform_.getRotation().getW(),
                                                                            baseToOdomTransform_.getRotation().getX(),
                                                                            baseToOdomTransform_.getRotation().getY(),
                                                                            baseToOdomTransform_.getRotation().getZ()));

        footprint_to_odom_ = kindr::HomTransformQuatD(kindr::Position3D(footprintToOdomTransform_.getOrigin().getX(),
                                                          footprintToOdomTransform_.getOrigin().getY(),
                                                          footprintToOdomTransform_.getOrigin().getZ()),
                                                 kindr::RotationQuaternionD(footprintToOdomTransform_.getRotation().getW(),
                                                                            footprintToOdomTransform_.getRotation().getX(),
                                                                            footprintToOdomTransform_.getRotation().getY(),
                                                                            footprintToOdomTransform_.getRotation().getZ()));
      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
      }

      if(keyBack == 1)
        {
          ROS_INFO("Reset Initial Quadruped State");
          free_gait_msgs::Step initial_steps;
          free_gait_msgs::BaseAuto base_auto_msg;
          base_auto_msg.height = 0.25;
          base_auto_msg.average_linear_velocity = 0.2;
          base_auto_msg.average_angular_velocity = 0.1;
          initial_steps.base_auto.push_back(base_auto_msg);
          step_goal.steps.push_back(initial_steps);

          stepActionClient_->sendGoal(step_goal);
          stepActionClient_->waitForResult();
          setInitialStateFlag_ = 1;
        }

    //    linearX_ = 0;
    //    linearY_ = 0;
    //    angular_ = 0;

      //! WSHY: controller switch
      if(setInitialStateFlag_)
        {

        }

      //! WSHY: set gait
      if(crossKeyLR == 1)
        {
          // Trot
          ROS_INFO("Set Trot Flag");
          trotFlag_ = 1;
          paceFlag_ = 0;
          standFlag_ = 0;
        }else if (crossKeyLR == -1) {
          // Pace
          ROS_INFO("Set Pace Flag");
          trotFlag_ = 0;
          paceFlag_ = 1;
          standFlag_ = 0;
        }else if (crossKeyUD == 1) {
          // Stand and stop gait
          ROS_INFO("Set Stand Flag");
          trotFlag_ = 0;
          paceFlag_ = 0;
          standFlag_ = 1;
        }
      //! WSHY: Set velocity
      if(trotFlag_)
        {
          geometry_msgs::Twist base_vel;
          base_vel.linear.x = leftStickUD;
          base_vel.linear.y = leftStickLR;
          base_vel.linear.z = rightStickUD;// react to value near the limits to avoiding unexpected trigger
          base_vel.angular.z = rightStickLR;
          vel_pub_.publish(base_vel);
          ROS_INFO("set Trot Velocity : vx = %f, vy = %f, vz = %f, wz = %f", base_vel.linear.x,
                   base_vel.linear.y, base_vel.linear.z, base_vel.angular.z);
        }
      if(paceFlag_)
        {
          geometry_msgs::Twist base_vel;
          base_vel.linear.x = leftStickUD/2;
          base_vel.linear.y = leftStickLR/2;
          base_vel.linear.z = rightStickUD;// react to value near the limits to avoiding unexpected trigger
          base_vel.angular.z = rightStickLR/2;
          vel_pub_.publish(base_vel);
          ROS_INFO("set Pace Velocity : vx = %f, vy = %f, vz = %f, wz = %f", base_vel.linear.x,
                   base_vel.linear.y, base_vel.linear.z, base_vel.angular.z);
        }
      if(standFlag_)
        {
          step_goal.steps.clear();
          free_gait_msgs::Step steps;
          free_gait_msgs::BaseTarget base_target_msg;
          free_gait_msgs::BaseAuto base_auto_msg;
          free_gait_msgs::EndEffectorTarget end_effector_msg;
          free_gait_msgs::Footstep footstep_msg;
          free_gait_msgs::LegMode leg_mode_msg;
          base_target_msg.target.header.frame_id = "odom";
          geometry_msgs::Quaternion target_q;
          geometry_msgs::Point target_position;
          kindr_ros::convertToRosGeometryMsg(base_to_odom_.getPosition(), target_position);
          kindr_ros::convertToRosGeometryMsg(base_to_odom_.getRotation(), target_q);
          if(RB==1)
            {
            // For Position

              target_position_.x = leftStickUD*0.2;
              target_position_.y = leftStickLR*0.2;
              target_position_.z = rightStickUD*0.3;

              kindr::Position3D target_in_base, target_in_odom;
              kindr_ros::convertFromRosGeometryMsg(target_position_,target_in_base);
              target_in_odom = base_to_odom_.getPosition() + base_to_odom_.getRotation().rotate(target_in_base);
              kindr_ros::convertToRosGeometryMsg(target_in_odom, target_position);
              ROS_INFO("base Target Position in Odom is : ");
              std::cout<<target_in_odom<<std::endl;
              base_target_msg.target.pose.position = target_position;
            }else if (LB ==1) {
              kindr::Position3D target_in_base, target_in_odom;
              kindr_ros::convertFromRosGeometryMsg(target_position_,target_in_base);
              target_in_odom = base_to_odom_.getPosition() + base_to_odom_.getRotation().rotate(target_in_base);
              kindr_ros::convertToRosGeometryMsg(target_in_odom, target_position);

              target_euler_.x = leftStickLR*0.5;
              target_euler_.y = leftStickUD*0.5;
              target_euler_.z = rightStickLR*0.5;
              kindr::RotationQuaternionD q_base, q_target;
              q_base = kindr::EulerAnglesXyzD(target_euler_.x, target_euler_.y, target_euler_.z);
              q_target = q_base*base_to_odom_.getRotation();
    //          tf::Quaternion q;
    //          q.setRPY(target_euler_.x, target_euler_.y, target_euler_.z);
              target_q.w = q_target.w();
              target_q.x = q_target.x();
              target_q.y = q_target.y();
              target_q.z = q_target.z();

              ROS_INFO("base Target Orinetation in Odom is : ");
              std::cout<<kindr::EulerAnglesXyzD(q_target)<<std::endl;
              base_target_msg.target.pose.orientation = target_q;
              base_target_msg.target.pose.position = target_position;
          }
          base_target_msg.average_linear_velocity = 0.1;
          base_target_msg.average_angular_velocity = 0.2;
          base_target_msg.ignore_timing_of_leg_motion = true;


          if(LB == 0 && RB == 0)
            {
              base_auto_msg.height = 0.5;//base_to_odom_.getPosition().z() - footprint_to_odom_.getPosition().z();
              base_auto_msg.ignore_timing_of_leg_motion = true;
//              base_auto_msg.average_linear_velocity = 0.5;
//              base_auto_msg.average_angular_velocity = 0.5;
              base_auto_msg.support_margin = 0.06;
              footstep_msg.target.header.frame_id = "odom";
              footstep_msg.ignore_contact = true;
              footstep_msg.ignore_for_pose_adaptation = true;
              footstep_msg.profile_type = "straight";

              end_effector_msg.ignore_contact = true;
              end_effector_msg.ignore_for_pose_adaptation = false;
              end_effector_msg.average_velocity = 0.8;
              geometry_msgs::PointStamped target_point;
              target_point.header.frame_id = "odom";
//              footstep_msg.average_velocity =
              geometry_msgs::Point lh_pos, rh_pos, lf_pos, rf_pos;
              if(keyA == 1)
                {

                  lh_pos_.x = leftStickUD*0.01 + lh_pos_.x;
                  lh_pos_.y = leftStickLR*0.01 + lh_pos_.y;
                  lh_pos_.z = rightStickUD*0.01 + lh_pos_.z;
                  kindr::Position3D target_in_footprint, target_in_odom;
                  kindr_ros::convertFromRosGeometryMsg(lh_pos_,target_in_footprint);
                  target_in_odom = footprint_to_odom_.getPosition() + footprint_to_odom_.getRotation().rotate(target_in_footprint);
                  kindr_ros::convertToRosGeometryMsg(target_in_odom, lh_pos);
                  ROS_INFO("LH_LEG Target Position in Odom is : ");
                  std::cout<<target_in_odom<<std::endl;
                  footstep_msg.target.point = lh_pos;
                  footstep_msg.name= "LH_LEG";

                  target_point.point = lh_pos;
                  end_effector_msg.target_position.push_back(target_point);
                  end_effector_msg.name = "LH_LEG";


                  leg_mode_msg.support_leg = false;
                  leg_mode_msg.name = "LH_LEG";
                  steps.base_auto.push_back(base_auto_msg);
                  steps.leg_mode.push_back(leg_mode_msg);
                  step_goal.steps.push_back(steps);
                  stepActionClient_->sendGoal(step_goal);
                  stepActionClient_->waitForResult();

                }
              if(keyB == 1)
                {

                  rh_pos_.x = leftStickUD*0.01 + rh_pos_.x;
                  rh_pos_.y = leftStickLR*0.01 + rh_pos_.y;
                  rh_pos_.z = rightStickUD*0.01 + rh_pos_.z;
                  kindr::Position3D target_in_footprint, target_in_odom;
                  kindr_ros::convertFromRosGeometryMsg(rh_pos_,target_in_footprint);
                  target_in_odom = footprint_to_odom_.getPosition() + footprint_to_odom_.getRotation().rotate(target_in_footprint);
                  kindr_ros::convertToRosGeometryMsg(target_in_odom, rh_pos);
                  ROS_INFO("RH_LEG Target Position in Odom is : ");
                  std::cout<<target_in_odom<<std::endl;
                  footstep_msg.target.point = rh_pos;
                  footstep_msg.name= "RH_LEG";

                  target_point.point = rh_pos;
                  end_effector_msg.target_position.push_back(target_point);
                  end_effector_msg.name = "RH_LEG";
                }
              if(keyX == 1)
                {

                  lf_pos_.x = leftStickUD*0.01 + lf_pos_.x;
                  lf_pos_.y = leftStickLR*0.01 + lf_pos_.y;
                  lf_pos_.z = rightStickUD*0.01 + lf_pos_.z;
                  kindr::Position3D target_in_footprint, target_in_odom;
                  kindr_ros::convertFromRosGeometryMsg(lf_pos_,target_in_footprint);
                  target_in_odom = footprint_to_odom_.getPosition() + footprint_to_odom_.getRotation().rotate(target_in_footprint);
                  kindr_ros::convertToRosGeometryMsg(target_in_odom, lf_pos);
                  ROS_INFO("LF_LEG Target Position in Odom is : ");
                  std::cout<<target_in_odom<<std::endl;
                  footstep_msg.target.point = lf_pos;
                  footstep_msg.name= "LF_LEG";

                  target_point.point = lf_pos;
                  end_effector_msg.target_position.push_back(target_point);
                  end_effector_msg.name = "LF_LEG";
                }
              if(keyY == 1)
                {

                  rf_pos_.x = leftStickUD*0.01 + rf_pos_.x;
                  rf_pos_.y = leftStickLR*0.01 + rf_pos_.y;
                  rf_pos_.z = rightStickUD*0.01 + rf_pos_.z;
                  kindr::Position3D target_in_footprint, target_in_odom;
                  kindr_ros::convertFromRosGeometryMsg(rf_pos_,target_in_footprint);
                  target_in_odom = footprint_to_odom_.getPosition() + footprint_to_odom_.getRotation().rotate(target_in_footprint);
                  kindr_ros::convertToRosGeometryMsg(target_in_odom, rf_pos);
                  ROS_INFO("RF_LEG Target Position in Odom is : ");
                  std::cout<<target_in_odom<<std::endl;
                  footstep_msg.target.point = rf_pos;
                  footstep_msg.name= "RF_LEG";

                  target_point.point = rf_pos;
                  end_effector_msg.target_position.push_back(target_point);
                  end_effector_msg.name = "RF_LEG";
                }
              if(keyA == 1 || keyB ==1 || keyX == 1 || keyY == 1)
                {
//                  count_ ++;
////                  steps.footstep.push_back(footstep_msg);
//                  ROS_INFO("Send end effector target");
//                  steps.end_effector_target.push_back(end_effector_msg);
//                  if(count_ >= 30)
//                    {
//                      steps.base_auto.push_back(base_auto_msg);
//                      count_ = 0;
//                    }
//                  step_goal.steps.push_back(steps);

//                  stepActionClient_->sendGoal(step_goal);
//                  stepActionClient_->waitForResult();
                }else if(!end_effector_msg.target_position.empty()){
//                  end_effector_msg.ignore_contact = false;
//                  end_effector_msg.ignore_for_pose_adaptation = false;
//                  steps.end_effector_target.push_back(end_effector_msg);
//                  steps.base_auto.push_back(base_auto_msg);
//                  step_goal.steps.push_back(steps);
//                  stepActionClient_->sendGoal(step_goal);
//                  stepActionClient_->waitForResult();
                }
            }else {
              ROS_INFO("Send base target");
            steps.base_target.push_back(base_target_msg);
            step_goal.steps.push_back(steps);
            stepActionClient_->sendGoal(step_goal);
            stepActionClient_->waitForResult();
          }
        }
      rate.sleep();
    }
}

void QuadrupedTeleOp::publishCommand()
{
    vel_pub_.publish(twist);
}

void QuadrupedTeleOp::publishDirectJointCommand()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_twist_node");
    ros::NodeHandle nh("~");
    QuadrupedTeleOp teleop_turtle(nh);
    ros::Rate rate(50);
    while (ros::ok()) {
//        teleop_turtle.publishCommand();
        rate.sleep();
        ros::spinOnce();
    }

}
