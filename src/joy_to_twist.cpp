#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
    TeleopTurtle(ros::NodeHandle& nodehandle);
    void publishCommand();
    void publishDirectJointCommand();
    int directJointFlag_;
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int resetFlag_,legMoveFlag_;
  double linearX_, linearY_, angular_;
  double l_scale_, a_scale_;
    geometry_msgs::Twist twist, legTwist;
  ros::Publisher vel_pub_, legMovePub_, cartesianDiffPub_,jointCommandPub_, homingCommandPub_, emergencyStopPub_, resetCommandPub_;
  ros::Subscriber joy_sub_;


};


TeleopTurtle::TeleopTurtle(ros::NodeHandle& nodehandle):
  directJointFlag_(0),
  nh_(nodehandle),
  resetFlag_(0),
  legMoveFlag_(0)
{

    nh_.param("scale_angular", a_scale_, 1.0);
    nh_.param("scale_linear", l_scale_, 1.0);
    ROS_INFO("augular scale is %f, linear scale is %f",a_scale_,l_scale_);

    legMovePub_ = nh_.advertise<geometry_msgs::Twist>("/leg_cmd_vel",1);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
//    visual_servo_test::CartesianPose diffPoseMsg;
//    twist.angular.z = a_scale_*joy->axes[angular_];
//    twist.linear.x = l_scale_*joy->axes[linear_];
//    vel_pub_.publish(twist);

    float leftStickLR = joy->axes[0];//contiunus 1~-1
    float leftStickUD = joy->axes[1];//contiunus 1~-1
    float LT = 1 - joy->axes[2];//contiunus 0~2
    float rightStickLR = joy->axes[3];//contiunus 1~-1
    float rightStickUD = joy->axes[4];//contiunus 1~-1
    float RT = 1 - joy->axes[5];//contiunus 0~2
    float crossKeyLR = joy->axes[6];//1/-1
    float crossKeyUD = joy->axes[7];//1/-1
    int keyA = joy->buttons[0];//0/1
    int keyB = joy->buttons[1];//0/1
    int keyX = joy->buttons[2];//0/1
    int keyY = joy->buttons[3];//0/1
    int LB = joy->buttons[4];//0/1
    int RB = joy->buttons[5];//0/1
    int keyBack = joy->buttons[6];//0/1
    int keyStart = joy->buttons[7];//0/1
    int keyPower = joy->buttons[8];//0/1

    float velocityScale = l_scale_*RT +1;
    float angularScale = a_scale_*LT +1;
    if(keyStart == 1)
    {
        ROS_INFO("Reset");

        resetFlag_ = 1;
        legMoveFlag_ = 1;
    }
    if(keyPower == 1)
    {
        ROS_INFO("Emergency Stop");
        linearX_ = 0;
        linearY_ = 0;
        angular_ = 0;
        legTwist.linear.x = 0;
        legTwist.linear.y = 0;
        legTwist.angular.z =0;
        legMovePub_.publish(legTwist);
        resetFlag_ = 0;

        return;
    }

//    linearX_ = 0;
//    linearY_ = 0;
//    angular_ = 0;

    if(resetFlag_)
    {
        linearX_ = velocityScale*leftStickUD;
        linearY_ = velocityScale*rightStickLR;
        angular_ = angularScale*leftStickLR;
            twist.linear.x = linearX_;
    twist.linear.y = linearY_;
    twist.angular.z = angular_;
    vel_pub_.publish(twist);
        ROS_INFO("update twist command linear x = %f y = %f, angular is %f",linearX_, linearY_,angular_);
    }
    if(legMoveFlag_==1&&crossKeyUD>0)//moveforward
    {
        legTwist.linear.x = crossKeyUD;
        legTwist.linear.y = 0;
        legTwist.angular.z =0;
        legMovePub_.publish(legTwist);
    }
    if(legMoveFlag_==1&&crossKeyUD<0)//movebackward
    {
        legTwist.linear.x = crossKeyUD;
        legTwist.linear.y = 0;
        legTwist.angular.z =0;
        legMovePub_.publish(legTwist);
    }
    if(legMoveFlag_==1&&crossKeyLR<0)//movebackward
    {
        legTwist.linear.x = 0;
        legTwist.linear.y = 0;
        legTwist.angular.z =crossKeyLR;
        legMovePub_.publish(legTwist);
    }
    if(legMoveFlag_==1&&crossKeyLR>0)//movebackward
    {
        legTwist.linear.x = 0;
        legTwist.linear.y = 0;
        legTwist.angular.z =crossKeyLR;
        legMovePub_.publish(legTwist);
    }

    if(legMoveFlag_==1&&keyY==1)//stand
    {
        legTwist.linear.x = 0;
        legTwist.linear.y = 0;
        legTwist.angular.z =0;
        legMovePub_.publish(legTwist);
    }
    if(legMoveFlag_==1&&keyA==1)//stand to wheel
    {
        legTwist.linear.x = 0;
        legTwist.linear.y = keyA;
        legTwist.angular.z =0;
        legMovePub_.publish(legTwist);
    }
    // twist.linear.x = linearX_;
    // twist.linear.y = linearY_;
    // twist.angular.z = angular_;
    // vel_pub_.publish(twist);

}

void TeleopTurtle::publishCommand()
{
    vel_pub_.publish(twist);
}

void TeleopTurtle::publishDirectJointCommand()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_twist_node");
    ros::NodeHandle nh("~");
    TeleopTurtle teleop_turtle(nh);
    ros::Rate rate(50);
    while (ros::ok()) {
//        teleop_turtle.publishCommand();
        rate.sleep();
        ros::spinOnce();
    }

}
