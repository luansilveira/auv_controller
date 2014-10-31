#include "controller.h"

Controller::Controller():
    angular_controller_(0.25,0,0), linear_controller_(1,0,0)
{
    loadConfig();

    createROSSubscribers();

    createROSPublishers();

}

void Controller::loadConfig()
{
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("control_topic",control_topic_,"/dataNavigator");

    private_nh.param<std::string>("odometry_topic",odometry_topic_,"/uwsim/girona500_odom");

    std::string path;
    private_nh.param<std::string>("path",path,"~/Codigos/indigo_ws/auv_controller/path.txt");

    readPath(path);
}

void Controller::readPath(std::string path)
{
    std::ifstream file(path.c_str());
    Point pt;

    while(!file.eof()){
        file >> pt.x >> pt.y >> pt.z;
        desired_path_.push_back(pt);
    }
    desired_path_.pop_back();
}

void Controller::createROSPublishers(){
    pub_ = nh_.advertise<nav_msgs::Odometry>(control_topic_, 1000);
}

void Controller::createROSSubscribers()
{
    sub_ = nh_.subscribe<nav_msgs::Odometry>(odometry_topic_, 1000, &Controller::callback,this);
}

void Controller::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    Vector3 linear_velocity;
    double angular_velocity;

    setPose(msg->pose.pose.position,tf::getYaw(msg->pose.pose.orientation));

    //! Test if goal is reached
    if(goalReached())
    {
        //! increase path index
        path_index_++;

        if(path_index_ == desired_path_.size())
        {
            //! Finish execution
            ros::requestShutdown();
            return;
        }
        else
        {
            //! Queue next desired goal
            setGoal(desired_path_[path_index_]);
        }
    }



    //! Compute AUV angular velocity
    angular_velocity = angular_controller_.get(angular_error_);
    angular_velocity = limitVelocity(angular_velocity,MAX_ANGULAR_VEL);

    //! Compute AUV linear velocity
    //! If the robot is oriented with an aceptable angular error, start to goal travelling
    if(fabs(angular_error_) > ACEPTABLE_ANGULAR_ERROR)
    {
        double linear_control = linear_controller_.get(linear_error_);
        linear_control = limitVelocity(linear_control,MAX_LINEAR_VEL);

        linear_velocity.x = linear_control * (   (goal_.x - position_.x)/ linear_error_  );
        linear_velocity.y = linear_control * (   (goal_.y - position_.y)/ linear_error_  );
        linear_velocity.z = linear_control * (   (goal_.z - position_.z)/ linear_error_  );
    }
    else
    {
        linear_velocity.x  = 0;
        linear_velocity.y  = 0;
        linear_velocity.z  = 0;
    }

    publish(linear_velocity,angular_velocity);

}


double Controller::limitVelocity(double input, double max)
{
    if (input < -max)
        return -max;
    else if (input > max)
        return max;
    else
        return input;
}

void Controller::setGoal(Point goal)
{
    goal_ = goal;

    computeAngularError();
    computeLinearError();
}

void Controller::setPose(Point position, double yaw)
{
    position_ = position;
    yaw_ = yaw;

    computeAngularError();
    computeLinearError();
}

bool Controller::goalReached()
{
    return linear_error_ < ACEPTABLE_DISTANCE_ERROR;
}

bool Controller::publish(Vector3 linear_velocity, double angular_velocity)
{
    nav_msgs::Odometry control_msg;
    control_msg.pose.pose.position.x=0.0;
    control_msg.pose.pose.position.y=0.0;
    control_msg.pose.pose.position.z=0.0;
    control_msg.pose.pose.orientation.x=0.0;
    control_msg.pose.pose.orientation.y=0.0;
    control_msg.pose.pose.orientation.z=0.0;
    control_msg.pose.pose.orientation.w=1;

    control_msg.twist.twist.linear = linear_velocity;
    control_msg.twist.twist.angular.x = 0;
    control_msg.twist.twist.angular.y = 0;
    control_msg.twist.twist.angular.z = angular_velocity;
    for (int i=0; i<36; i++) {
        control_msg.twist.covariance[i]=0;
        control_msg.pose.covariance[i]=0;
    }
    pub_.publish(control_msg);

}

void Controller::computeAngularError()
{
    double angle_robot_goal;

    angle_robot_goal = atan2(goal_.y - position_.y,goal_.x - position_.x);

    angular_error_ =  angles::shortest_angular_distance(yaw_,angle_robot_goal);
}


void Controller::computeLinearError()
{
    linear_error_ = sqrt(pow(goal_.x - position_.x,2)+
                         pow(goal_.y - position_.y,2)+
                         pow(goal_.z - position_.z,2));
}
