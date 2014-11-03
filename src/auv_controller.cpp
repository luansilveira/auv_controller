#include "auv_controller.h"

AUVController::AUVController():
    angular_controller_(0.25,0,0), linear_controller_(1,0,0)
{
    loadConfig();

    //createROSSubscribers();

    createROSPublishers();

    createROSTimers();

}

void AUVController::loadConfig()
{
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("control_topic",control_topic_,"/g500/twist");

    private_nh.param<std::string>("odometry_topic",odometry_topic_,"/uwsim/girona500_odom");

    std::string path;
    private_nh.param<std::string>("path",path,"/home/lsilveira/Codigos/indigo_ws/src/auv_controller/path_trainning.txt");

    readPath(path);
}

void AUVController::readPath(std::string path)
{
    std::ifstream file(path.c_str());
    Point pt;

    if (!file.is_open())
    {
        std::cerr << "The file " << path << " did not open succesfully." << std::endl;
        exit(-1);
    }

    while(!file.eof()){
        file >> pt.x >> pt.y >> pt.z;
        std::cout << pt << std::endl;
        desired_path_.push_back(pt);
    }
    desired_path_.pop_back();

}

void AUVController::createROSPublishers(){
    pub_ = nh_.advertise<geometry_msgs::TwistStamped>(control_topic_, 1000);
}

void AUVController::createROSTimers()
{
    timer_ = nh_.createTimer(ros::Duration(0.1),&AUVController::update,this);
}

void AUVController::createROSSubscribers()
{
    sub_ = nh_.subscribe<nav_msgs::Odometry>(odometry_topic_, 1000, &AUVController::callback,this);
}

bool AUVController::updatePose()
{
    geometry_msgs::Pose pose;
    tf::StampedTransform transform;

    try{
        tf_listener_.lookupTransform("/girona500/base_link", "/world",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return false;
    }

    tf::poseTFToMsg(transform,pose);

    setPose(pose.position,tf::getYaw(pose.orientation));

    return true;


}

void AUVController::update(const ros::TimerEvent& event)
{
    Vector3 linear_velocity;
    double angular_velocity;

    if (!updatePose())
        return;

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

void AUVController::callback(const nav_msgs::Odometry::ConstPtr& msg)
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


double AUVController::limitVelocity(double input, double max)
{
    if (input < -max)
        return -max;
    else if (input > max)
        return max;
    else
        return input;
}

void AUVController::setGoal(Point goal)
{
    goal_ = goal;

    computeAngularError();
    computeLinearError();
}

void AUVController::setPose(Point position, double yaw)
{
    position_ = position;
    yaw_ = yaw;

    computeAngularError();
    computeLinearError();
}

bool AUVController::goalReached()
{
    return linear_error_ < ACEPTABLE_DISTANCE_ERROR;
}

bool AUVController::publish(Vector3 linear_velocity, double angular_velocity)
{

    TwistStamped control_msg;
    control_msg.header.stamp = ros::Time::now();
    control_msg.header.frame_id = "girona_500/base_link";
    control_msg.twist.linear = linear_velocity;
    control_msg.twist.angular.x = 0;
    control_msg.twist.angular.y = 0;
    control_msg.twist.angular.z = angular_velocity;

    pub_.publish(control_msg);
}

void AUVController::computeAngularError()
{
    double angle_robot_goal;

    angle_robot_goal = atan2(goal_.y - position_.y,goal_.x - position_.x);

    angular_error_ =  angles::shortest_angular_distance(yaw_,angle_robot_goal);
}


void AUVController::computeLinearError()
{
    linear_error_ = sqrt(pow(goal_.x - position_.x,2)+
                         pow(goal_.y - position_.y,2)+
                         pow(goal_.z - position_.z,2));
}
