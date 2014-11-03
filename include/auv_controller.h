#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <pid_controller.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <tf/transform_listener.h>

using namespace geometry_msgs;

class AUVController
{
public:
    AUVController();

    void loadConfig();
    void createROSSubscribers();
    void createROSPublishers();
    void createROSTimers();

    void readPath(std::string path);

    void setGoal(Point goal);
    void setPose(Point position, double yaw);
    bool updatePose();

    bool goalReached();

    bool publish(Vector3 linear_velocity, double angular_velocity);

    void callback(const nav_msgs::Odometry::ConstPtr &msg);

    void update(const ros::TimerEvent& event);


private:

    void computeLinearError();
    void computeAngularError();

    double limitVelocity(double input, double max);

    double linear_error_;
    double angular_error_;

    PIDController angular_controller_;
    PIDController linear_controller_;

    Point position_;
    double yaw_;

    Point goal_;

    std::vector<Point> desired_path_;
    int path_index_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    tf::TransformListener tf_listener_;

    std::string odometry_topic_;
    std::string control_topic_;

    static const double MAX_LINEAR_VEL = 0.3;
    static const double MAX_ANGULAR_VEL = 0.17;

    static const double ACEPTABLE_ANGULAR_ERROR = 0.09;
    static const double ACEPTABLE_DISTANCE_ERROR = 0.15;



};

#endif // CONTROLLER_H
