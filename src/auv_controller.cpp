
#include <boost/random.hpp>

#include <controller.h>

//void addSeaCurrent(geometry_msgs::Vector3 &linear,geometry_msgs::Vector3 &angular){

//    //Now add some gaussian noise
//    static boost::mt19937 rng_; ///< Boost random number generator
//    static boost::mt19937 rng2_; ///< Boost random number generator

//    static boost::normal_distribution<> normal(0,MAX_LINEAR_VEL/3.5);
//    static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng_, normal);

//    static boost::normal_distribution<> normal2(0,MAX_ANGULAR_VEL/10);
//    static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor2(rng2_, normal2);


//    //! calcula ruido para simular corrente
//    linear.x += var_nor();
//    linear.y += var_nor();
//    linear.z += var_nor();

//    angular.z += var_nor2();

//}

int main(int argc, char **argv){

    ros::init(argc, argv, "auv_controller");

    Controller controller;

    ros::spin();


    return 0;

}
