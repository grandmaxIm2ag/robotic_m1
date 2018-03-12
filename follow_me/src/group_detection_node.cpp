// Signal handling
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

class group_detection {
private:
    ros::NodeHandle n;
    
    //Receive robot's position
    ros::Subscriber sub_robot;
    //Receive the set of detected persons
    ros::Subscriber sub_detect_person;
    //Publish goal_to_reach
    ros::Publisher pub_moving_persons_detector;
    //Publish display
    ros::Publisher pub_group_detector_marker;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];
    
    //to store the goal to reach that we will be published
    geometry_msgs::Point goal_to_reach;

    //to perform detection of moving legs and to store them
    int nb_group_detected;
    // to store the middle of each group
    geometry_msgs::Point group_detected[1000];
    
public:

    void perso_callback(const std_msgs::Float32MultiArray::
                        ConstPtr& array){}
    void position_callback(const geometry_msgs::Point::ConstPtr& g);
    void update(){}
    void detect_group(){}
};

int main(int argc, char **argv){

    return 0;
}
