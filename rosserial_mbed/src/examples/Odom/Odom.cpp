/*
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <ros/ros_time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";

int main(void) {
    nh.initNode();
    broadcaster.init(nh);


    while (1) {
        // drive in a circle
        double dx = 0.2;
        double dtheta = 0.18;
        x += cos(theta)*dx*0.1;
        y += sin(theta)*dx*0.1;
        theta += dtheta*0.1;
        if (theta > 3.14)
            theta=-3.14;

        // tf odom->base_link
        t.header.frame_id = odom;
        t.child_frame_id = base_link;

        t.transform.translation.x = x;
        t.transform.translation.y = y;

        t.transform.rotation = tf::createQuaternionFromYaw(theta);
        t.header.stamp = nh.now();

        broadcaster.sendTransform(t);
        nh.spinOnce();

        wait_ms(10);
    }
}

