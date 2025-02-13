/*
 * rosserial IR Ranger Example
 *
 * This example is calibrated for the Sharp GP2D120XJ00F.
 */

#include <ros.h>
#include <ros/ros_time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;


sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

#if defined(TARGET_LPC1768)
PinName analog_pin = p20;
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
PinName analog_pin = A0;
#else
#error "You need to specify a pin for the mic"
#endif
unsigned long range_timer;

/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.
 */
float getRange(PinName pin_num) {
    int sample;
    // Get data
    sample = AnalogIn(pin_num).read_u16()/4;
    // if the ADC reading is too low,
    //   then we are really far away from anything
    if(sample < 10)
        return 254;     // max range
    // Magic numbers to get cm
    sample= 1309/(sample-3);
    return (sample - 1)/100; //convert to meters
}

char frameid[] = "/ir_ranger";

Timer t;
int main() {
    t.start();
    nh.initNode();
    nh.advertise(pub_range);

    range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg.header.frame_id =  frameid;
    range_msg.field_of_view = 0.01;
    range_msg.min_range = 0.03;
    range_msg.max_range = 0.4;

    while (1) {
        // publish the range value every 50 milliseconds
        //   since it takes that long for the sensor to stabilize
        if ( (t.read_ms()-range_timer) > 50) {
            range_msg.range = getRange(analog_pin);
            range_msg.header.stamp = nh.now();
            pub_range.publish(&range_msg);
            range_timer =  t.read_ms() + 50;
        }
        nh.spinOnce();
    }
}

