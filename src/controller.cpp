#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <blimp_controller/ControllerConfig.h>
#include <string>
#include <vector>
#include <algorithm>
#include <std_msgs/Float32.h>

class PID {
    private:
        float kp, ki, kd;
        float current_u;
        float prev_u;
        float current_e;
        float prev_e_1;
        float prev_e_2;
        ros::Time current;
        ros::Time last;
        double dt;
        
    public:
        PID () {
            kp = 0.0;
            ki = 0.0;
            kd = 0.0;
            prev_u = 0;
            prev_e_1 = 0;
            prev_e_2 = 0;
            last = ros::Time::now();
        }
        PID (float kp_, float ki_, float kd_) {
            kp = kp_;
            ki = ki_;
            kd = kd_;
            prev_u = 0;
            prev_e_1 = 0;
            prev_e_2 = 0;
            last = ros::Time::now();
        }
        
        float update (float e_) {
            current = ros::Time::now();
            current_e = e_;
            dt = (current - last).toSec();
            current_u = prev_u + kp*(current_e-prev_e_1) + ki*dt*current_e + kd/dt*(current_e-2*prev_e_1+prev_e_2);
            
            // TODO saturation

            // Store data as previous
            prev_u = current_u;
            prev_e_2 = prev_e_1;
            prev_e_1 = current_e;
            last = current;
            return current_u;
        }
        
        void tune_pid (float kp_, float ki_, float kd_) {
            kp = kp_;
            ki = ki_;
            kd = kd_;
        }
        
        void reset () {
            prev_u = 0;
            prev_e_1 = 0;
            prev_e_2 = 0;
            last = ros::Time::now();
        }
};

class BlimpController {
    private:
        ros::NodeHandle nh_;
        dynamic_reconfigure::Server<blimp_controller::ControllerConfig> server;
        dynamic_reconfigure::Server<blimp_controller::ControllerConfig>::CallbackType f;
        tf::TransformListener listener;
        PID pid_thrust;
        float command_thrust;
        ros::Publisher command_pub;
        bool isManual;
        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Vector3 goal;
        tf::Quaternion q;
    public:
        BlimpController(ros::NodeHandle &node_)
        {
            nh_ = node_;
            f = boost::bind(&BlimpController::callback, this, _1, _2);
            server.setCallback(f);
            pid_thrust.tune_pid(0.12, 0.0, 0.15);
            command_pub = nh_.advertise<std_msgs::Float32>("thrust",1);
            isManual = true;
            goal = tf::Vector3(0.0,0.0,0.0);
            q.setRPY(0.0, 0.0, 0.0);
            transform.setOrigin(goal);
            transform.setRotation(q);
        }
        void callback(blimp_controller::ControllerConfig &config, uint32_t level){
            goal = tf::Vector3(config.groups.goal.x, config.groups.goal.y, config.groups.goal.z);
            q.setRPY(0.0, 0.0, config.groups.goal.yaw);
            isManual = config.manual;
            transform.setOrigin(goal);
            transform.setRotation(q);
            pid_thrust.tune_pid((float)config.groups.pid.p, (float)config.groups.pid.i, (float)config.groups.pid.d);
        }
        
        void publish(){
            if (isManual){
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
            }
        }
        
        void navigate(){
            std::vector<std::string> frames;
            tf::StampedTransform transform;
            ros::Time last_time;
            std::string *error;
            try{
                listener.getFrameStrings(frames);
                if (std::find(frames.begin(), frames.end(), "blimp") != frames.end()){
                    listener.getLatestCommonTime("world","blimp",last_time,error);
                    if (ros::Time::now()-last_time > ros::Duration(1))
                    {
                        pid_thrust.reset();
                        std_msgs::Float32 msg;
                        msg.data = 0;
                        command_pub.publish(msg);
                        return;         // Too old
                    }
                    listener.lookupTransform("blimp", "goal", ros::Time(0), transform);
                    
                    // TODO align orientation to the shortest part.
                    // transform.getOrientation() --> Quaternion
                    
                    float e_x = transform.getOrigin().x();
                    float e_y = transform.getOrigin().y();
                    float e_z = transform.getOrigin().z();
                    
                    command_thrust = pid_thrust.update(e_x);
                    std_msgs::Float32 msg;
                    msg.data = command_thrust;
                    command_pub.publish(msg);
                    
                }
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }
        }
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "blimp_controller");//, ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    ros::Rate rate(30);
    BlimpController blimp_controller(nh);
    while (nh.ok()){
        blimp_controller.publish();
        blimp_controller.navigate();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
