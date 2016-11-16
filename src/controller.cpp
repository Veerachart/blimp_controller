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

const double PI = 3.141592653589793;

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
        float command_yaw;
        ros::Publisher command_pub;
        ros::Publisher yaw_pub;
        bool isManual;
        bool isWaitingAngle;
        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Vector3 goal;
        tf::Quaternion q;
        float prev_dist;
    public:
        BlimpController(ros::NodeHandle &node_)
        {
            nh_ = node_;
            f = boost::bind(&BlimpController::callback, this, _1, _2);
            server.setCallback(f);
            pid_thrust.tune_pid(0.12, 0.0, 0.15);
            command_pub = nh_.advertise<std_msgs::Float32>("thrust",1);
            yaw_pub = nh_.advertise<std_msgs::Float32>("yaw_cmd",1);
            isManual = true;
            isWaitingAngle = false;
            goal = tf::Vector3(0.0,0.0,0.0);
            q.setRPY(0.0, 0.0, 0.0);
            transform.setOrigin(goal);
            transform.setRotation(q);
            prev_dist = 1e6;
        }
        void callback(blimp_controller::ControllerConfig &config, uint32_t level){
            goal = tf::Vector3(config.groups.goal.x, config.groups.goal.y, config.groups.goal.z);
            q.setRPY(0.0, 0.0, config.groups.goal.yaw*PI/180);
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
                        prev_dist = 1e6;
                        std_msgs::Float32 msg;
                        msg.data = 0;
                        command_pub.publish(msg);
                        return;         // Too old
                    }
                    listener.lookupTransform("blimp", "goal", ros::Time(0), transform);
                    tf::StampedTransform t_blimp;
                    listener.lookupTransform("world", "blimp", last_time, t_blimp);
                    
                    // TODO align orientation to the shortest part.
                    // transform.getOrientation() --> Quaternion
                    tf::Vector3 d = transform.getOrigin();
                    tf::Vector3 d_blimp = t_blimp.getOrigin();
                    tf::Matrix3x3 m(t_blimp.getRotation());
                    double r_blimp, p_blimp, y_blimp;
                    m.getRPY(r_blimp, p_blimp, y_blimp);
                    if (isWaitingAngle) {           // Waiting for the turn
                        if (fabs(y_blimp - command_yaw) < 0.01) {      // Angle is already good
                            isWaitingAngle = false;
                            pid_thrust.reset();
                            prev_dist = 1e6;
                            std_msgs::Float32 msg;
                            msg.data = 0;
                            command_pub.publish(msg);
                        }
                    }
                    else if (pow(d.x(),2) + pow(d.y(),2) > 1.0e-4) {     // Some displacement to travel
                        if (fabs(d.y()) > 0.02) {     // Align x axis to the displacement
                            double y_turn = atan2(d.y(), d.x());
                            std_msgs::Float32 msg;
                            if (fabs(y_turn) > PI/2){
                                command_yaw = y_blimp + y_turn - PI;        // Reversed
                            }else{
                                command_yaw = y_blimp + y_turn;
                            }
                            if (command_yaw > PI)
                                command_yaw -= 2*PI;
                            else if (command_yaw <= -PI)
                                command_yaw += 2*PI;
                            msg.data = command_yaw;
                            yaw_pub.publish(msg);
                            pid_thrust.reset();
                            prev_dist = 1e6;
                            msg.data = 0;
                            command_pub.publish(msg);
                            isWaitingAngle = true;
                        }
                        else {
                            float e_x = d.x();
                            //float e_y = d.y();
                            //float e_z = d.z();
                            
                            command_thrust = pid_thrust.update(e_x);
                            std_msgs::Float32 msg;
                            msg.data = command_thrust;
                            command_pub.publish(msg);
                            prev_dist = pow(d.x(),2) + pow(d.y(),2);
                        }
                    }
                    else if (fabs(sqrt(pow(d.x(),2) + pow(d.y(),2)) - sqrt(prev_dist)) < 1e-4){       // close to the goal at low speed; just align to the goal
                        tf::StampedTransform t_blimp;
                        listener.lookupTransform("world", "blimp", last_time, t_blimp);
                        tf::Matrix3x3 m(t_blimp.getRotation());
                        double r_blimp, p_blimp, y_blimp;
                        m.getRPY(r_blimp, p_blimp, y_blimp);
                        tf::Matrix3x3 m2(transform.getRotation());
                        double r_turn, p_turn, y_turn;
                        m2.getRPY(r_turn, p_turn, y_turn);
                        std_msgs::Float32 msg;
                        command_yaw = y_blimp + y_turn;
                        if (command_yaw > PI)
                            command_yaw -= 2*PI;
                        else if (command_yaw <= -PI)
                            command_yaw += 2*PI;
                        msg.data = command_yaw;
                        yaw_pub.publish(msg);
                        pid_thrust.reset();
                        prev_dist = pow(d.x(),2) + pow(d.y(),2);
                        msg.data = 0;
                        command_pub.publish(msg);
                        isWaitingAngle = true;
                    }
                    else {
                        pid_thrust.reset();
                        prev_dist = pow(d.x(),2) + pow(d.y(),2);
                        std_msgs::Float32 msg;
                        msg.data = 0;
                        command_pub.publish(msg);
                    }
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
