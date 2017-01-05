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
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <fstream>

const double PI = 3.141592653589793;
const double ALPHA_MAX = 3.0;
const double OMEGA_MAX = 6.0;
const double A_MAX = 0.1;
const double V_MAX = 0.2;

class PID {
    private:
        float kp, ki, kd;
        float u;
        float current_e, prev_e;
        float P, I, D;
        ros::Time current;
        ros::Time last;
        double dt, Tf;
        
    public:
        PID () {
            kp = 0.0;
            ki = 0.0;
            kd = 0.0;
            prev_e = 0;
            P = 0;
            I = 0;
            D = 0;
            //Tf = kd/5.0;
            last = ros::Time::now();
        }
        PID (float kp_, float ki_, float kd_) {
            kp = kp_;
            ki = ki_;
            kd = kd_;
            //Tf = kd/5.0;
            prev_e = 0;
            P = 0;
            I = 0;
            D = 0;
            last = ros::Time::now();
        }
        
        float update (float e_) {
            current = ros::Time::now();
            current_e = e_;
            dt = (current - last).toSec();
            P = kp*current_e;
            I = I+ki*dt*current_e;
            //D = (Tf*D + kd*Tf*(current_e-prev_e))/(dt+Tf);
            D = kd*(current_e-prev_e)/dt;
            u = P+I+D;
            
            // TODO saturation

            // Store data as previous
            prev_e = current_e;
            last = current;
            return u;
        }
        
        void tune_pid (float kp_, float ki_, float kd_) {
            kp = kp_;
            ki = ki_;
            kd = kd_;
            //Tf = kd/5.0;
        }
        
        void reset () {
            prev_e = 0;
            P = 0;
            I = 0;
            D = 0;
            last = ros::Time::now();
        }
};

class BlimpController {
    private:
        ros::NodeHandle nh_;
        dynamic_reconfigure::Server<blimp_controller::ControllerConfig> server;
        dynamic_reconfigure::Server<blimp_controller::ControllerConfig>::CallbackType f;
        tf::TransformListener listener;
        tf::TransformBroadcaster br;
        PID pid_thrust;
        PID pid_z;
        bool isManual;
        float command_yaw;
        float command_thrust;
        ros::Publisher command_pub;
        geometry_msgs::Twist msg;
        
        // Start --turn--> move_origin --move--> turn_origin --turn--> final_goal
        tf::StampedTransform start;
        tf::StampedTransform move_origin;
        tf::StampedTransform turn_origin;
        tf::StampedTransform final_goal;
        tf::StampedTransform current;
        
        float goal_x, goal_y, goal_z, goal_yaw;
        tf::Quaternion q;
        ros::Time goal_update;
        
        unsigned int state;             // 0 = at goal, 1 = align, 2 = move, 3 = turn
        
        float a_max, v_max;             // limit of movement for path generation
        float alpha_max, omega_max;     // limit of rotation
        float t_rise;                   // rising time of acceleration
        float t_const;                  // time of constant velocity
        
        float yaw_1, yaw_2;             // yaw_1 for rotate before move, yaw_2 for rotate after move
        float distance;                 // distance to move
        
        ros::Time t_start;
        float t_old;
        float a_old, v_old;
        float alpha_old, omega_old;
        float a, v, x;
        float alpha, omega;
        
        tf::Vector3 d_blimp;
        tf::Vector3 d;
        
    public:
        BlimpController(ros::NodeHandle &node_)
        {
            nh_ = node_;
            f = boost::bind(&BlimpController::callback, this, _1, _2);
            server.setCallback(f);
            command_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
            state = 0;
            t_rise = 2.0;
            pid_thrust.tune_pid(0.006, 0.0, 1.0);
            goal_x, goal_y, goal_z, goal_yaw = 0;
            final_goal.setOrigin(tf::Vector3(goal_x, goal_y, goal_z));
            q.setRPY(0.0, 0.0, goal_yaw*PI/180.0);
            final_goal.setRotation(q);
        }
        
        void callback(blimp_controller::ControllerConfig &config, uint32_t level){
            if ((goal_x != config.groups.goal.x || goal_y != config.groups.goal.y || goal_z != config.groups.goal.z || goal_yaw != config.groups.goal.yaw) && config.manual) {
                // the goal is changed
                goal_x = config.groups.goal.x;
                goal_y = config.groups.goal.y;
                goal_z = config.groups.goal.z;
                goal_yaw = config.groups.goal.yaw;
                q.setRPY(0.0, 0.0, config.groups.goal.yaw*PI/180);
                final_goal.setOrigin(tf::Vector3(goal_x, goal_y, goal_z));
                final_goal.setRotation(q);
                
                br.sendTransform(tf::StampedTransform(final_goal, ros::Time::now(), "world", "goal"));
                br.sendTransform(tf::StampedTransform(final_goal, ros::Time::now(), "world", "command"));
            }
            isManual = config.manual;
            pid_thrust.tune_pid((float)config.groups.pid_th.p_th, (float)config.groups.pid_th.i_th, (float)config.groups.pid_th.d_th);
            pid_z.tune_pid((float)config.groups.pid_z.p_z, (float)config.groups.pid_z.i_z, (float)config.groups.pid_z.d_z);
        }
        
        void navigate(){
            br.sendTransform(tf::StampedTransform(final_goal, ros::Time::now(), "world", "goal"));
            std::vector<std::string> frames;
            ros::Time last_time;
            std::string *error;
            float t;
            double r_blimp, p_blimp, y_blimp;
            
            try{
                listener.getFrameStrings(frames);
                if (std::find(frames.begin(), frames.end(), "blimp") != frames.end()){
                    listener.getLatestCommonTime("world","blimp",last_time,error);
                    if (ros::Time::now()-last_time > ros::Duration(1))
                    {
                        pid_thrust.reset();
                        pid_z.reset();
                        //msg.linear.x = 0;
                        //msg.linear.z = 0;
                        //msg.angular.z = 0;
                        //command_pub.publish(msg);
                        return;         // Too old
                    }
                    listener.lookupTransform("world", "blimp", last_time, current);
                    d_blimp = current.getOrigin();
                    tf::Matrix3x3 m(current.getRotation());
                    m.getRPY(r_blimp, p_blimp, y_blimp);
                }
                tf::StampedTransform diff;
                listener.lookupTransform("blimp", "goal", ros::Time(0), diff);
                float dist = sqrt(pow(diff.getOrigin().x(),2) + pow(diff.getOrigin().y(),2) + pow(diff.getOrigin().z(),2));
                tf::Matrix3x3 m(diff.getRotation());
                double roll,pitch,turn;
                m.getRPY(roll, pitch, turn);
                
                if (std::find(frames.begin(), frames.end(), "command") != frames.end()){
                    listener.getLatestCommonTime("world","command",last_time,error);
                    if ((last_time - goal_update).toSec() || (state == 0 && (dist > 0.1 || fabs(turn) > PI/18.0))){     // Time is not equal --> new update || distance is too far, angle error when it is at rest
                        if ((last_time - goal_update).toSec())
                            ROS_INFO("Goal updated!");
                        else
                            ROS_INFO("Readjusting");
                        goal_update = last_time;
                        if (state == 0){
                            // Create path and steps
                            t_start = ros::Time::now();
                            tf::StampedTransform trans;
                            listener.lookupTransform("blimp", "goal", ros::Time(0), trans);
                            d = trans.getOrigin();
                            command_yaw = y_blimp;
                            
                            yaw_1 = atan2(d.y(), d.x());
                            if (fabs(yaw_1) > 2*PI/180.0) {
                                if (fabs(yaw_1) > PI/2){
                                    if (yaw_1 > 0)
                                        yaw_1 = yaw_1 - PI;         // Reversed
                                    else
                                        yaw_1 = yaw_1 + PI;
                                }
                                
                                alpha_old = 0;
                                omega_old = 0;
                                a_old = 0;
                                v_old = 0;
                                if (yaw_1 > 2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                    alpha_max = ALPHA_MAX*PI/180.0;
                                    t_const = (yaw_1 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                }
                                else if (yaw_1 < -2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                    alpha_max = -ALPHA_MAX*PI/180.0;
                                    t_const = (yaw_1 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                }
                                else{
                                    alpha_max = yaw_1/(2*t_rise*t_rise);
                                    t_const = 0;
                                }
                                state = 1;
                            }
                            else if (pow(d.x(),2) + pow(d.y(),2) > 0.1) {
                                distance = sqrt(pow(d.x(),2) + pow(d.y(),2));
                                if (distance > 2*A_MAX*t_rise*t_rise) {
                                    a_max = A_MAX;
                                    t_const = (distance - 2*a_max*t_rise*t_rise)/(a_max*t_rise);
                                    state = 2;
                                }
                                else if (distance < -2*A_MAX*t_rise*t_rise) {
                                    a_max = -A_MAX;
                                    t_const = (distance - 2*a_max*t_rise*t_rise)/(a_max*t_rise);
                                    state = 2;
                                }
                                else{
                                    a_max = distance/(2*t_rise*t_rise);
                                    t_const = 0;
                                    state = 2;
                                }
                                pid_thrust.reset();
                                state = 2;
                                
                                move_origin = current;
                            
                                br.sendTransform(tf::StampedTransform(move_origin, ros::Time::now(), "world", "start"));

                            }
                            else {
                                yaw_2 = goal_yaw*PI/180.0 - y_blimp;
                                if (yaw_2 > PI)
                                    yaw_2 = yaw_2 - 2*PI;
                                else if (yaw_2 <= -PI)
                                    yaw_2 = yaw_2 + 2*PI;
                                ROS_INFO("%.2f", yaw_2*180/PI);
                                    
                                alpha_old = 0;
                                omega_old = 0;
                                a_old = 0;
                                v_old = 0;
                                if (yaw_2 > 2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                    alpha_max = ALPHA_MAX*PI/180.0;
                                    t_const = (yaw_2 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                }
                                else if (yaw_2 < -2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                    alpha_max = -ALPHA_MAX*PI/180.0;
                                    t_const = (yaw_2 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                }
                                else{
                                    alpha_max = yaw_2/(2*t_rise*t_rise);
                                    t_const = 0;
                                }
                            
                                state = 3;
                            }
                        }
                        else {
                            state = 4;
                        }
                    }
                    else {      // Progress with the previous path planning
                        br.sendTransform(tf::StampedTransform(final_goal, ros::Time::now(), "world", "goal"));
                        t = (ros::Time::now() - t_start).toSec();
                        float delta_t = t-t_old;
                        tf::StampedTransform blimp;
                        ROS_INFO("%d", state);
                        
                        switch (state) {
                            case 1:
                                if (t < t_rise)
                                    alpha = alpha_old + alpha_max/t_rise*delta_t;
                                else if (t < 2*t_rise)
                                    alpha = alpha_old - alpha_max/t_rise*delta_t;
                                else if (t < 2*t_rise + t_const)
                                    alpha = 0;
                                else if (t < 3*t_rise + t_const)
                                    alpha = alpha_old - alpha_max/t_rise*delta_t;
                                else if (t < 4*t_rise + t_const)
                                    alpha = alpha_old + alpha_max/t_rise*delta_t;
                                else{
                                    alpha = 0;
                                    if (t - (4*t_rise + t_const) > 3.0) {       // Wait 3.0 s
                                        t_start = t_start + ros::Duration(t);
                                        tf::StampedTransform translation;
                                        listener.lookupTransform("blimp", "goal", ros::Time(0), translation);
                                        d = translation.getOrigin();
                                        distance = d.x();
                                        
                                        if (fabs(distance) < 0.1) {
                                            yaw_2 = goal_yaw*PI/180.0 - y_blimp;
                                            if (yaw_2 > PI)
                                                yaw_2 = yaw_2 - 2*PI;
                                            else if (yaw_2 <= -PI)
                                                yaw_2 = yaw_2 + 2*PI;
                                            ROS_INFO("%.2f", yaw_2*180/PI);
                                                
                                            alpha_old = 0;
                                            omega_old = 0;
                                            a_old = 0;
                                            v_old = 0;
                                            if (yaw_2 > 2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                                alpha_max = ALPHA_MAX*PI/180.0;
                                                t_const = (yaw_2 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                            }
                                            else if (yaw_2 < -2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                                alpha_max = -ALPHA_MAX*PI/180.0;
                                                t_const = (yaw_2 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                            }
                                            else{
                                                alpha_max = yaw_2/(2*t_rise*t_rise);
                                                t_const = 0;
                                            }
                                            state = 3;
                                        }
                                        else if (distance > 2*A_MAX*t_rise*t_rise) {
                                            a_max = A_MAX;
                                            t_const = (distance - 2*a_max*t_rise*t_rise)/(a_max*t_rise);
                                            state = 2;
                                        }
                                        else if (distance < -2*A_MAX*t_rise*t_rise) {
                                            a_max = -A_MAX;
                                            t_const = (distance - 2*a_max*t_rise*t_rise)/(a_max*t_rise);
                                            state = 2;
                                        }
                                        else{
                                            a_max = distance/(2*t_rise*t_rise);
                                            t_const = 0;
                                            state = 2;
                                        }
                                        pid_thrust.reset();
                                        t_old = 0;
                                        move_origin = current;
                            
                                        br.sendTransform(tf::StampedTransform(move_origin, ros::Time::now(), "world", "start"));
                                        break;
                                    }
                                }
                                omega = omega_old + (alpha_old+alpha)/2*delta_t;
                                command_yaw = command_yaw + (omega_old+omega)/2*delta_t;
                                
                                alpha_old = alpha;
                                omega_old = omega;
                                t_old = t;
                                msg.angular.z = command_yaw;
                                break;
                            case 2:
                                br.sendTransform(tf::StampedTransform(move_origin, ros::Time::now(), "world", "start"));
                                listener.lookupTransform("start", "blimp", ros::Time(0), blimp);
                                
                                if (t < t_rise)
                                    a = a_old + a_max/t_rise*delta_t;
                                else if (t < 2*t_rise)
                                    a = a_old - a_max/t_rise*delta_t;
                                else if (t < 2*t_rise + t_const)
                                    a = 0;
                                else if (t < 3*t_rise + t_const)
                                    a = a_old - a_max/t_rise*delta_t;
                                else if (t < 4*t_rise + t_const)
                                    a = a_old + a_max/t_rise*delta_t;
                                else{
                                    a = 0;
                                    if (t - (4*t_rise + t_const) > 2.0) {       // Wait 2.0 s
                                        t_start = t_start + ros::Duration(t);
                                        
                                        yaw_2 = goal_yaw*PI/180.0 - y_blimp;
                                        if (yaw_2 > PI)
                                            yaw_2 = yaw_2 - 2*PI;
                                        else if (yaw_2 <= -PI)
                                            yaw_2 = yaw_2 + 2*PI;
                                        ROS_INFO("%.2f", yaw_2*180/PI);
                                            
                                        alpha_old = 0;
                                        omega_old = 0;
                                        a_old = 0;
                                        v_old = 0;
                                        v = 0;
                                        x = 0;
                                        if (yaw_2 > 2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                            alpha_max = ALPHA_MAX*PI/180.0;
                                            t_const = (yaw_2 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                        }
                                        else if (yaw_2 < -2*ALPHA_MAX*t_rise*t_rise*PI/180.0) {
                                            alpha_max = -ALPHA_MAX*PI/180.0;
                                            t_const = (yaw_2 - 2*alpha_max*t_rise*t_rise)/(alpha_max*t_rise);
                                        }
                                        else{
                                            alpha_max = yaw_2/(2*t_rise*t_rise);
                                            t_const = 0;
                                        }
                                        command_thrust = 0;
                                        t_old = 0;
                                        if (fabs(yaw_2) < PI/(180.0*5))
                                            state = 0;
                                        else
                                            state = 3;
                                        break;
                                    }
                                }
                                v = v_old + (a_old+a)/2*delta_t;
                                x = x + (v_old+v)/2*delta_t;
                                
                                a_old = a;
                                v_old = v;
                                t_old = t;
                                
                                command_thrust = pid_thrust.update(x-blimp.getOrigin().x());
                                msg.linear.x = command_thrust;
                                break;
                            case 3:
                                if (t < t_rise)
                                    alpha = alpha_old + alpha_max/t_rise*delta_t;
                                else if (t < 2*t_rise)
                                    alpha = alpha_old - alpha_max/t_rise*delta_t;
                                else if (t < 2*t_rise + t_const)
                                    alpha = 0;
                                else if (t < 3*t_rise + t_const)
                                    alpha = alpha_old - alpha_max/t_rise*delta_t;
                                else if (t < 4*t_rise + t_const)
                                    alpha = alpha_old + alpha_max/t_rise*delta_t;
                                else{
                                    alpha = 0;
                                    if (t - (4*t_rise + t_const) > 2.0) {       // Wait 2.0 s
                                        t_old = 0;
                                        state = 0;
                                        break;
                                    }
                                }
                                omega = omega_old + (alpha_old+alpha)/2*delta_t;
                                command_yaw = command_yaw + (omega_old+omega)/2*delta_t;
                                
                                alpha_old = alpha;
                                omega_old = omega;
                                t_old = t;
                                msg.angular.z = command_yaw;
                                break;
                            case 4:
                                // Immediate stop before creating new path
                                break;
                            default:
                                ROS_INFO("Stopped");
                        }
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
        blimp_controller.navigate();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
