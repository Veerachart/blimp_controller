#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

const double PI = 3.141592653589793;

class Blimp{
    private:
        ros::NodeHandle nh;
        float x,y,z,psi;
        float u,v,w,q;
        float prev_x, prev_y, prev_z, prev_psi;
        float prev_u, prev_v, prev_w, prev_q;
        float M_x, C_x, D_x;
        float I_zz, C_q, D_q;
        float ux;
        float upsi;
        float u_dot;
        float q_dot;
        ros::Time current;
        ros::Time last;
        tf::TransformListener listener;
        tf::TransformBroadcaster broadcaster;
        tf::Transform transform;
        tf::Quaternion rot;
        ros::Subscriber command_sub;
        ros::Subscriber turn_sub;
    public:
        Blimp (ros::NodeHandle &nh_, float x_ ,float y_, float z_){
            nh = nh_;
            x = x_;
            y = y_;
            z = z_;
            psi = 0;
            u = 0;
            v = 0;
            w = 0;
            q = 0;
            prev_x = 0;
            prev_y = 0;
            prev_z = 0;
            prev_psi = 0;
            prev_u = 0;
            prev_v = 0;
            prev_w = 0;
            prev_q = 0;
            M_x = 0.1;
            C_x = 0.0;
            D_x = 0.1;
            C_q = 0.0;
            D_q = 0.05;
            I_zz= 0.03;
            last = ros::Time::now();
            command_sub = nh.subscribe("thrust", 1, &Blimp::thrust_cb, this);
            turn_sub = nh.subscribe("yaw_cmd", 1, &Blimp::turn_cb, this);
        }
        
        void update (){
            current = ros::Time::now();
            // x direction
            u_dot = (ux - C_x*prev_u - D_x*prev_u)/M_x;
            u = prev_u + u_dot * (current-last).toSec();
            //u = prev_u + u_dot * 0.033;
            x = prev_x + u * cos(psi) * (current-last).toSec();
            //x = prev_x + u * 0.033;
            y = prev_y + u * sin(psi) * (current-last).toSec();
            
            // rotation
            float e_psi = upsi - psi;
            float uq = 0.03*e_psi;
            q_dot = (uq -C_q*prev_q - D_q*prev_q)/I_zz;
            q = prev_q + q_dot * (current-last).toSec();
            psi = prev_psi + q * (current-last).toSec();
            prev_u = u;
            prev_q = q;
            prev_x = x;
            prev_y = y;
            prev_psi = psi;
            last = current;
            transform.setOrigin(tf::Vector3(x,y,z));
            rot.setRPY(0,0,psi);
            transform.setRotation(rot);
            broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","blimp"));
        }
        
        void thrust_cb(const std_msgs::Float32::ConstPtr& msg){
            ux = msg->data;
        }
        void turn_cb(const std_msgs::Float32::ConstPtr& msg){
            upsi = msg->data;
        }
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "blimp_simulator", ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    ros::Rate rate(30);
    Blimp blimp(nh, 0.0, 0.0, 0.0);
    while (nh.ok()){
        blimp.update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
