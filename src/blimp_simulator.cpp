#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

class Blimp{
    private:
        ros::NodeHandle nh;
        float x,y,z,psi;
        float u,v,w,q;
        float prev_x, prev_y, prev_z, prev_psi;
        float prev_u, prev_v, prev_w, prev_q;
        float M_x, C_x, D_x;
        float ux;
        float u_dot;
        ros::Time current;
        ros::Time last;
        tf::TransformListener listener;
        tf::TransformBroadcaster broadcaster;
        tf::Transform transform;
        tf::Quaternion rot;
        ros::Subscriber command_sub;
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
            last = ros::Time::now();
            command_sub = nh.subscribe("thrust", 1, &Blimp::thrust_cb, this);
        }
        
        void update (){
            current = ros::Time::now();
            u_dot = (ux - C_x*prev_u - D_x*prev_u)/M_x;
            u = prev_u + u_dot * (current-last).toSec();
            //u = prev_u + u_dot * 0.033;
            x = prev_x + u * (current-last).toSec();
            //x = prev_x + u * 0.033;
            prev_u = u;
            prev_x = x;
            last = current;
            transform.setOrigin(tf::Vector3(x,y,z));
            rot.setRPY(0,0,psi);
            transform.setRotation(rot);
            broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","blimp"));
        }
        
        void thrust_cb(const std_msgs::Float32::ConstPtr& msg){
            ux = msg->data;
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
