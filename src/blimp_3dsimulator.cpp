#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <Eigen/Core>
#include <Eigen/Dense>

const double PI = 3.141592653589793;
const double g = 9.81;

using namespace Eigen;

class Blimp{
    private:
        ros::NodeHandle nh;
        Vector4f X;
        Vector4f V;
        Vector4f X_old;
        Vector4f V_old;
        Matrix4f M, C, D, J;
        Vector4f T;
        Vector4f G;
        Vector4f V_dot, X_dot;
        float m, I, X_udot, Y_vdot, Z_wdot, N_rdot;
        float X_uu, Y_vv, Z_ww, N_rr;
        float W;        // Gravitational force
        float B;        // Buoyancy force
        float upsi;
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
            X << x_, y_, z_, 0;
            V << 0, 0, 0, 0;
            X_old = X;
            V_old = V;
            m = 0.1;
            I = 0.05;
            X_udot = 0.001;
            Y_vdot = 0.001;
            Z_wdot = 0.002;
            N_rdot = 0.0001;
            M << m-X_udot, 0, 0, 0,
                 0, m-Y_vdot, 0, 0,
                 0, 0, m-Z_wdot, 0,
                 0, 0, 0, I-N_rdot;
            X_uu = -0.2;
            Y_vv = -0.2;
            Z_ww = -0.5;
            N_rr = -0.2;
            W = m*g;
            B = W;
            G << 0, 0, W-B, 0;
            C << Matrix4f::Zero();
            T = Vector4f::Zero();
            upsi = 0;
            last = ros::Time::now();
            command_sub = nh.subscribe("thrust", 1, &Blimp::thrust_cb, this);
            turn_sub = nh.subscribe("yaw_cmd", 1, &Blimp::turn_cb, this);
        }
        
        void update (){
            current = ros::Time::now();
            C(0,3) = -(m-Y_vdot)*V(1);
            C(1,3) =  (m-X_udot)*V(0);
            C(3,3) =  (m-Y_vdot)*V(1);
            C(3,1) = -(m-X_udot)*V(0);
            
            Vector4f d(-X_uu*abs(V(0)), -Y_vv*abs(V(1)), -Z_ww*abs(V(2)), -N_rr*abs(V(3)));
            D = d.asDiagonal();
            
            J << cos(X(3)), -sin(X(3)), 0, 0,
                 sin(X(3)),  cos(X(3)), 0, 0,
                         0,          0, 1, 0,
                         0,          0, 0, 1;
            
            // rotation
            float e_psi = upsi - X(3);
            T(3) = 0.03*e_psi;
            V_dot = M.inverse() * (T - C*V - D*V);
            V = V + V_dot*(current-last).toSec();
            X_dot = J*V;
            X = X + X_dot*(current-last).toSec();
            if (X(3) > PI)
                X(3) -= 2*PI;
            else if (X(3) <= -PI)
                X(3) += 2*PI;
            last = current;
            transform.setOrigin(tf::Vector3(X(0),X(1),X(2)));
            rot.setRPY(0,0,X(3));
            transform.setRotation(rot);
            broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","blimp"));
        }
        
        void thrust_cb(const std_msgs::Float32::ConstPtr& msg){
            T(0) = msg->data;
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
