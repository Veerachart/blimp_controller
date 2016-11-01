#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <blimp_controller/ControllerConfig.h>


class BlimpController {
    private:
        ros::NodeHandle nh_;
        dynamic_reconfigure::Server<blimp_controller::ControllerConfig> server;
        dynamic_reconfigure::Server<blimp_controller::ControllerConfig>::CallbackType f;
    public:
        BlimpController(ros::NodeHandle &node_)
        {
            nh_ = node_;
            f = boost::bind(&BlimpController::callback, this, _1, _2);
            server.setCallback(f);
        }
        void callback(blimp_controller::ControllerConfig &config, uint32_t level){
            if(config.go){
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(config.groups.goal.x, config.groups.goal.y, config.groups.goal.z) );
                tf::Quaternion q;
                q.setRPY(0, 0, config.groups.goal.yaw);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
            }
        }
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "blimp_controller");//, ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    BlimpController blimp_controller(nh);
    ros::spin();
    return 0;
}
