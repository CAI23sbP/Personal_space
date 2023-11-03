#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pedsim_msgs/AgentStates.h>
#include "gaussian_generator/SingleGaussian.h"
#include "gaussian_generator/MultiGaussians.h"

// #include <gaussian_generator/SingleGaussian.h>
// #include <gaussian_generator/MultiGaussians.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher gau_pub_;
std::string p_frame_id_;

void RobustNormalize(int image_size , std::vector<std::vector<float>>& Z, float epsilon)
{


    for (int i = 0; i < image_size; ++i) {
        for (int j = 0; j < image_size; ++j) {
            if (Z[i][j] <= 0.1) {
                Z[i][j] = 0.0;
            }
        }
    }
    for (int i = 0; i < image_size; ++i) {
        for (int j = 0; j < image_size; ++j) {
            Z[i][j] = (Z[i][j] - 0.5) / (0.75 + epsilon - 0.25);
        }
    }


}

float AsymmetryGaussian(float x,float y, float vx, float vy ,float xc, float yc, float sig_h,float sig_s,float sig_r ,float theta)
{   
    float alpha = atan2(y - yc, x - xc) - theta + M_PI / 2;
    alpha = fmod((alpha + M_PI), (2 * M_PI)) - M_PI;
    float sig = (alpha <= 0) ? sig_r : sig_h;
    float a = pow(cos(theta), 2) / (2 * pow(sig, 2)) + pow(sin(theta), 2) / (2 * pow(sig_s, 2));
    float b = sin(2 * theta) / (4 * pow(sig, 2)) - sin(2 * theta) / (4 * pow(sig_s, 2));
    float c = pow(sin(theta), 2) / 2 * pow(sig, 2) + pow(cos(theta), 2) / (2 * pow(sig_s, 2));

    return exp(-(a * pow(x - xc, 2) + 2 * b * (x - xc) * (y - yc) + c * pow(y - yc, 2)));
}

void PedOdomCallback(const pedsim_msgs::AgentStates::ConstPtr& crowd ,const sensor_msgs::LaserScan::ConstPtr& scan, const nav_msgs::Odometry::ConstPtr& odom)
{   
    float X = 0;
    float Y = 0;
    float Vx = 0;
    float Vy = 0;
    float default_ = 0.5; 
    float range =  scan->range_max; 
    int image_size = 30; // Replace with your desired image size
    float epsilon = 1e-6; // Small epsilon value
    int num = crowd->agent_states.size();
    gaussian_generator::MultiGaussians multi_gaussian_;

    multi_gaussian_.header.frame_id = p_frame_id_;
    multi_gaussian_.header.stamp = ros::Time::now();
    multi_gaussian_.size = image_size;

    if (num > 0)
        for(unsigned int j=0; j<num ;j++)
        {   
            gaussian_generator::SingleGaussian single_gaussian_; 
            X = crowd->agent_states[j].pose.position.x - odom->pose.pose.position.x;
            Y = crowd->agent_states[j].pose.position.x - odom->pose.pose.position.y;
            std::vector<std::vector<float>> Z(image_size, std::vector<float>(image_size, 0.0));
            if (sqrt(X*X + Y*Y)<range) 
            {
                Vx = crowd->agent_states[j].twist.linear.x;
                Vy = crowd->agent_states[j].twist.linear.y;
                

                float norm = sqrt(pow(Vx, 2) + pow(Vy, 2));
                float radian = atan2(Vx, Vy);
                float sig_h = std::max(2 * norm,default_ );
                float sig_s = (2.0 / 3) * sig_h;
                float sig_r = 0.5 * sig_h;
                
                for (int i = 0; i < image_size; ++i) {
                    for (int j = 0; j < image_size; ++j) {
                        float x = -5.0 + (i * 10.0) / (image_size - 1) + Y;
                        float y = -5.0 + (j * 10.0) / (image_size - 1) + X;
                        Z[i][j] = AsymmetryGaussian(x, y, Vx, Vy, X, Y,sig_h, sig_s,sig_r, radian);
                    }
                }
                
                RobustNormalize(image_size,Z,epsilon);
                
                single_gaussian_.center.x = X;
                single_gaussian_.center.y = Y;
                single_gaussian_.center.theta = radian;
                single_gaussian_.velocity.x = Vx;
                single_gaussian_.velocity.x = Vy;
                for (const std::vector<float>& row : Z) {
                    for (float value : row) {
                        single_gaussian_.data.push_back(value);
                    }
                }
                multi_gaussian_.gaussian.push_back(single_gaussian_);
            } 
            else
            {   
                
                X = 0;
                Y = 0; 
                Vx = 0;
                Vy = 0;

                float norm = sqrt(pow(Vx, 2) + pow(Vy, 2));
                float radian = atan2(Vx, Vy);
                float sig_h = std::max(2 * norm,default_ );
                float sig_s = (2.0 / 3) * sig_h;
                float sig_r = 0.5 * sig_h;

                for (int i = 0; i < image_size; ++i) {
                    for (int j = 0; j < image_size; ++j) {
                        float x = -5.0 + (i * 10.0) / (image_size - 1) + Y;
                        float y = -5.0 + (j * 10.0) / (image_size - 1) + X;
                        Z[i][j] = AsymmetryGaussian(x, y, Vx, Vy, X, Y,sig_h, sig_s,sig_r, radian);
                    }
                }
                
                RobustNormalize(image_size,Z,epsilon);

                single_gaussian_.center.x = X;
                single_gaussian_.center.y = Y;
                single_gaussian_.center.theta = 0;
                single_gaussian_.velocity.x = Vx;
                single_gaussian_.velocity.x = Vy;
                for (const std::vector<float>& row : Z) {
                    for (float value : row) {
                        single_gaussian_.data.push_back(value);
                    }
                }
                multi_gaussian_.gaussian.push_back(single_gaussian_);

            }
        }

    else
    {   
        std::vector<std::vector<float>> Z(image_size, std::vector<float>(image_size, 0.0));
        for(unsigned int j=0; j<5 ;j++)
        { 
            gaussian_generator::SingleGaussian single_gaussian_; 
            X = 0;
            Y = 0; 
            Vx = 0;
            Vy = 0;

            float norm = sqrt(pow(Vx, 2) + pow(Vy, 2));
            float radian = atan2(Vx, Vy);
            float sig_h = std::max(2 * norm,default_ );
            float sig_s = (2.0 / 3) * sig_h;
            float sig_r = 0.5 * sig_h;

            for (int i = 0; i < image_size; ++i) {
                for (int j = 0; j < image_size; ++j) {
                    float x = -5.0 + (i * 10.0) / (image_size - 1) + Y;
                    float y = -5.0 + (j * 10.0) / (image_size - 1) + X;
                    Z[i][j] = AsymmetryGaussian(x, y, Vx, Vy, X, Y,sig_h, sig_s,sig_r, radian);
                }
            }
            single_gaussian_.center.x = X;
            single_gaussian_.center.y = Y;
            single_gaussian_.center.theta = 0;
            single_gaussian_.velocity.x = Vx;
            single_gaussian_.velocity.x = Vy;
            for (const std::vector<float>& row : Z) {
                for (float value : row) {
                    single_gaussian_.data.push_back(value);
                }
            }
            multi_gaussian_.gaussian.push_back(single_gaussian_);

        }
    }
    gau_pub_.publish(multi_gaussian_);
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"asymmetric_gaussian_generator");
    // ros::NodeHandle public_nh("");
    ros::NodeHandle nh;

    //TODO
    message_filters::Subscriber<pedsim_msgs::AgentStates> ped_sub(nh,"pedsim_simulator/simulated_agents",1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh,"scan",1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"odom",1);
    message_filters::TimeSynchronizer<pedsim_msgs::AgentStates, sensor_msgs::LaserScan, nav_msgs::Odometry> sync(ped_sub,scan_sub, odom_sub, 10);
    sync.registerCallback(boost::bind(&PedOdomCallback, _1, _2,_3));
    nh.param<std::string>("/frame_id", p_frame_id_, "base_footprint");
    gau_pub_ = nh.advertise<gaussian_generator::MultiGaussians>("/gaussian_status", 1);

    ros::spin(); 
}