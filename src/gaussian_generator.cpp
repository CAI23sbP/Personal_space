#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <pedsim_msgs/AgentStates.h>
#include "gaussian_generator/SingleGaussian.h"
#include "gaussian_generator/MultiGaussians.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher gau_pub_;
std::string p_frame_id_;
int image_size_ ;


float AsymmetryGaussian(
                        float dx, 
                        float dy, 
                        float sig_h,
                        float sig_s,
                        float sig_r ,
                        float theta,
                        float pi
                        )
{   

    float alpha = std::atan2(dy,dx) - theta  + pi/2;
    alpha = std::fmod((alpha + pi),(2 * pi) ) - pi;
    float sig = (alpha <= 0) ? sig_h : sig_r;
    // #(math.pow(math.cos(theta),2) / (2 * math.pow(sig,2)) )+ (math.pow(math.sin(theta),2) / (2 * math.pow(sig_s,2)))
    float a = std::pow(std::cos(theta), 2) / (2 * std::pow(sig, 2)) + std::pow(std::sin(theta), 2) / (2 * std::pow(sig_s, 2));
    // (math.sin(2 * theta)) / (4 * math.pow(sig,2)) - (math.sin(2 * theta)) / (4 * math.pow(sig_s,2))
    float b = std::sin(2 * theta) / (4 * std::pow(sig, 2)) - std::sin(2 * theta) / (4 * std::pow(sig_s, 2));
    
    // math.pow(math.sin(theta),2) / (2 * math.pow(sig,2)) + math.pow(math.cos(theta),2) / (2 * math.pow(sig_s,2))

    float c = std::pow(std::sin(theta), 2) / (2 * std::pow(sig, 2)) + std::pow(std::cos(theta), 2) / (2 * std::pow(sig_s, 2));
    // math.exp(-(a * math.pow(dx,2) + 2 * b * dx * dy + c * math.pow(dy,2)))*100
    return exp(-( a * std::pow(dx, 2) + 2 * b * dx * dy + c * std::pow(dy, 2) ) )*100;
}

void PedOdomCallback(const pedsim_msgs::AgentStates::ConstPtr& crowd ,const sensor_msgs::LaserScan::ConstPtr& scan, const nav_msgs::Odometry::ConstPtr& odom)
{   
    float dY = 0;
    float dX = 0;
    float Vx = 0;
    float Vy = 0;
    float default_ = 0.5; 
    float range =  scan->range_max; 
    int image_size = image_size_; // Replace with your desired image size
    int num = crowd->agent_states.size();
    gaussian_generator::MultiGaussians multi_gaussian_;
    float pi = M_PI ;
    multi_gaussian_.header.frame_id = p_frame_id_;
    multi_gaussian_.header.stamp = ros::Time::now();
    multi_gaussian_.size = image_size;
    float value  = 0 ;

    if (num > 0)
        for(unsigned int k=0; k<num ;k++)
        {   
            gaussian_generator::SingleGaussian single_gaussian_; 
            dX = crowd->agent_states[k].pose.position.x - odom->pose.pose.position.x;
            dY = crowd->agent_states[k].pose.position.y - odom->pose.pose.position.y;
         
            
            if (sqrt(std::pow(dX,2) + std::pow(dY,2))<range) 
            {
                Vx = crowd->agent_states[k].twist.linear.x;
                Vy = crowd->agent_states[k].twist.linear.y;

                float norm = sqrt(std::pow(Vx, 2) + std::pow(Vy, 2));
                float theta = std::atan2(Vy ,Vx);
                float sig_h = std::max(2 * norm, default_ );
                float sig_s = (2.0 / 3) * sig_h;
                float sig_r = 0.5 * sig_h;
                
               
                single_gaussian_.center.x = dX;
                single_gaussian_.center.y = dY;
                single_gaussian_.center.theta = theta;
                single_gaussian_.velocity.x = Vx;
                single_gaussian_.velocity.y = Vy;
            
                
                for (int i = 0; i < image_size; ++i) 
                {
                    float y = -5.0 + (i * 10.0) / (image_size - 1);

                    for (int j = 0; j < image_size; ++j) 
                    {
                        float x = -5.0 + (j * 10.0) / (image_size - 1) ;
                        value = AsymmetryGaussian(
                                                x,
                                                y,  
                                                sig_h, 
                                                sig_s, 
                                                sig_r, 
                                                theta, 
                                                pi
                                                );
                        uint8_t uintValue = static_cast<uint8_t>(value);
                        single_gaussian_.data.push_back(uintValue);
                    }
                }
                
                multi_gaussian_.gaussian.push_back(single_gaussian_);
            } 
            else
            {   
                

                single_gaussian_.center.x = 0;
                single_gaussian_.center.y = 0;
                single_gaussian_.center.theta = 0;
                single_gaussian_.velocity.x = 0;
                single_gaussian_.velocity.y = 0;
                for (int i = 0; i < image_size; ++i)
                {
                    float y = -5.0 + (i * 10.0) / (image_size - 1) ;

                    for (int j = 0; j < image_size; ++j) 
                    {
                        float x = -5.0 + (j * 10.0) / (image_size - 1) ;
                        uint8_t uintValue = static_cast<uint8_t>(0.0);
                        single_gaussian_.data.push_back(uintValue);
                    }
                }
                
                multi_gaussian_.gaussian.push_back(single_gaussian_);

            }
        }

    else
    {   
        for(unsigned int k=0; k<5 ;k++)
        { 
            gaussian_generator::SingleGaussian single_gaussian_; 
            single_gaussian_.center.x = 0;
            single_gaussian_.center.y = 0;
            single_gaussian_.center.theta = 0;
            single_gaussian_.velocity.x = 0;
            single_gaussian_.velocity.y = 0;
            for (int i = 0; i < image_size; ++i) 
            {
                float y = -5.0 + (i * 10.0) / (image_size - 1);

                for (int j = 0; j < image_size; ++j) 
                {
                    float x = -5.0 + (j * 10.0) / (image_size - 1);
                    uint8_t uintValue = static_cast<uint8_t>(0.0);
                    single_gaussian_.data.push_back(uintValue);
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
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    //TODO
    message_filters::Subscriber<pedsim_msgs::AgentStates> ped_sub(nh,"pedsim_simulator/simulated_agents",1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh,"scan",1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"odom",1);
    message_filters::TimeSynchronizer<pedsim_msgs::AgentStates, sensor_msgs::LaserScan, nav_msgs::Odometry> sync(ped_sub,scan_sub, odom_sub, 10);
    sync.registerCallback(boost::bind(&PedOdomCallback, _1, _2,_3));
    nh_priv.param<std::string>("frame_id", p_frame_id_, "base_footprint");
    nh_priv.param<int>("image_size", image_size_, 100);
    gau_pub_ = nh.advertise<gaussian_generator::MultiGaussians>("/gaussian_status", 1);

    ros::spin(); 
}
