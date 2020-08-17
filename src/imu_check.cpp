//ros basic
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <ros/time.h>

#include <sensor_msgs/Imu.h>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>

using namespace std;

vector<float> t_array;//time array
vector<float> ax_array;//acceleration array
vector<float> ay_array;
vector<float> az_array;
vector<float> gx_array;//gyro array
vector<float> gy_array;
vector<float> gz_array;

int set_condition = 0;
int arr_fin = 0;
float t_sec = 0.0;
unsigned int t_pre;
unsigned int dt;


void imuCallback(const sensor_msgs::Imu::ConstPtr& getdata){
    
    float a_x = getdata->linear_acceleration.x;
    float a_y = getdata->linear_acceleration.y;
    float a_z = getdata->linear_acceleration.z;
    float g_x = getdata->angular_velocity.x;
    float g_y = getdata->angular_velocity.y;
    float g_z = getdata->angular_velocity.z;

    unsigned int t = getdata->header.stamp.toNSec();

    if ( t < t_pre ){
        dt = 4294967295 - t_pre + t;
    }else{
        dt = t - t_pre;
    }
    
    t_pre = t;
    t_sec = t_sec + dt / 1000000000.0;

    ax_array.push_back(a_x);
    ay_array.push_back(a_y);
    az_array.push_back(a_z);
    gx_array.push_back(g_x);
    gy_array.push_back(g_y);
    gz_array.push_back(g_z);
    t_array.push_back(t_sec);

    arr_fin = arr_fin + 1;
    
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "imu_check_node");
	ros::NodeHandle n;
    ros::Subscriber imusub = n.subscribe<sensor_msgs::Imu>("imu/data", 1, imuCallback);
    
    
	ros::Rate loop_rate(10);

	while(ros::ok())
	{

        ofstream outputfile("imu_result.csv");
        if ( set_condition == 0 ){
            outputfile << "time" << "," << "a_x" << "," << "a_y" << "," << "a_z" << "," << "g_x" << "," << "g_y" << "," << "g_z" <<endl;
        }
        for ( int i = 0; i < arr_fin; i++){
            outputfile << t_array[i] << "," << ax_array[i] << "," << ay_array[i] << "," << az_array[i] << "," << gx_array[i] << "," << gy_array[i] << "," << gz_array[i] << endl;
        }
        outputfile.close();


        ros::spinOnce(); //subscribe
        loop_rate.sleep(); 
    
	}
	return 0; 

}