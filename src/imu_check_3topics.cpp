//ros basic
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

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

int arr_fin = 0;
int a_cnt = 0;
int g_cnt = 0;
int set_condition = 0;

void tdataCallback(const std_msgs::Float64::ConstPtr& getdata){

    arr_fin = arr_fin + 1;
    float t = getdata->data;
    t_array.push_back(t);
    //ROS_INFO("ok");
}

void adataCallback(const geometry_msgs::Vector3::ConstPtr& getdata){
    a_cnt = a_cnt + 1;
    
    float a_x = getdata->x;
    float a_y = getdata->y;
    float a_z = getdata->z;
    ax_array.push_back(a_x);
    ay_array.push_back(a_y);
    az_array.push_back(a_z);
    while( a_cnt < arr_fin ){
        a_cnt = a_cnt + 1;
        ax_array.push_back(0);
        ay_array.push_back(0);
        az_array.push_back(0);
    }
    
}

void gdataCallback(const geometry_msgs::Vector3::ConstPtr& getdata){
    g_cnt = g_cnt + 1;

    float g_x = getdata->x;
    float g_y = getdata->y;
    float g_z = getdata->z;
    gx_array.push_back(g_x);
    gy_array.push_back(g_y);
    gz_array.push_back(g_z);
    while( g_cnt < arr_fin ){
        g_cnt = g_cnt + 1;
        gx_array.push_back(0);
        gy_array.push_back(0);
        gz_array.push_back(0);
    }
}


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "imu_check_3topics_node");
	ros::NodeHandle n;
	ros::Subscriber tsub = n.subscribe<std_msgs::Float64>("time", 1, tdataCallback);
    ros::Subscriber asub = n.subscribe<geometry_msgs::Vector3>("acceldata", 1, adataCallback);
    ros::Subscriber gsub = n.subscribe<geometry_msgs::Vector3>("gyrodata", 1, gdataCallback);

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