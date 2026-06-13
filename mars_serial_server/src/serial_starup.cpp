/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Bluewhale Robot
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Author: oldking
 *******************************************************************************/

#include "AsyncSerial.h"
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>


// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>


#include <hardware_driver.h>

using namespace std;
// using namespace cv;
int main(int argc, char** argv)
{
    ROS_INFO_STREAM("welcome to mars serial server,please feel free at home!");

    ros::init(argc, argv, "serial_server_node");
    ros::start();

    // 获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    ROS_INFO_STREAM("port:" << port << " baud:" << baud);


    // cv::VideoCapture cap1;
    // cv::VideoCapture cap2;
    // cv::Mat frame1, frame2;
    // cap1.open(0);
    // cap2.open(2);
    // if(!cap1.isOpened() || !cap2.isOpened() ){
    //     std::cerr<<"ERROR! Unable to open camera"<<std::endl;
    //     return 0;
    // }
    // cap1.set(CV_CAP_PROP_FOCUS, 0);
    // cap2.set(CV_CAP_PROP_FOCUS, 0);

    try {
        CallbackAsyncSerial serial(port,baud);
        HardwareDriver hardware_driver(&serial);
   
        serial.setCallback(boost::bind(&HardwareDriver::receiveMessage, &hardware_driver, _1, _2));

        boost::thread motioncontroller_thread(boost::bind(&HardwareDriver::rosNode, &hardware_driver));


        ros::Duration(1).sleep();

        static int frequency_cnt=0;
        ros::Rate r(100);  //发布周期�?50hz
        while (ros::ok())
        {
          // cap1>>frame1;
          // cap2>>frame2;
          // if(!frame1.empty() && !frame2.empty()){
          //   cout <<"aaaaaaa" << endl;
          //   cv::imshow("frame1",frame1);
          //   cv::imshow("frame2",frame2);
          //   cv::waitKey(4);
          // }
            //  cout << "1111111" << endl;
             
          if(frequency_cnt%1 == 0)
          { 
            // hardware_driver.sendAutoChargeCommand();
            // hardware_driver.receiveInfDatas();
          }

          if(frequency_cnt%10 == 0)
          {
              hardware_driver.UpdateImu();

            // hardware_driver.sendUltraCommand();
          }
          if(frequency_cnt%100 == 0){
             
          }
        //   /* frequency 50 Hz */
         
          // if(frequency_cnt%10 == 0)
          // {
          //   hardware_driver.UpdateImage();
          // }

         if(frequency_cnt%5 == 0)
          {
              // hardware_driver.sendUltraCommand(); 
              hardware_driver.UpdateUltra();
            // hardware_driver.UpdateImage();
          }

          frequency_cnt++;
          
          r.sleep();
        }
        quit:
            serial.close();  
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        ros::shutdown();
        return 1;
    }

    ros::shutdown();
    return 0;
}
