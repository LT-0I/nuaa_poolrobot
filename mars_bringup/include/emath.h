#ifndef __EMATH_H__
#define __EMATH_H__

#include <public.h>
#include <iostream>
#include <math.h>
#include <cstring>
#include <geometry_msgs/Pose2D.h>


class Emath
{
public:
	double Dot(Point a, Point b);
    double Cross(Point a, Point b);
	double modValue(Point a);
    double modSquare(Point a);
	double angleConstrain(double fai);
    double valueConstrain(double value, double up_limit, double dp_limit);
    double angleTo2pi(double fai);
    double angleConstrainTo2pi(double fai);
    double pointDist(Point a, Point b);
    double PoseDist(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b);
    double pointPoseDist(Point a, geometry_msgs::Pose2D b);
    void   intToByte(int i, uint8_t* bytes);
    int    bytesToInt(uint8_t* bytes);
    double toRadian(double degree);
    double toDegree(double radian);

};

#endif