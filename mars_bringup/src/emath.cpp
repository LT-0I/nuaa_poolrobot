#include <emath.h>
double Emath::Dot(Point a, Point b ){
	return (a.x*b.x + a.y*b.y);
}


double Emath::Cross(Point a, Point b){
	return( a.x*b.y - a.y*b.x);
}

double Emath::modValue(Point a){
	return sqrt(a.x*a.x + a.y*a.y);
}

double Emath::modSquare(Point a){
	return (a.x*a.x + a.y*a.y);
}

double Emath::angleConstrain(double fai){
    if (fai>=PI_M)
	{
		fai=fai - 2*PI_M;
	}else if (fai<-PI_M)
	{
		fai=fai + 2*PI_M;
	}
	return fai;
}

double Emath::valueConstrain(double value, double up_limit, double dp_limit) 
{
	if(value > up_limit){
		value = up_limit;
	}
	if(value < dp_limit){
		value = dp_limit;
	}
	return value;
}

double Emath::angleTo2pi(double fai){
    if(fai < 0){
        fai = fai + 2 * PI_M;
    }else{
        fai = fai;
    }
    return fai;
}

double Emath::angleConstrainTo2pi(double fai)
{
	// 使用 fmod 函数进行模运算，确保结果在 [0, 2*PI_M)
	fai = std::fmod(fai, 2 * PI_M);

	// 如果 fai 为负数，则加上 2*PI_M 将其调整到 [0, 2*PI_M)
	if (fai < 0.0)
	{
		fai += 2 * PI_M;
	}

	return fai;
}

double Emath::pointDist(Point a, Point b)
{
	return sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
}

double Emath::PoseDist(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b)
{
	return sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
}

double Emath::pointPoseDist(Point a, geometry_msgs::Pose2D b)
{
	return sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
}

void Emath::intToByte(int i, uint8_t* bytes)
{
	size_t length = sizeof(int);
	// 初始化数组
    memset(bytes, 0, sizeof(uint8_t) * length);
    bytes[0] = (uint8_t)(0xff & i);
    bytes[1] = (uint8_t)((0xff00 & i) >> 8);
    bytes[2] = (uint8_t)((0xff0000 & i) >> 16);
    bytes[3] = (uint8_t)((0xff000000 & i) >> 24);
    return;
}
int Emath::bytesToInt(uint8_t* bytes)
{
    // 位操作时 使用一个unsigned int变量来作为位容器。
    int addr = bytes[0] & 0xFF;
    addr |= ((bytes[1] << 8) & 0xFF00);
    addr |= ((bytes[2] << 16) & 0xFF0000);
    addr |= ((bytes[3] << 24) & 0xFF000000);
    return addr;
}

double Emath::toRadian(double degree)
{
	double radian = degree/180*PI_M;
	return radian;
}
double Emath::toDegree(double radian)
{
	double degree = radian/PI_M*180;
	return degree;
}