#include "ywmap.h"
#include <cstdio>
#define DEBUG_3
using namespace cv;
using namespace std;
using namespace pugi;

cv::Scalar YWMap::hex2BGR(std::string hex)
{
	std::string r = hex.substr(1,2);
	std::string g = hex.substr(3,2);
	std::string b = hex.substr(5,2);
	b = std::string("0x") + b;
	g = std::string("0x") + g;
	r = std::string("0x") + r;
	int B,G,R;
	sscanf(b.c_str(),"%X", &B);
	sscanf(g.c_str(),"%X", &G);
	sscanf(r.c_str(),"%X", &R);
	return cv::Scalar(B,G,R);
}

cv::Point2d YWMap::p2P(point v, point p, double scale)
{
	return cv::Point2d((v.get<1>() - p.get<1>()) * scale, (p.get<0>() - v.get<0>()) * scale);
}
