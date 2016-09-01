#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class HSVcolor{
private:
	double H;
	double S;
	double V;

public:
	HSVcolor(){
		H = S = V = 0.0;
	}

	HSVcolor(double h, double s, double v){
		H = h;
		S = s;
		V = v;
	}

	double& operator[](int c){
		if (c == 0)return H;
		else if (c == 1)return S;
		else return V;
	}

	bool operator<(HSVcolor &a){
		return ((*this)[0] < a[0] && (*this)[1] < a[1] && (*this)[2] < a[2]);
	}

	bool operator>(HSVcolor &a){
		return ((*this)[0] > a[0] && (*this)[1] > a[1] && (*this)[2] > a[2]);
	}

	bool operator<=(HSVcolor &a){
		return ((*this)[0] <= a[0] && (*this)[1] <= a[1] && (*this)[2] <= a[2]);
	}

	bool operator>=(HSVcolor &a){
		return ((*this)[0] >= a[0] && (*this)[1] >= a[1] && (*this)[2] >= a[2]);
	}

	bool operator==(HSVcolor &a){
		return ((*this)[0] == a[0] && (*this)[1] == a[1] && (*this)[2] == a[2]);
	}

	bool operator!=(HSVcolor &a){
		return !((*this)[0] == a[0] && (*this)[1] == a[1] && (*this)[2] == a[2]);
	}
};


cv::Vec3b HSV2BGR(double H, double S = 1.0, double V = 1.0){

	int h = (int)H / 60;
	double F = H / 60.0 - (double)h;
	double P = V * (1.0 - S);
	double Q = V * (1.0 - F * S);
	double T = V * (1.0 - (1.0 - F) * S);

	P *= 255.0;
	Q *= 255.0;
	T *= 255.0;
	V *= 255.0;

	unsigned char p = (unsigned char)P;
	unsigned char q = (unsigned char)Q;
	unsigned char t = (unsigned char)T;
	unsigned char v = (unsigned char)V;

	switch (h){
	case 0: return cv::Vec3b(p, t, v);
	case 1: return cv::Vec3b(p, v, q);
	case 2: return cv::Vec3b(t, v, p);
	case 3: return cv::Vec3b(v, q, p);
	case 4: return cv::Vec3b(v, p, t);
	case 5: return cv::Vec3b(q, p, v);
	}
	return cv::Vec3b(0, 0, 0);
}

cv::Vec3b HSV2BGR(HSVcolor color){
	return HSV2BGR(color[0], color[1], color[2]);
}

HSVcolor BGR2HSV(int b, int g, int r){
	double B = (double)b / 255.0;
	double G = (double)g / 255.0;
	double R = (double)r / 255.0;
	HSVcolor result;
	double max, min;

	max = MAX(MAX(B, G), R);
	min = MIN(MIN(B, G), R);

	if (max == min)result[0] = 0;
	else if (max == R && g >= b)result[0] = 60.0*(G - B) / (max - min);
	else if (max == R && g < b)result[0] = 60.0*(G - B) / (max - min) + 360.0;
	else if (max == G)result[0] = 60.0*(B - R) / (max - min) + 120.0;
	else if (max == B)result[0] = 60.0*(R - G) / (max - min) + 240.0;

	result[1] = (max <= 0.0 ? 0 : 1.0 - min / max);
	result[2] = max;

	return result;
}

HSVcolor BGR2HSV(cv::Vec3b color){
	return BGR2HSV(color[0], color[1], color[2]);
}

#define BLEND_MODE_OVERLAY 0
#define BLEND_MODE_OVERLAY_GRAYSCALE 1

uchar overlayBlending(uchar target, uchar blend){
	double t = (double)target / 255.0;
	double b = (double)blend / 255.0;
	if (t < 0.5)return (uchar)(2 * t * b * 255.0);
	else return (uchar)((1 - 2 * (1 - t)*(1 - b))*255.0);
}

uchar unOverlayBlending(uchar target, uchar unblend){
	double c = (double)target / 255.0;
	double t = (double)unblend / 255.0;

	if (t < 0.5)return (uchar)(c/(2*t)*255.0);
	else return (uchar)((1 - (1 - c) / (2 * (1 - t)))*255.0);
}

cv::Vec3b blending(cv::Vec3b target, cv::Vec3b blend, int mode=0){

	cv::Vec3b result;
	uchar gray;

	switch (mode){
	case 0:
		result[0] = overlayBlending(target[0], blend[0]);
		result[1] = overlayBlending(target[1], blend[1]);
		result[2] = overlayBlending(target[2], blend[2]);
		break;
	case 1:
		gray = (uchar)(0.299 * (double)target[2] + 0.587 * (double)target[1] + 0.114 * (double)target[0]);
		result[0] = overlayBlending(gray, blend[0]);
		result[1] = overlayBlending(gray, blend[1]);
		result[2] = overlayBlending(gray, blend[2]);
		break;
	default:
		result[0] = overlayBlending(target[0], blend[0]);
		result[1] = overlayBlending(target[1], blend[1]);
		result[2] = overlayBlending(target[2], blend[2]);
	}

	return result;
}

cv::Vec3b separate(cv::Vec3b target, cv::Vec3b blend, int mode = 0){
	cv::Vec3b result;
	uchar gray;
	switch (mode){
	case 0:
		result[0] = overlayBlending(target[0], blend[0]);
		result[1] = overlayBlending(target[1], blend[1]);
		result[2] = overlayBlending(target[2], blend[2]);
		break;
	case 1:
		gray = (uchar)(0.299 * (double)target[2] + 0.587 * (double)target[1] + 0.114 * (double)target[0]);
		result[0] = unOverlayBlending(target[0], gray);
		result[1] = unOverlayBlending(target[1], gray);
		result[2] = unOverlayBlending(target[2], gray);
		break;
	default:
		break;
	}
	return result;
}
