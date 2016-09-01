#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>

#define CONTOUR_OPENED 0
#define CONTOUR_CLOSED 1
#define CONTOUR_AREA 2
#define CONTOUR_INNER 0
#define CONTOUR_OUTER 1
#define SMALLER_THAN 0
#define GREATER_THAN 1

class Contour{
private:
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<bool> flag;
	cv::Size imageSize;
public:

	Contour(){}

	Contour(cv::Size ImageSize){
		imageSize = ImageSize;
	}

	Contour(cv::Mat &image, int type = CV_RETR_CCOMP){
		imageSize = image.size();
		find(image);
	}

	//image=CV_8UC1 or CV_8UC3
	int find(cv::Mat &image, int type = CV_RETR_CCOMP){
		imageSize = image.size();
		cv::Mat t = image.clone();
		if (t.channels() == 3){
			cv::cvtColor(t, t, CV_BGR2GRAY);
		}
		cv::findContours(t, contours, hierarchy, type, CV_CHAIN_APPROX_SIMPLE);
		for(int i=0;i<contours.size();i++)flag.push_back(true);
		return (int)contours.size();
	}

	//input=CV_8UC3 , output = CV_8UC3
	void draw(cv::Mat &input, cv::Mat &output, int thick=2, bool randomColor = true, cv::Scalar color = cv::Scalar(0, 0, 255)){
		cv::Mat drawing = input.clone();
		cv::RNG rng(12345);
		for (int i = 0; i < contours.size(); i++){
			cv::Scalar c;
			if (randomColor) c = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			else c = color;
			if (flag[i] == false)continue;
			drawContours(drawing, contours, i, c, thick, 8, hierarchy, 0);
		}
		output = drawing;
	}

	//output=CV_8UC3
	void draw(cv::Mat &output,int thick=2, bool randomColor = true, cv::Scalar color = cv::Scalar(0, 0, 255)){
		output = cv::Mat::zeros(imageSize, CV_8UC3);
		draw(output, output, thick, randomColor, color);
	}

	//input=CV_8UC3 , output=CV_8UC3
	void drawPoint(cv::Mat &input, cv::Mat &output){
		cv::Mat drawing = input.clone();
		for (int i = 0; i < contours.size(); i++){
			if (flag[i] == false)continue;
			for (int j = 0; j < contours[i].size(); j++){
				cv::Point pt = contours[i][j];
				cv::circle(drawing, pt, 1, cv::Scalar(255, 255, 255), 1);
			}
		}
		output = drawing;
	}

	void drawPoint(cv::Mat &output){
		output = cv::Mat::zeros(imageSize, CV_8UC3);
		drawPoint(output, output);
	}

	void re(){
		for (int i = 0; i < contours.size(); i++){
			flag[i] = true;
		}
	}

	//type=CONTOUR_OPENED or CONTOUR_CLOSED or CONTOUR_AREA(need 4th param)
	//remain=discard or remain
	//type2=CONTOUR_INNER or CONTOUR_OUTER
	//area=area size
	//type=CONTOUR_OPENED or CONTOUR_CLOSED or CONTOUR_AREA(4th param), remain=remain/discard, type2(if remain)=CONTOUR_INNER or CONTOUR_OUTER, area=area size
	void fliter(int type = CONTOUR_OPENED, bool remain = false, int type2 = CONTOUR_INNER, double area = 0.0){
		int n = (type << 2) + (type2 << 1) + (remain ? 1 : 0);
		for (int i = 0; i < contours.size(); i++){
			if (flag[i] == false)continue;
			switch (n){
			case 0x0: //00 0 0
			case 0x2: //00 1 0
				if (hierarchy[i][2] == -1 && hierarchy[i][3] == -1)flag[i] = false;
				break;
			case 0x1: //00 0 1
			case 0x3: //00 1 1
				if (!(hierarchy[i][2] == -1 && hierarchy[i][3] == -1))flag[i] = false;
				break;
			case 0x4: //01 0 0
			case 0x6: //01 1 0
				if (hierarchy[i][2] != -1 || hierarchy[i][3] != -1)flag[i] = false;
				break;
			case 0x5: //01 0 1
				if (!(hierarchy[i][2] == -1 && hierarchy[i][3] != -1))flag[i] = false;
				break;
			case 0x7: //01 1 1
				if (!(hierarchy[i][3] == -1 && hierarchy[i][2] != -1))flag[i] = false;
				break;
			case 0x8: //10 0 0
				if (cv::contourArea(contours[i]) < area)flag[i] = false;
				break;
			case 0x9: //10 0 1
				if (!(cv::contourArea(contours[i]) < area))flag[i] = false;
				break;
			case 0xA: //10 1 0
				if (cv::contourArea(contours[i]) > area)flag[i] = false;
				break;
			case 0xB: //10 1 1
				if (!(cv::contourArea(contours[i]) > area))flag[i] = false;
				break;
			default:
				break;
			}
		}
	}

	size_t size(){
		return contours.size();
	}

	std::vector< std::vector<cv::Point> > &getContours(){
		return contours;
	}

	std::vector<cv::Vec4i> &getHierarchy(){
		return hierarchy;
	}

	cv::Size getImageSize(){
		return imageSize;
	}

	Contour &operator=(Contour &c){
		contours = c.contours;
		hierarchy = c.hierarchy;
		imageSize = c.imageSize;
	}
	
};
