

#define __CHAMFER_DEBUG_MODE___
#define __CHAMFER_LOW_MEMORY___

#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <cstdio>

#include "Color.h"
#include "Contour.h"
#include "Detector.h"
#include "ChamferMatcher.h"


void matchPuzzle(cv::Mat &img, cv::Mat &tpl){
	cv::Mat image = img.clone();
	cv::Mat iedge;
	cv::Mat templ = tpl.clone();

	ending::DEBUG_img = image.clone();

	colorEdgeDetection(image, iedge, true);

	ending::RChamferMatcher cmatcher(1, 20, 1.0, 3, 3, 3, 0.8, 1.2, 0.5, 20, 5);

	cmatcher.addMatcher(templ);

	std::vector<ending::Matcher::MatchPoints> matchpoints;

	cmatcher.multimatching(iedge, matchpoints);

	for (int i = 0; i < matchpoints.size(); i++){
		ending::Matcher::MatchPoint &mp = matchpoints[i][0];

		cv::Point center = mp.getBoundingBoxCenter();  //the center of puzzle
		double rotateAngle = mp.getAngle();  //radian, clockwise 0~360

		cv::Size matchSize = mp.getBoundingBoxSize();
		

		int maxRadius_ = (matchSize.width > matchSize.height ? matchSize.width : matchSize.height);
		maxRadius_ = (maxRadius_ + 1) / 2;
		cv::circle(image, center, maxRadius_, cv::Scalar(0, 255, 0));
		cv::Point lineEnd(center.x - (int)(sin(rotateAngle)*maxRadius_ + 0.5), center.y - (int)(cos(rotateAngle)*maxRadius_ + 0.5));
		cv::line(image, center, lineEnd, cv::Scalar(0, 255, 0));
	}

	cv::imshow("result", image);
	cv::imshow("debug", ending::DEBUG_img);
}


void showdevice(){
	openni::Array<openni::DeviceInfo> aDeviceList;
	openni::OpenNI::enumerateDevices(&aDeviceList);

	std::cout << "Depth Camera Count: " << aDeviceList.getSize();

	for (int i = 0; i < aDeviceList.getSize(); i++){
		std::cout << "Number: " << i << std::endl;
		const openni::DeviceInfo &rDevInfo = aDeviceList[i];
		std::cout << "Name: " << rDevInfo.getName() << std::endl;
		std::cout << "ID: " << rDevInfo.getUsbProductId() << std::endl;
		std::cout << "Vender¡G " << rDevInfo.getVendor() << std::endl;
		std::cout << "VenderID: " << rDevInfo.getUsbVendorId() << std::endl;
		std::cout << "URI: " << rDevInfo.getUri() << std::endl;
	}
}


int main(int argc, char *argv[]){

	openni::OpenNI::initialize();

	showdevice();

	openni::Device camera;

	if (camera.open(openni::ANY_DEVICE) != openni::STATUS_OK){
		std::cout << "::Error:: Can't open device..." << std::endl;
		return -1;
	}
	
	openni::VideoStream DepthStream;
	openni::VideoStream ColorStream;

	if (DepthStream.create(camera, openni::SENSOR_DEPTH) != openni::STATUS_OK){
		std::cout << "Can't create depth stream..." << std::endl;
	}
	else{
		openni::VideoMode mMode;
		mMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		DepthStream.setVideoMode(mMode);

		if (DepthStream.start() != openni::STATUS_OK){
			std::cout << "Can't start depth stream..." << std::endl;
			DepthStream.destroy();
		}
	}

	if (ColorStream.create(camera, openni::SENSOR_COLOR) != openni::STATUS_OK){
		std::cout << "Can't create color sensor..." << std::endl;
	}
	else{
		openni::VideoMode mMode;
		mMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
		ColorStream.setVideoMode(mMode);

		if (ColorStream.start() != openni::STATUS_OK){
			std::cout << "Can't start color stream..." << std::endl;
			ColorStream.destroy();
		}
	}

	if (!ColorStream.isValid() || !DepthStream.isValid()){
		std::cout << "Some stream is invalid..." << std::endl;
		openni::OpenNI::shutdown();
		return -1;
	}

	if (camera.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)){
		camera.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}


	cv::namedWindow("Image");

	int MaxDepth = DepthStream.getMaxPixelValue();
	int MinDepth = DepthStream.getMinPixelValue();
	std::cout << "Min Depth: " << MinDepth << std::endl;
	std::cout << "Max Depth: " << MaxDepth << std::endl;

	openni::VideoFrameRef DepthFrame;
	openni::VideoFrameRef ColorFrame;

	std::string prefix = "edge_x04/";
	std::string templfile = "A.png";

	if (argc > 1){
		templfile = std::string(argv[1]);
	}

	templfile = prefix + templfile;

	cv::Mat templ = cv::imread(templfile, CV_LOAD_IMAGE_GRAYSCALE);

	while (1){
		cv::Mat tdepth, mdepth;
		cv::Mat mcolor;
		cv::Mat image;

		if (DepthStream.readFrame(&DepthFrame) == openni::STATUS_OK){
			tdepth = cv::Mat(DepthFrame.getHeight(), DepthFrame.getWidth(), CV_16UC1, (void*)DepthFrame.getData());
			tdepth.convertTo(mdepth, CV_8UC1, 255.0 / MaxDepth);
			cv::resize(mdepth, mdepth, cv::Size(640, 480));
			cv::cvtColor(mdepth, mdepth, CV_GRAY2BGR);
		}

		if (ColorStream.readFrame(&ColorFrame) == openni::STATUS_OK){
			cv::Mat tcolor(ColorFrame.getHeight(), ColorFrame.getWidth(), CV_8UC3, (void*)ColorFrame.getData());
			cv::cvtColor(tcolor, mcolor, CV_RGB2BGR);
			cv::resize(mcolor, mcolor, cv::Size(640, 480));
		}


		char key = cv::waitKey(10);

		if (key == ' '){
			matchPuzzle(mcolor, templ);
		}

		cv::hconcat(mdepth, mcolor, image);
		cv::imshow("Image", image);
	}


	DepthStream.destroy();
	ColorStream.destroy();
	camera.close();
	openni::OpenNI::shutdown();

	return 0;
}