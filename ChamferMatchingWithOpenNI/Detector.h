#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include "Color.h"
#include "Contour.h"

void image_or(cv::Mat &input1, cv::Mat &input2, cv::Mat &output){
	cv::Size p = input1.size();
	output = cv::Mat::zeros(p, CV_8UC1);
	for (int i = 0; i < p.height; i++){
		for (int j = 0; j < p.width; j++){
			if (input1.at<uchar>(i, j) >200 || input2.at<uchar>(i, j) > 200){
				output.at<uchar>(i, j) = 255;
			}
		}
	}
}

void image_and(cv::Mat &input1, cv::Mat &input2, cv::Mat &output){
	cv::Size p = input1.size();
	output = cv::Mat::zeros(p, CV_8UC1);
	for (int i = 0; i < p.height; i++){
		for (int j = 0; j < p.width; j++){
			if (input1.at<uchar>(i, j) >200 && input2.at<uchar>(i, j) > 200){
				output.at<uchar>(i, j) = 255;
			}
		}
	}
}

void image_not(cv::Mat &input, cv::Mat &output){
	cv::Size p = input.size();
	output = input.clone();
	for (int i = 0; i < p.height; i++){
		for (int j = 0; j < p.width; j++){
			output.at<uchar>(i, j) = (input.at<uchar>(i, j) == 0 ? 255 : 0);
		}
	}
}


void changeMaskColor(cv::Mat &input, cv::Mat &output, cv::Mat mask, cv::Vec3b bgr){
	output = input.clone();

	for (int i = 0; i < input.rows; i++){
		for (int j = 0; j < input.cols; j++){
			if (mask.at<uchar>(i, j) == 255){
				cv::Vec3b c = input.at<cv::Vec3b>(i, j);
				output.at<cv::Vec3b>(i, j) = blending(c, bgr, 1);
			}
		}
	}
}

void wearMask(cv::Mat &input, cv::Mat &output, cv::Mat mask){
	output = input.clone();
	for (int i = 0; i < input.rows; i++){
		for (int j = 0; j < input.cols; j++){
			if (mask.at<uchar>(i, j) == 0){
				output.at<cv::Vec3b>(i, j) = 0;
			}
		}
	}
}

void addToMask(cv::Mat &input, cv::Mat &output){
	for (int i = 0; i < input.rows; i++){
		for (int j = 0; j < input.cols; j++){
			if (input.at<uchar>(i, j) > 200){
				output.at<uchar>(i, j) = 255;
			}
		}
	}
}

void subToMask(cv::Mat &input, cv::Mat &output){
	for (int i = 0; i < input.rows; i++){
		for (int j = 0; j < input.cols; j++){
			if (input.at<uchar>(i, j) > 200){
				output.at<uchar>(i, j) = 0;
			}
		}
	}
}

void resetMask(cv::Mat &input){
	input = cv::Mat::zeros(input.size(), CV_8UC1);
}

void templateEdgeDetection(cv::Mat &input, cv::Mat &output){
	cv::Mat outputImage = input.clone();
	if (input.channels() == 3){
		cv::cvtColor(input, outputImage, CV_BGR2GRAY);
	}
	cv::Canny(outputImage, outputImage, 50, 100, 3);
	output = outputImage;
}

void colorMask(cv::Mat &input, cv::Mat &output, HSVcolor lower, HSVcolor upper){

	cv::Mat outputImage = cv::Mat::zeros(input.size(), CV_8UC3);

	for (int i = 0; i < input.rows; i++){
		for (int j = 0; j < input.cols; j++){
			cv::Vec3b color = input.at<cv::Vec3b>(i, j);
			HSVcolor hsv = BGR2HSV(color);
			if (lower[0] <= upper[0]){
				if (hsv >= lower && hsv <= upper)outputImage.at<cv::Vec3b>(i, j) = color;
			}
			else{
				if (hsv >= lower || hsv <= upper)outputImage.at<cv::Vec3b>(i, j) = color;
			}

		}
	}

	output = outputImage.clone();
}

//input=CV_8UC3, output=CV_8UC1, lower=lower bound, upper=upper bound
void colorDetection(cv::Mat &input, cv::Mat &output, HSVcolor lower, HSVcolor upper){

	cv::Mat outputImage = cv::Mat::zeros(input.size(), CV_8UC1);

	for (int i = 0; i < input.rows; i++){
		for (int j = 0; j < input.cols; j++){
			cv::Vec3b color = input.at<cv::Vec3b>(i, j);
			HSVcolor hsv = BGR2HSV(color);
			if (lower[0] <= upper[0]){
				if (hsv >= lower && hsv <= upper)outputImage.at<uchar>(i, j) = 255;
				else outputImage.at<uchar>(i, j) = 0;
			}
			else{
				if (hsv >= lower || hsv <= upper)outputImage.at<uchar>(i, j) = 255;
				else outputImage.at<uchar>(i, j) = 0;
			}

		}
	}

	output = outputImage.clone();
}

//input=CV_8UC1 image, output=CV_8UC1 image, b=blur(true/false)
void edgeDetection(cv::Mat &input, cv::Mat &output, bool b = true){
	cv::Mat outputImage = input;
	if (outputImage.channels() == 3){
		cv::cvtColor(outputImage, outputImage, CV_BGR2GRAY);
	}

	cv::Mat blurImage;
	if(b)cv::bilateralFilter(outputImage, blurImage, 5, 150, 150);
	else blurImage = outputImage;

	cv::Canny(blurImage, outputImage, 50, 100, 3);

	//cv::erode(outputImage, outputImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)));
	//cv::dilate(outputImage, outputImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)));

	cv::dilate(outputImage, outputImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
	cv::erode(outputImage, outputImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

	//cv::findContours(outputImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	output = outputImage;
}

//input=CV_8UC3 Image, output=CV_8UC1
void colorEdgeDetection(cv::Mat &input, cv::Mat &output, bool b = true){
	
	//create output image
	cv::Mat inputImage = input.clone();
	cv::Mat outputImage;
	if (input.channels() == 1){
		cv::cvtColor(inputImage, inputImage, CV_GRAY2BGR);
	}

	std::vector<cv::Mat> colorchannels;
	cv::split(inputImage, colorchannels);


	edgeDetection(colorchannels[0], colorchannels[0], b);
	edgeDetection(colorchannels[1], colorchannels[1], b);
	edgeDetection(colorchannels[2], colorchannels[2], b);


	cv::bitwise_or(colorchannels[0], colorchannels[1], outputImage);
	cv::bitwise_or(colorchannels[2], outputImage, outputImage);


	output = outputImage;
}

void rotateWithBoundingBox(cv::Mat &input, cv::Mat &output, double angle){

	// get rotation matrix for rotating the image around its center
	cv::Point2f center(input.cols / 2.0f, input.rows / 2.0f);
	cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
	// determine bounding rectangle
	cv::Rect bbox = cv::RotatedRect(center, input.size(), (float)angle).boundingRect();
	// adjust transformation matrix
	rot.at<double>(0, 2) += bbox.width / 2.0 - center.x;
	rot.at<double>(1, 2) += bbox.height / 2.0 - center.y;

	cv::warpAffine(input, output, rot, bbox.size());
}

void rotate(cv::Mat &input, cv::Mat &output, double angle){

	// get rotation matrix for rotating the image around its center
	cv::Point2f center(input.cols / 2.0f, input.rows / 2.0f);
	cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);

	cv::warpAffine(input, output, rot, input.size());
}
