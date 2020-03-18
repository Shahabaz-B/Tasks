#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <conio.h>
#include <string>
//#include "..\testerProject\vcpkg\installed\x86-windows\include\json\json.h"

cv::RNG rng(12345);

void DetectCircle(cv::Mat image, int thresh1, int thresh2, int minRad, int maxRad, int minDist, bool customParam = false);

int main(int argc, char** argv)
{

	cv::Mat src, srcGray, draw;

    std::cout << "Enter image file path\n";
	std::string imagePath;
	std::cin >> imagePath;
	/// Read the image
	src = cv::imread(imagePath, cv::IMREAD_COLOR);

	if (!src.data)
	{
		return -1;
	}

	DetectCircle(src, 100, 35, 70, 240, 110);
	
	return 0;
}

void DetectCircle(cv::Mat image, int thresh1, int thresh2, int minRad, int maxRad, int minDist, bool customParam)
{
	cv::Mat srcGray, draw;
	/// Reduce the noise so we avoid false circle detection
	cv::cvtColor(image, srcGray, cv::COLOR_BGR2GRAY);
	cv::medianBlur(srcGray, srcGray, 5);

	std::vector<cv::Vec3f> circles;

	if (!customParam)
	{
		/// Apply the Hough Transform to find the circles
		HoughCircles(srcGray, circles, CV_HOUGH_GRADIENT, 1, minDist, thresh1, thresh2, minRad, maxRad);

		draw = image.clone();

		//Json::Value root;
		//Json::StreamWriterBuilder builder;
		//const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
		//std::ofstream o("pretty.json");

		/// Draw the circles detected
		for (size_t i = 0; i < circles.size(); i++)
		{
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// circle center
			std::cout << "center : " << center << " radius : " << radius << std::endl;
		
			//root["center_point"] = (center.x, center.y);
			//root["size"] = radius * 2;
			//writer->write(root, &o);
			
			cv::circle(draw, center, 3, color, -1, 8, 0);
			cv::circle(draw, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
		}

		/// Show your results
		cv::namedWindow("Hough Circle Transform Demo", cv::WINDOW_GUI_NORMAL);
		cv::imshow("Hough Circle Transform Demo", draw);

		cv::waitKey();
	}
	else
	{
		std::string parameterWindow = "Parameter Window";
		cv::namedWindow(parameterWindow, cv::WINDOW_AUTOSIZE); // Create Window

		cv::createTrackbar("Parameter1", parameterWindow, &thresh1, 1000);
		cv::createTrackbar("Parameter2", parameterWindow, &thresh2, 1000);
		cv::createTrackbar("MinRad", parameterWindow, &minRad, 1000);
		cv::createTrackbar("MaxRad", parameterWindow, &maxRad, 1000);
		cv::createTrackbar("MinDist", parameterWindow, &minDist, 1000);

		int keyPress = 0;

		std::cout << "Press ESC to exit\n";

		while (1)
		{
			/// Apply the Hough Transform to find the circles
			HoughCircles(srcGray, circles, CV_HOUGH_GRADIENT, 1, minDist, thresh1, thresh2, minRad, maxRad);

			draw = image.clone();

			/// Draw the circles detected
			for (size_t i = 0; i < circles.size(); i++)
			{
				cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				int radius = cvRound(circles[i][2]);
				// circle center
				cv::circle(draw, center, 3, color, -1, 8, 0);
				// circle outline
				cv::circle(draw, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
			}

			/// Show your results
			cv::namedWindow("Hough Circle Transform Demo", cv::WINDOW_GUI_NORMAL);
			cv::imshow("Hough Circle Transform Demo", draw);

			_kbhit() ? keyPress = _getch() : cv::waitKey(10);

			if (keyPress == 27)
				break;
		}
	}

}
