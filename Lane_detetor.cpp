#include <iostream>
#include "cv.hpp"

using namespace cv;
using namespace std;

int main()
{
	VideoCapture cap("lane.avi");

	Mat src, gray, blur, sobel, canny, grad_x, grad_y, roi, result, masked, eroded;
	double fps = cap.get(CAP_PROP_FPS);
	float rho, theta, theta_sum, theta_sum2, theta_mean, theta_mean2, a, b, x0, y0;
	float rho_sum, rho_sum2, rho_mean, rho_mean2;
	int cnt = 0, cnt2 = 0;
	theta_sum = 0, theta_sum2 = 0;
	rho_sum = 0, rho_sum2 = 0;
	int delay = 1000 / (int)fps;
	Point p1, p2;
	vector<Vec2f> lines, lines2;

	cap >> src;

	Rect rect(src.size().width / 2 - 220, src.size().height / 2 - 50, 500, 180);
	Rect rect_inner(src.size().width / 2 - 70, src.size().height / 2, 200, 300),
		rect_inner1(src.size().width / 2 - 230, src.size().height / 2-50, 100, 100),
		rect_inner2(src.size().width / 2 + 180, src.size().height / 2 - 70, 100, 100);

	roi = src(rect); //Set ROI

	Mat mask = Mat::zeros(src.size(), src.type()); //Mask to isolate ROI

	cvtColor(mask, mask, CV_BGR2GRAY);
	rectangle(mask, rect, 255, CV_FILLED, 8); //Drawing a rectangular mask 
	rectangle(mask, rect_inner, 0, CV_FILLED, 8); //Rectangular mask for center of the road
	rectangle(mask, rect_inner1, 0, CV_FILLED, 8); //Rectangular mask for lefttop of the road
	rectangle(mask, rect_inner2, 0, CV_FILLED, 8); //Rectangular mask for righttop of the road

	//imshow("hi", mask);

	while (1)
	{
		if (!cap.read(src))
		{
			cout << "Input video error!" << endl;
			break;
		}
		if (src.empty())
		{
			cout << "Input frame is empty!" << endl;
			break;
		}
		cvtColor(src, gray, CV_BGR2GRAY);
		threshold(gray, result, 150, 255, THRESH_BINARY | THRESH_OTSU); //Otsu's thresholding
		
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
		erode(result, eroded, kernel); //Erode the image with the 'kernel' - structuring element
		//imshow("ERODED", eroded);
		Canny(eroded, canny, 50, 180, 5); //Edge detection & localization

		bitwise_and(canny, mask, masked); //Use bitwise_and to mask out the rest outside of ROI

		HoughLines(masked, lines, 1, CV_PI / 180, 50, 0, 0, 130 * (CV_PI/180), 145 * (CV_PI/180)); //Right lane (white-dotted lane)
		HoughLines(masked, lines2, 1, CV_PI / 180, 70, 0, 0, 30 * (CV_PI / 180), 55 * (CV_PI / 180)); //Left lane (yellow lane)

		for (int i = 0; i < lines.size(); i++) //Get the sum of rho and theta for the right lane
		{
			rho_sum += lines[i][0];
			theta_sum += lines[i][1];
			cnt++;
		}
		for (int j = 0; j < lines2.size(); j++) //Get the sum of rho and theta for the left lane
		{
			rho_sum2 += lines2[j][0];
			theta_sum2 += lines2[j][1];
			cnt2++;
		}

		rho_mean = rho_sum / cnt; //Calculate the mean of the rho
		rho_mean2 = rho_sum2 / cnt2;

		theta_mean = theta_sum / cnt; //Calculate the mean of the theta
		theta_mean2 = theta_sum2 / cnt2;

		cout << "theta_right: " << (theta_mean*180) / CV_PI<< endl <<  "theta_left " << (theta_mean2*180) / CV_PI << endl;

		for (int i = 0; i < lines.size(); i++) //Right lane (white-dotted lane)
		{
			rho = lines[i][0];
			theta = lines[i][1];

			a = cos(theta_mean);
			b = sin(theta_mean);
			x0 = a * rho_mean;
			y0 = b * rho_mean;
			p1 = Point(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * a));
			p2 = Point(cvRound(x0 - 10000 * (-b)), cvRound(y0 - 10000 * a));
			if (theta_mean < 0)
				break;
			else
			{
				line(src, p1, p2, Scalar(0, 0, 255), 8, 8);
				cout << (theta_mean * 180) / CV_PI << endl;
			}
		}
		for (int i = 1; i < lines2.size(); i++) //Left lane (yellow lane)
		{
			rho = lines2[i][0];
			theta = lines2[i][1];

			a = cos(theta_mean2);
			b = sin(theta_mean2);
			x0 = a * rho_mean2;
			y0 = b * rho_mean2;
			p1 = Point(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * a));
			p2 = Point(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * a));
			if (theta_mean2 < 0)
				break;
			else
			{
				line(src, p1, p2, Scalar(0, 0, 255), 8, 8);
				cout << (theta_mean2 * 180) / CV_PI << endl;
			}
		}

		//imshow("CANNY_EDGE", canny);
		imshow("ORIGINAL", src);
		//imshow("BLURRED", blur);
		//imshow("ROI_BINARY", masked);
		imshow("ROI", roi);

		waitKey(delay/10);
	}
}