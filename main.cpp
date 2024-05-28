// Include files for required libraries
#include <stdio.h>

#include "OpenCV/opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c/pi2c.h"
#include <iostream>

using namespace std;

Pi2c arduinoVehicle(7);
Pi2c arduinoServoSensor(5);
Pi2c arduinoInertial(3);

// Detects if traffic light goes green
bool trafficLightGreen();

// Identifies colour to follow
int identifyColourOfLine();

// Follows a colour with given parameters
void followLineColour(int colourNumber);

// Detects what symbol is in front of the camera
int symbolDetection(Mat matrixArray[]);

void getShapes();


void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV

    cout<<"Opening Camera..."<<endl;
}

//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------

// Called to identify the new colour to follow
int identifyColourOfLine()
{
    Mat frame;

    while(frame.empty())
    {
        frame = captureFrame();
        cv::rotate(frame, frame, ROTATE_180);
    }

    int HSVvalues[5][6] = {{0,0,0,179,255,70},        // BLACK
                           {170,88,50,179,255,255},   // RED
                           {78,158,0,138,255,255},  // BLUE
                           {35,68,30,73,255,255},     // GREEN
                           {20,100,100,30,255,255}};  // YELLOW

    // Keeps track of the colour with the most pixels
    int largestNumberOfPixels = 0;

    // Saves which colour and corresponds to the coloursArray
    int colourNumber = 0;
    int i = 0;

    for (i=1; i<(sizeof(HSVvalues))/(4*sizeof(int)); i++)
    {
        Mat frameHSV;       // Convert the frame to HSV and apply the limits

        cvtColor(frame, frameHSV, COLOR_BGR2HSV);

        inRange(frameHSV,
                Scalar(HSVvalues[i][0], HSVvalues[i][1], HSVvalues[i][2]),
                Scalar(HSVvalues[i][3], HSVvalues[i][4], HSVvalues[i][5]),
                frameHSV);

        // Count the number of white pixels for the certain HSV values applied
        int colouredPixels = countNonZero(frameHSV);

        // Keeps track of which colour has the most pixels


        if (colouredPixels > largestNumberOfPixels && colouredPixels > 100)
        {

            largestNumberOfPixels = colouredPixels;
            colourNumber = i;
        }
    }

    cout<<colourNumber<<"\n";

    return colourNumber;
}

//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------

// Follows a coloured line
void followLineColour(int colourNumber)
{

    int HSVvaluesArray[5][6] = {{0,0,0,179,255,80},     // BLACK
                        {160,0,100,179,255,255},   // RED
                        {78,158,100,138,255,255},  // BLUE
                        {35,68,57,73,255,255},     // GREEN
                        {20,100,100,30,255,255}};  // YELLOW


    // Point camera up to read symbol
	char servoCommand[] = "D";
	arduinoServoSensor.i2cWrite(servoCommand, 1);

	int key = cv::waitKey(1000); // 3000

    cv::startWindowThread();
    cv::namedWindow("HSV Tester", WINDOW_AUTOSIZE);   // Create a GUI window called HSV Tester


    // Boolean variable to exit loop if it spots an image
    bool followLine = true;

    while(followLine)    // Main loop to perform image processing
    {
        Mat frame;

        while(frame.empty())
        {
            frame = captureFrame();
            cv::rotate(frame, frame, ROTATE_180);
        }

                // Crop the image
        Mat croppedFrame;

        int imageHeight = frame.rows;
        int imageWidth = frame.cols;
        Rect myROI(0,160,320,80);
        croppedFrame = frame(myROI);


        Mat frameCheck;       // Convert the frame to HSV and apply the limits
        cv::cvtColor(croppedFrame, frameCheck, COLOR_BGR2HSV);

        // Check for indicator to read symbol
        cv::inRange(frameCheck, Scalar(155, 120, 0), Scalar(168, 255, 255), frameCheck);

        if (countNonZero(frameCheck)>3000)
        {
            followLine = false;
            //cout<<countNonZero(frameCheck);
        }


        Mat frameHSV;
        cv::cvtColor(croppedFrame, frameHSV, COLOR_BGR2HSV);


        cv::inRange(frameHSV, Scalar(HSVvaluesArray[colourNumber][0], HSVvaluesArray[colourNumber][1], HSVvaluesArray[colourNumber][2]),
                    Scalar(HSVvaluesArray[colourNumber][3], HSVvaluesArray[colourNumber][4], HSVvaluesArray[colourNumber][5]), frameHSV);


        if (countNonZero(frameHSV)<100)
        {
            followLine = false;
        }


        cv::imshow("HSV Tester", frameHSV); //Display the image in the window

        int key = cv::waitKey(10);   // Wait 10ms for a keypress (required to update windows)


        // Find middle pixel of the line
        Moments m = moments(frameHSV);
        Point p(m.m10/m.m00, m.m01/m.m00);
        circle(frameHSV, p, 5, Scalar(128,0,0), -1);
        int middlePixelOfLine = p.x;

        //--------------------------------------------------------------------------------------------

        int middlePixelOfCamera = frameHSV.cols/2;

        // if the middle pixel of the lin is in the middle of the camera feed then the car should go
        // forward, if the line is to the left the middle pixel will be a greater value so the differnce
        // would be negative, the greater the number the more the vehicle should turn it will have a
        // range of 0-160, this can be mapped to 0-100 as this is what the motors expect for speed..
        // If the number is negative the direction is Left, and positive right.

        int speedAndDirection = middlePixelOfLine- middlePixelOfCamera;

        char command[4];

        char direction;

        if (speedAndDirection<5 && speedAndDirection>-5)
        {
            direction = 'F';
            speedAndDirection = 0;
        }
        else if (speedAndDirection>160)
        {
            speedAndDirection = 0;
        }
        else if (speedAndDirection<0)
        {
            direction = 'o'; //o //p
        }
        else
        {
            direction = 'u'; //u //i
        }


        // Concatenates the command with the speed
        sprintf(command, "%c%03d", direction, (int)((60/160.)*(float)abs(speedAndDirection)+10));

        // Sends commnad to vehicle Arduino
        //arduinoVehicle.i2cWrite(command, 4);
	}


	//closeCV();  // Disable the camera and close any windows
    char command[] = "D000";
	arduinoVehicle.i2cWrite(command, 4);

    //closeCV();  // Disable the camera and close any windows

    // Point camera up to read symbol
	char servoCommand1[] = "U";
	arduinoServoSensor.i2cWrite(servoCommand1, 1);

	key = cv::waitKey(1000); //1000

    return;
}

void inclineMeasurement()
{
    cout<<"HELLO\n";

    char command[] = "00000";
    arduinoInertial.i2cWrite(command, 1);

    int key = cv::waitKey(200);

    char angleOfSlope[5];

    arduinoInertial.i2cRead(angleOfSlope, 5);

    cout<<angleOfSlope<<"\n";
}



//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------


int symbolDetection(Mat matrixArray[])
{
    bool symbolFlag = true;

    //setup();    // Call a setup function to prepare IO and device

    int symbol = 10;

    while(symbolFlag)    // Main loop to perform image processing
    {
        Mat frame; //Create a matrix for the camera image

        while(frame.empty())
        {
            frame = captureFrame(); //Capture still image from camera
            cv::rotate(frame, frame, ROTATE_180);
        }

        if (frame.empty())
        {
            cout<<"No Image";
        }
        else
        {

            //Create a HSV version of the camera image

            Mat frameHSV;
            cvtColor(frame, frameHSV, COLOR_BGR2HSV);
            inRange(frameHSV, Scalar(150, 120, 0), Scalar(168, 255, 255), frameHSV);

            if (frameHSV.empty())
            {
                cout<<"No Image\n";
            }
            else
            {

                std::vector< std::vector<cv::Point> > contours; // Variable for list of contours
                std::vector<Vec4i> hierarchy; // Variable for image topology data
                cv::findContours(frameHSV, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE,Point(0, 0)); // Calculate the contours and store them

                std::vector< std::vector<cv::Point> > approxedcontours(contours.size()); // Array for new contours
                for(int i = 0; i < (int) contours.size(); i++)
                {
                    cv::approxPolyDP(contours[i],approxedcontours[i], 10, true); // Approximate the contour
                }

                if (approxedcontours.size()>0){
                if (approxedcontours[0].size()!=4) //If there are not 4 points in approxedcontours[0], then the symbol has not been found
                {
                    cout<<"No contours\n";
                }

                else
                {

                    //Transform the rectangle formed so the symbol is the same size as the images to compare with

                    Mat transformed = transformPerspective(approxedcontours[0], frameHSV, 320, 240); // Transform the perspective of the four points found in frame into a 320x240 image and return this into transformed.

                    if (!transformed.empty()){

                    //Loop used to compare the symbols to the camera image
                    for (int i=0; i<10; i++)
                    {
                        float match = compareImages(matrixArray[i], transformed); // Compare the pixels of two images and return a percentage match. Each image must have the same resolution and colour depth. Works best with black and white images.

                        if (match>=60)  //If the match is significant, we can say that the image is i
                        {
                            cout<<"The image is "<<i<<"\n";
                            symbol = i;
                            symbolFlag = false;
                            break;
                        }

                }
                }
                }
            }

            cv::imshow("HSV Tester", frameHSV); //Display the image in the window

            int key = cv::waitKey(10);   // Wait 10ms for a keypress (required to update windows)
        }
    }
    }
    return symbol;
}

//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------

bool trafficLightGreen()
{
    bool isGreen = false;

    Mat frame;

    while(frame.empty())
    {
        frame = captureFrame();
        cv::rotate(frame, frame, ROTATE_180);
        //cout<<"1";
    }

    Mat frameHSV;       // Convert the frame to HSV and apply the limits

    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    inRange(frameHSV,Scalar(35, 68, 30), Scalar(73, 255, 255), frameHSV);
    //cout<<"2";

    if (countNonZero(frameHSV)>1000)
    {
        isGreen = true;
        cout<<"Boom";
    }


    return isGreen;
}

//-----------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

void getShapes()
{
    Mat src;

	while(src.empty())
    {
        src = captureFrame();
        cv::rotate(src, src, ROTATE_180);
        //cout<<"1";
    }

	// Convert to grayscale
	cv::Mat gray;
	cv::cvtColor(src, gray, CV_BGR2GRAY);

	// Use Canny instead of threshold to catch squares with gradient shading
	cv::Mat bw;
	cv::Canny(gray, bw, 0, 50, 5);

	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw.clone(), contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;
	cv::Mat dst = src.clone();

	for (int i = 0; i < contours.size(); i++)
	{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects
		if (std::fabs(cv::contourArea(contours[i])) < 20 || !cv::isContourConvex(approx))
			continue;

		if (approx.size() == 3)
		{
			setLabel(dst, "TRI", contours[i]);    // Triangles
		}
		else if (approx.size() >= 4 && approx.size() <= 6)
		{
			// Number of vertices of polygonal curve
			int vtc = approx.size();

			// Get the cosines of all corners
			std::vector<double> cos;
			for (int j = 2; j < vtc+1; j++)
				cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

			// Sort ascending the cosine values
			std::sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
				setLabel(dst, "RECT", contours[i]);
			else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
				setLabel(dst, "PENTA", contours[i]);
			else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
				setLabel(dst, "HEXA", contours[i]);
		}
		else
		{
			// Detect and label circles
			double area = cv::contourArea(contours[i]);
			cv::Rect r = cv::boundingRect(contours[i]);
			int radius = r.width / 2;

			if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
			    std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
				setLabel(dst, "CIR", contours[i]);
		}
	}

	cv::imshow("src", src);
	cv::imshow("dst", dst);
	cv::waitKey(0);
	return;
}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    // Initialise the camera down
    char servoCommand[] = "D";
    arduinoServoSensor.i2cWrite(servoCommand, 1);

    //Import each of the symbol images into their respective matrices

    Mat ShapeCounter = imread("Images/ShapeCounter.PNG");
    Mat Football = imread("Images/Football.PNG");
    Mat BlueShortCut = imread("Images/BlueShortCut.PNG");
    Mat DistanceMeasurement = imread("Images/DistanceMeasurement.PNG");
    Mat FollowBlack = imread("Images/FollowBlack.PNG");
    Mat GreenShortCut = imread("Images/GreenShortCut.PNG");
    Mat InclineMeasurement = imread("Images/InclineMeasurement.PNG");
    Mat RedShortCut = imread("Images/RedShortCut.PNG");
    Mat StopLight = imread("Images/StopLight.PNG");
    Mat YellowShortCut = imread("Images/YellowShortCut.PNG");

    //Create a matrix containing the image matrices

    Mat matrixArray[10] = {ShapeCounter,Football,BlueShortCut,DistanceMeasurement,FollowBlack,GreenShortCut,InclineMeasurement,RedShortCut,StopLight,YellowShortCut};

    //Loop used to assign the HSV filter to each image

    for (int i=0; i<10; i++)
    {
        Mat masterHSV;
        cvtColor(matrixArray[i], masterHSV, COLOR_BGR2HSV);
        inRange(masterHSV, Scalar(30, 255, 255), Scalar(179, 255, 255), masterHSV);
        //inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameHSV);
        matrixArray[i]=masterHSV;
    } //Assign the HSV value back to the respective matrix in the array


    followLineColour(0); // Follow black

    int key;
    int sensorValueInt;
    char sensorValue[3];
    char message[50];

    while(1){
    switch(symbolDetection(matrixArray))
    {
        case 0:
            getShapes();
            break;
        case 1:
            //Football Function
            break;
        case 2:
            key = cv::waitKey(100);
            followLineColour(identifyColourOfLine());
            break;
        case 3:
            arduinoServoSensor.i2cRead(sensorValue, 3);
            sensorValueInt = atoi(sensorValue);

            //cout<<sensorValue<<"\n";

            snprintf(message, sizeof(message), "espeak \" Distance is %d centimeters\"", sensorValueInt);
            system(message);

            followLineColour(0);
            break;
        case 4:
            //Follow black
            break;
        case 5:
            //Coloured line following
            break;
        case 6:
            key = cv::waitKey(1000);
            inclineMeasurement();
            break;
        case 7:
            //Coloured line following
            break;
        case 8:
            while (!trafficLightGreen())
            {
                cout<<"Waiting\n";
            }
            followLineColour(0);
            break;
        case 9:
            //Coloured line following
            break;
        case 10:
            cout<<"No matches";
            break;
    }
    }

}





