#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include <iostream>
#include <fstream>
//#include "mavlink_control.h"

using namespace sl;
using namespace std;

#define DIS_THRESH 6	//Threshold for the depth values. Represents 6 feet
#define PER_THRESH 20	//Threshold for the percentage of pixels in a section that are above the disThresh
#define VELO 10	//This is velocity when the UAV must move into another section

void printImageValues(sl::Mat&);	//Prints the values of each pixel to a text file (This is used for testing)
cv::Mat slMat2cvMat(sl::Mat& input);	//Converts a sl::Mat to a cv::Mat
void partitionCalc(sl::Mat&, float*);	//Partition the disparity map and calculate all of the percentage of pixels higher than the threshold in each section
void countPixels(sl::Mat&, const int&, const int&, const int&, const int&, int&, int&);	//Count the number of pixels in a given section that are higher than the threshold
float getPercentage(const int&, const int&);	//Returns the percentage of pixels higher than the threshold in the given section
int selectSection(const float*);	//Selects the section with the lowest percentage that is lower than the percentage threshold
//void manuever(const int&, Autopilot_Interface&); //Moves the UAV based on the section selected
void manuever(const int&);	//Moves the UAV based on the section selected

int main()
{
	// Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;	//Can be set to PERFORMANCE, MEDIUM, OR QUALITY
    init_params.coordinate_units = sl::UNIT_FOOT;	//Measurements are in feet
	//init_params.camera_fps = 60;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        return 1;

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_FILL; // Use STANDARD sensing mode for obstacle detection (the other option is FILL)

    // Create sl and cv Mat to get ZED left image and depth image
    // Best way of sharing sl::Mat and cv::Mat :
    // Create a sl::Mat and then construct a cv::Mat using the ptr to sl::Mat data.
    Resolution image_size = zed.getResolution();
				//This is used to see the right and left image and is only for testing
				/*sl::Mat image_zed_left(image_size,sl::MAT_TYPE_8U_C4); // Create a sl::Mat to handle Left image
				sl::Mat image_zed_right(image_size,sl::MAT_TYPE_8U_C4); // Create a sl::Mat to handle right image
				cv::Mat image_ocv_left = slMat2cvMat(image_zed_left);*/
	sl::Mat depth_image_zed(image_size, MAT_TYPE_8U_C4);
	sl::Mat depth_image_zed_print(image_size, MAT_TYPE_8U_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed_print);
				//cv::Mat image_ocv_right = slMat2cvMat(image_zed_right);


    // Create OpenCV images to display (lower resolution to fit the screen)
    cv::Size displaySize(1280, 720);
				//cv::Mat image_ocv_left_display(displaySize, CV_8UC4);
				//cv::Mat image_ocv_right_display(displaySize, CV_8UC4);
    cv::Mat depth_image_ocv_display(displaySize, CV_8UC4);

	// Jetson only. Execute the calling thread on 2nd core
    Camera::sticktoCPUCore(2);

														/*
														#ifdef __APPLE__
															char *uart_name = (char*)"/dev/tty.usbmodem1";
														#else
															char *uart_name = (char*)"/dev/ttyUSB0";
														#endif

														int baudrate = 57600;

														Serial_Port serial_port(uart_name, baudrate);

														Autopilot_Interface autopilot_interface(&serial_port);

														serial_port_quit         = &serial_port;
														autopilot_interface_quit = &autopilot_interface;
														signal(SIGINT,quit_handler);

														serial_port.start();
														autopilot_interface.start();

														
														*/

    // Loop until 'q' is pressed
    char key = ' ';
	while (key != 'x')	//Used to exit the program
	{
		//Used to get one 
		while (key != 'q') {

		    // Grab and display image and depth 
		    if (zed.grab(runtime_parameters) == SUCCESS) {

							//zed.retrieveImage(image_zed_left, VIEW_LEFT); // Retrieve the left image
							//zed.retrieveImage(image_zed_right, VIEW_RIGHT); // Retrieve the right image
		        zed.retrieveImage(depth_image_zed_print, VIEW_DEPTH); //Retrieve the depth view (image)
				zed.retrieveMeasure(depth_image_zed, MEASURE_DEPTH);	//Retrieve the depth for the image

		        // Resize and display with OpenCV
							/*cv::resize(image_ocv_left, image_ocv_left_display, displaySize);
							imshow("Image Left", image_ocv_left_display);
							cv::resize(image_ocv_right, image_ocv_right_display, displaySize);
							imshow("Image right", image_ocv_right_display);*/

		        cv::resize(depth_image_ocv, depth_image_ocv_display, displaySize);	//Used to print the disparity map

				float sectionValues[9];	//Holds the percentage of pixels that are above the threshold in each section of the disparity image
				partitionCalc(depth_image_zed, sectionValues);	//Partitions the image and calculates the percent of pixels in each section
				int section = selectSection(sectionValues);		//The section that is selected
				cout << "The selected section is: " << section << endl;
				manuever(section);	//Moves the UAV in a certain direction
				//manuever(section, autopilot_interface);	//Moves the UAV in a certain direction

				//Shows the lines that partition the image
				cv::line(depth_image_ocv_display,
								cv::Point(depth_image_ocv_display.cols / 3, 0),
								cv::Point(depth_image_ocv_display.cols / 3, depth_image_ocv_display.rows),
								0,
								1);
				cv::line(depth_image_ocv_display,
								cv::Point(depth_image_ocv_display.cols / 3 * 2, 0),
								cv::Point(depth_image_ocv_display.cols / 3 * 2, depth_image_ocv_display.rows),
								0,
								1);
				cv::line(depth_image_ocv_display,
								cv::Point(0, depth_image_ocv_display.rows / 3),
								cv::Point(depth_image_ocv_display.cols, depth_image_ocv_display.rows / 3),
								0,
								1);
				cv::line(depth_image_ocv_display,
								cv::Point(0, depth_image_ocv_display.rows / 3 * 2),
								cv::Point(depth_image_ocv_display.cols ,depth_image_ocv_display.rows / 3 * 2),
								0,
								1);
				//Shows the disparity image
		        imshow("Disparity Map", depth_image_ocv_display);
		        key = cv::waitKey(10);
		    }
		}
		//Print the values for the image to a txt file
		printImageValues(depth_image_zed);
		imshow("Depth Selected", depth_image_ocv_display);
		cv::waitKey();
		cin >> key;
	}

														//autopilot_interface.stop();
														//serial_port.stop();
	zed.close();
	return 0;
}

//Converts the sl::Mat to the cv::Mat (This is just to be able to see the disparity map durring testing)
cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
		case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
		case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
		case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
		case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
		case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
		case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
		case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
		case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
		default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

//Print the image to a txt file for testing
void printImageValues(sl::Mat& depthMap)
{
	ofstream myFile;
	myFile.open("imageValues.txt");
	myFile << "[ ";
	for(int y = 0; y < depthMap.getHeight(); y++)
	{
		myFile << "[ ";
		for(int x = 0; x < depthMap.getWidth(); x++)
		{
			float depth;
			depthMap.getValue(x, y, &depth);
			myFile << depth << ", ";
		}
		myFile << "]\n";
	}
	myFile << " ]";
	cout << "Image wrote to file.";
	myFile.close();
}

//Partition the disparity map and calculate all of the percentage of pixels higher than the threshold in each section
void partitionCalc(sl::Mat& disparityMap, float *sectionValues)
{
	//An array of values that seperates the disparity image across its width
	int startWidth[] = { 0, disparityMap.getWidth() / 3, disparityMap.getWidth() / 3 * 2 };
	//An array of values that seperates the disparity image across its height
	int startHeight[] = { 0, disparityMap.getHeight() / 3, disparityMap.getHeight() / 3 * 2 };

	int numAbove = 0;	//Holds the number of pixels that are above the given threshold
	int totalPix = 0;	//Holds the total number of pixels in the section
	
	//Loops through each section
	for (int i = 0; i < 9; i++)
	{
		int row = i / 3;	//Used to know the row of the current section
		int col = i % 3;	//Used to know the column of the current section
		int startW = startWidth[col];	//Used to know the starting point for the width of the current section
		int startH = startHeight[row];	//Used to know the starting point for the height of the current section
		int endW;	//Used to know where the width of the current section ends
		int endH;	//Used to know where the height of the current section ends
		if (col == 2)
			endW = disparityMap.getWidth();	//The width ends at the overall width of the image
		else
			endW = startWidth[col + 1];	//The width ends at the point the next section starts
		if (row == 2)
			endH = disparityMap.getHeight();	//The height ends at the overall height of the image
		else
			endH = startHeight[row + 1];	//The height ends at the point the next section starts

		numAbove = 0;	//Resets the value to 0;
		totalPix = 0;	//Resets the value to 0;

		//Calculates the number of pixels above the threshold for the current section
		countPixels(disparityMap, startW, startH, endW, endH, numAbove, totalPix);
		cout << "Section " << i << ": " << numAbove << " / " << totalPix << endl;
		//Calculate the percentage
		sectionValues[i] = getPercentage(numAbove, totalPix);
		cout << "Section " << i << ": " << sectionValues[i] << "%" << endl;
	}
	cout << "\n\n\n";
	//cout << "Partitioning complete";
}

//Count the number of pixels in a given section that are higher than the threshold
void countPixels(sl::Mat& disparityMap, const int& startW, const int& startH, const int& endW, const int& endH, int& numAbove, int& totalPix)
{
	//Loop through all of the pixels in the given section
	for(int y = startH; y < endH; y++)
	{
		for(int x = startW; x < endW; x++)
		{
			totalPix++;	//Increase the total number of pixels by 1
			float depth;	//Used to hold the depth at the current pixel
			disparityMap.getValue(x, y, &depth);	//Gets the depth at the current pixel
			//Checks if the distance at the current pixel is closer than the threshold distance
			if(depth <= DIS_THRESH || depth == NAN || depth == TOO_CLOSE)
			{
				numAbove++;	//Increases the number of pixels greater than the threshold by 1
			}
		}
	}
}

//Returns the percentage of pixels that are above the threshold
float getPercentage(const int& numAbove, const int& totalPix)
{
	return ((float)(numAbove) / totalPix) * 100;
}


//Selects the section that has the smallest percentage
int selectSection(const float *sectionValues)
{
	//Start with the center section so the UAV flies to the destination
	float minPercent = sectionValues[4];
	int minPos = 4;
	//Get the section number that has the lowest percentage
	for (int i = 0; i < 9; i++)
		if (sectionValues[i] < minPercent)
		{
			minPercent = sectionValues[i];
			minPos = i;
		}
	//Check the minPercent with the percentage threshold
	if (minPercent < PER_THRESH)
		return minPos;
	return -1;	//The section with the smallest percentage has a percentage higher than the percentage threshold
}

//Move the UAV in respect to the section that was selected.
//void manuever(const int& section, Autopilot_Interface api)
void manuever(const int& section)
{
														/*api.enable_offboard_control();
														usleep(100); // give some time to let it sink in
														// initialize command data strtuctures
														mavlink_set_position_target_local_ned_t sp;
														mavlink_set_position_target_local_ned_t ip = api.initial_position;*/

	switch (section)
	{
		case 0:
			//Move left and increase altitude
			cout << "The UAV is moving up and left.";
														//set_velocity(0, VELO * -1, VELO * -1, sp);
			break;
		case 1:
			//Increase altitude
			cout << "The UAV is moving up.";
														//set_velocity(0, 0, VELO * -1, sp);
			break;
		case 2:
			//Move right and increase altitude
			cout << "The UAV is moving up and right.";
														//set_velocity(0, VELO, VELO * -1, sp);
			break;
		case 3:
			//Move left
			cout << "The UAV is moving left.";
														//set_velocity(0, VELO * -1, 0, sp);
			break;
		case 4:
			//Continue to destination
			cout << "The UAV is flying straight.";
			break;
		case 5:
			//Move right
			cout << "The UAV is moving right.";
														//set_velocity(0, VELO, 0, sp);
			break;
		case 6:
			//Move left and decrease altitude
			cout << "The UAV is moving down and left.";
														//set_velocity(0, VELO * -1, VELO, sp);
			break;
		case 7:
			//Decrease altitude
			cout << "The UAV is moving down.";
														//set_velocity(0, 0, VELO, sp);
			break;
		case 8:
			//Move right and decrease altitude
			cout << "The UAV is moving down and right.";
														//set_velocity(0, VELO, VELO, sp);
			break;
		default:
			//Turn in place or hover in place
			cout << "The UAV is turning in place.";
														//set_velocity(0, 0, 0, sp);
														
	}
	cout << endl;
}











