#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <common/mavlink.h>

#include "autopilot_interface.h"
#include "serial_port.h"

//using std::string;
using namespace sl;
using namespace std;

#define DIS_THRESH 6		//Threshold for the depth values. Represents 6 feet
#define PER_THRESH 15		//Threshold for the percentage of pixels in a section that are below the DIS_THRESH
#define HALF_WIDTH 314	//This is half of the width of each rectangle (in pixels)
#define HALF_HEIGHT 126	//This is half of the height of each rectangle (in pixels)
#define NUM_RECT 17		//This is the number of rectangles in each row and column
#define TOTAL_RECT (NUM_RECT * NUM_RECT)	//This is the total number of rectangles for the entire image
#define WIDTH 1280			//This is the overall width of the image (in pixels)
#define HEIGHT 720			//This is the overall height of the image (in pixels)
#define CENTER_WIDTH (WIDTH / 2)	//This is the width of the center point of the screen
#define CENTER_HEIGHT (HEIGHT / 2)	//This is the height of the center point of the screen
#define TOTAL_PIXELS (HALF_WIDTH * HALF_HEIGHT * 4)	//This is the total number of pixels in a rectangle
#define VELO 2.5
#define PI 3.14159265358979323

Serial_Port *serial_port_quit;
Autopilot_Interface *autopilot_interface_quit;

void printImageValues(sl::Mat&);	//Prints the values of each pixel to a text file (This is used for testing)
cv::Mat slMat2cvMat(sl::Mat& input);	//Converts a sl::Mat to a cv::Mat
void partition(int*, int*);		//Creates the center points for the partitions
void fillArray(int*, const int&, const int&);	//Recursive function to fill in the array for the center points
float getPercentage(const int&, const int&);	//Returns the percentage of pixels higher than the threshold in the given section
void countPixels(sl::Mat&, const int*, const int*, float*);	//Iterate through all of the pixels and increment the appropriate counters
void checkCols(bool*, const int*, const int&);	//Check which columns of rectangles the current pixel will fall into
void checkRows(bool*, const int*, const int&);	//Check which rows of rectangles the current pixel will fall into
void updateCounters(const bool*, const bool*, int*);	//Increments the appropriate counters
void calcPercentages(float*, const int*);	//Calculates all of the percentages for each rectangle
int selectSection(const float*);	//Selects the section with the lowest percentage that is lower than the percentage threshold
void manuever(const int&, Autopilot_Interface, const int&, const int&);	//Moves the UAV based on the section selected
void clearPositions(int*, int&);
int closest(const int*, const int&);
float distanceCalc(const int&);
void getCenter(int&, int&, const int&, const int*, const int*);	//Gets the center of the selected rectangle. This is used to print the box the UAV will fly to
void quit_handler( int sig );

int main()
{
	// Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;	//Can be set to PERFORMANCE, MEDIUM, OR QUALITY
    init_params.coordinate_units = sl::UNIT_FOOT;	//Measurements are in feet

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
	sl::Mat depth_image_zed(image_size, MAT_TYPE_8U_C4);
	sl::Mat depth_image_zed_print(image_size, MAT_TYPE_8U_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed_print);

    // Create OpenCV images to display (lower resolution to fit the screen)
    cv::Size displaySize(1280, 720);
    cv::Mat depth_image_ocv_display(displaySize, CV_8UC4);

	// Jetson only. Execute the calling thread on 2nd core
    Camera::sticktoCPUCore(2);

	char *uart_name = (char*)"/dev/ttyUSB0";

	int baudrate = 57600;

	Serial_Port serial_port(uart_name, baudrate);

	Autopilot_Interface autopilot_interface(&serial_port);

	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	serial_port.start();
	autopilot_interface.start();

	float sectionValues[TOTAL_RECT];	//Holds the percentage of pixels that are above the threshold in each section of the disparity image
	int widthSections[NUM_RECT];		//Holds the width value of the center points of all of the rectangles
	int heightSections[NUM_RECT];		//Holds the height value of the center points of all of the rectangles
	partition(widthSections, heightSections);	//Creates the center points for the partitions

	//Initializes the rectangle that will be printed to the center of the image
	int centerW = WIDTH / 2;
	int centerH = HEIGHT / 2;

	// Loop until 'q' is pressed
    char key = ' ';
	while (key != 'x')	//Used to exit the program
	{
		//Used to get one disparity image
		while (key != 'q')
		{
			//clock_t start = clock();
			//double duration = 0;
			//int counter = 0;
			//while(duration < 1)
			//{
				// Grab and display image and depth 
				if (zed.grab(runtime_parameters) == SUCCESS)
				{
					zed.retrieveImage(depth_image_zed_print, VIEW_DEPTH);	//Retrieve the depth view (image)
					zed.retrieveMeasure(depth_image_zed, MEASURE_DEPTH);	//Retrieve the depth for the image

					// Resize and display with OpenCV
					cv::resize(depth_image_ocv, depth_image_ocv_display, displaySize);	//Used to print the disparity map

					countPixels(depth_image_zed, widthSections, heightSections, sectionValues);	//Iterates through each pixel and increments the appropriate counters
					int section = selectSection(sectionValues);		//The section that is selected
					//cout << "The selected section is: " << section << endl;

					//Gets the center of the selected rectangle
					getCenter(centerW, centerH, section, widthSections, heightSections);
					//cout << "Center: " << centerW << ", " << centerH << endl;
					manuever(section, autopilot_interface, centerW, centerH);	//Moves the UAV in a certain direction

					//Initializes the rectangle that will be printed to the center of the image
					//int centerW = WIDTH / 2;
					//int centerH = HEIGHT / 2;

					//Prints the rectanlge representing the selected section if there is one
					if(centerW != 0 && centerH != 0)
					{
						cv::rectangle(depth_image_ocv_display,
										cv::Point(centerW - HALF_WIDTH, centerH - HALF_HEIGHT),
										cv::Point(centerW + HALF_WIDTH, centerH + HALF_HEIGHT),
										cv::Scalar(0, 255, 0),
										2);
					}

					//Shows the disparity image
					imshow("Disparity Map", depth_image_ocv_display);
					key = cv::waitKey(10);

					//counter++;
				}
				//duration = (clock() - start) / (double)CLOCKS_PER_SEC;
			//}
			//cout << "Counter: " << counter << ", Duration: " << duration << endl;
		}
		//Print the values for the image to a txt file
		//printImageValues(depth_image_zed);
		imshow("Depth Selected", depth_image_ocv_display);
		cv::waitKey();
		cin >> key;
	}

	autopilot_interface.stop();
	serial_port.stop();

	zed.close();	//Close the ZED camera
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

//Calculate the center points for each rectangle
void partition(int *width, int *height)
{
	width[0] = HALF_WIDTH;
	width[NUM_RECT - 1] = WIDTH - HALF_WIDTH;
	height[0] = HALF_HEIGHT;
	height[NUM_RECT - 1] = HEIGHT - HALF_HEIGHT;

	fillArray(width, 0, NUM_RECT - 1);	//Fill the remaining positions of the width array
	fillArray(height, 0, NUM_RECT - 1);	//Fill the remaining positions of the height array
}

//Recursive function that finds all of the center points between two points in the array
void fillArray(int *arr, const int& start, const int& end)
{
	//Return if there are no more empty positions
	if(end - start == 1)
		return;
	//Find the middle index between the two end positions
	int mid = (start + end) / 2;
	//Set the middle index to the average of the two end positions
	arr[mid] = (arr[start] + arr[end]) / 2;
	fillArray(arr, start, mid);	//Fill in the array between the start and the middle positions
	fillArray(arr, mid, end);	//Fill in the array between the middle and the end positions
}

//Returns the percentage of pixels that are above the threshold
float getPercentage(const int& numBelow)
{
	return ((float)numBelow / TOTAL_PIXELS) * 100;
}

//Iterates through the image and increments the appropriate counters
void countPixels(sl::Mat& depthMap, const int *width, const int *height, float *sectionValues)
{
	bool rows[NUM_RECT];	//Keeps track of the possible rows the pixel can be in
	bool cols[NUM_RECT];	//Keeps track of the possible columns the pixel can be in
	int sections[TOTAL_RECT];	//Keeps track of how many pixels are below the DIS_THRESH in each section

	//Initializes the values to 0
	for(int i = 0; i < TOTAL_RECT; i++)
		sections[i] = 0;

	//Initializes the vlues to false
	for(int i = 0; i < NUM_RECT; i++)
	{
		rows[i] = false;
		cols[i] = false;
	}

	//The first pixels will always be in the rectangle that is in the first row and column
	rows[0] = true;
	cols[0] = true;

	//Iterate through all rows of pixels
	for(int y = 0; y < depthMap.getHeight(); y++)
	{
		//Update if the new row of pixels falls in new rows of rectangles
		checkRows(rows, height, y);

		//Reset the values in the columns array
		for(int i = 0; i < NUM_RECT; i++)
			cols[i] = false;
		cols[0] = true;
		
		//Iterate through all columns of pixels
		for(int x = 0; x < depthMap.getWidth(); x++)
		{
			//Update if the new column of pixels falls in new columns of rectangles
			checkCols(cols, width, x);

			float depth;	//Holds the depth at the pixel
			depthMap.getValue(x, y, &depth);	//Finds the depth at the current pixel

			//If the current pixel is below the DIS_THRESH then update the appropriate counters
			if(depth <= DIS_THRESH || depth == NAN || depth == TOO_CLOSE)
				updateCounters(rows, cols, sections);
		}
	}
	
	calcPercentages(sectionValues, sections);	//Calculate the percentages of each section
}

//Update if the new column of pixels falls in new columns of rectangles
void checkCols(bool *cols, const int *width, const int& x)
{
	int i = 0;	//Start with the first column
	//Continue on to the next column of rectangles until the end or when there is a column that the last pixel fell into
	while(i < NUM_RECT && cols[i] == false)
		i++;
	//If the new pixel is not in this column then set the value to false
	if(i < NUM_RECT && width[i] + HALF_WIDTH <= x)
		cols[i] = false;
	i++;
	//Coninue on to the end or when there is a column that the last pixel didn't fall into
	while(i < NUM_RECT && cols[i] == true)
		i++;
	//If the new pixel is in this column then set the vlaue to true
	if(i < NUM_RECT && width[i] - HALF_WIDTH <= x)
		cols[i] = true;
}

//Update if the new row of pixels falls in new rows of rectangles
void checkRows(bool *rows, const int *height, const int& y)
{
	int i = 0;	//Start with the first row
	//Continue on to the next row of rectangles until the end or when there is a row that the last pixel fell into
	while(i < NUM_RECT && rows[i] == false)
		i++;
	//If the new pixel is not in this row then set the value to false
	if(i < NUM_RECT && height[i] + HALF_HEIGHT <= y)
		rows[i] = false;
	i++;
	//Continue on to the end or when there is a row that the last pixel didn't fall into
	while(i < NUM_RECT && rows[i] == true)
		i++;
	//If the new pixel is in this row then set the value to true
	if(i < NUM_RECT && height[i] - HALF_HEIGHT <= y)
		rows[i] = true;
}

//Update the counters of the rectangles that the current pixel is in
void updateCounters(const bool *rows, const bool *cols, int *sections)
{
	int r = 0;	//Used to find the first row of rectangles that the pixel is in
	int c = 0;	//Used to find the first column of rectangles that the pixel is in

	//Continue moving along the rows until the end or a true value is found
	while(r < NUM_RECT && rows[r] == false)
		r++;
	//Continue moving along the columns until the end or a true value is found
	while(c < NUM_RECT && cols[c] == false)
		c++;

	int r2 = r;	//Place holder for the remaining rows of rectangles the pixel falls in
	int c2 = c;	//Place holder for the remaining columns of rectangles the pixel falls in

	//Increase the counter each rectangle the pixel falls in
	while(r2 < NUM_RECT && rows[r2] == true)
	{
		c2 = c;	//Reset the column counter for each new row of rectangles the pixel falls in
		while(c2 < NUM_RECT && cols[c2]== true)
		{
			sections[(r2 * NUM_RECT) + c2]++;	//Increase the counter of the current rectangle
			c2++;
		}
		r2++;
	}
}

//Calculate the percentage fo each rectangle
void calcPercentages(float *sectionValues, const int *sections)
{
	//ofstream file;
	//file.open("percentages.txt");

	for(int i = 0; i < TOTAL_RECT; i++)
	{
		sectionValues[i] = getPercentage(sections[i]);
		//file << "Section " << i << ": " << sections[i] << " / " << TOTAL_PIXELS << endl;
		//file << "Section " << i << ": " << sectionValues[i] << "%\n";
	}

	//file.close();
}



//Selects the section that has the smallest percentage
int selectSection(const float *sectionValues)
{
	int positions[TOTAL_RECT];	//Keep track of any rectangle with the same percentage value (will be the lowest percentage value)
	int position = 0;
	//Start with the center section so the UAV flies to the destination
	float minPercent = sectionValues[0];//TOTAL_RECT / 2];
	int minPos = 0;//TOTAL_RECT / 2;
	positions[position++] = minPos;
	//if(minPercent < PER_THRESH)
		//return minPos;
	//Get the section number that has the lowest percentage
	for (int i = 0; i < TOTAL_RECT; i++)
	{
		if (sectionValues[i] < minPercent)
		{
			clearPositions(positions, position);
			minPercent = sectionValues[i];
			minPos = i;
			positions[position++] = i;
		}
		else if(sectionValues[i] == minPercent)
		{
			positions[position++] = i;
		}
	}
	//ofstream file;
	//file.open("minPercentages.txt");
	//for(int i = 0; i < position; i++)
		//file << percentages[i] << endl;
	//file.close();
	//Check the minPercent with the percentage threshold
	if (minPercent < PER_THRESH)
		return closest(positions, position);
	return -1;	//The section with the smallest percentage has a percentage higher than the percentage threshold
}

void clearPositions(int *positions, int& position)
{
	for(int i = 0; i < position; i++)
		positions[i] = 0;
	position = 0;
}

int closest(const int *positions, const int& position)
{
	int closestPos = positions[0];
	float closestDis = distanceCalc(positions[0]);
	for(int i = 1; i < position; i++)
	{
		float distance = distanceCalc(positions[i]);
		if(distance < closestDis)
		{
			closestDis = distance;
			closestPos = positions[i];
		}
	}
	return closestPos;
}

float distanceCalc(const int& section)
{
	int row = section / NUM_RECT;
	int col = section % NUM_RECT;
	int deltaRow = abs(row - (NUM_RECT / 2));
	int deltaCol = abs(col - (NUM_RECT / 2));
	return sqrt((deltaRow * deltaRow) + (deltaCol * deltaCol));
}

//Get the center of the selected section. This is for testing and seeing what section was selected
void getCenter(int& centerW, int& centerH, const int& section, const int *widthSections, const int *heightSections)
{
	//If there are no open sections, do not show a rectangle on the image
	if(section == -1)
	{
		centerW = 0;
		centerH = 0;
	}
	else
	{
		int row = section / NUM_RECT;
		int col = section % NUM_RECT;

		//Set the center points to the center points of the selected rectangle
		centerW = widthSections[col];
		centerH = heightSections[row];
	}
}

//Move the UAV in respect to the section that was selected.
void manuever(const int& section, Autopilot_Interface api, const int& centerW, const int& centerH)
{
	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in
	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	//There is no section that is open
	if(centerW == 0 || centerH == 0)
	{
		//Rotate in place
		set_velocity(0, 0, 0, sp);
		set_yaw_rate(PI / 4, sp);
	}
	else
	{
		int delta_w = abs(CENTER_WIDTH - centerW);
		int delta_h = abs(CENTER_HEIGHT - centerH);
		double theta = atan(delta_h / delta_w);
		double VELO_Y = VELO * cos(theta);
		double VELO_Z = VELO * sin(theta);

		//The selected section is on the left of the center
		if(centerW < CENTER_WIDTH)
		{
			//The selected section is directly left
			if(centerH == CENTER_HEIGHT)
			{
				set_velocity(0, VELO_Y * -1, 0, sp);
				set_yaw(ip.yaw, sp);
			}
			//The selected section is above the center and to the left
			else if(centerH < CENTER_HEIGHT)
			{
				//This is up and to the left. Will adjust the values to be at the angle of the section later
				set_velocity(0, VELO_Y * -1, VELO_Z * -1, sp);
				set_yaw(ip.yaw, sp);
			}
			//The selected section is below the center and to the left
			else
			{
				set_velocity(0, VELO_Y * -1, VELO_Z, sp);
				set_yaw(ip.yaw, sp);
			}
		}
		//The selected section is above, below or is the center
		else if(centerW == CENTER_WIDTH)
		{
			//The selected section is the center section
			if(centerH == CENTER_HEIGHT)
			{
				//For now the UAV will just hover in place but later we will have it fly to the waypoint
				set_velocity(0, 0, 0, sp);
				set_yaw(ip.yaw, sp);
			}
			//The selected section is directly above the center
			else if(centerH < CENTER_HEIGHT)
			{
				set_velocity(0, 0, VELO_Z * -1, sp);
				set_yaw(ip.yaw, sp);
			}
			//The selected section is directly below the center
			else
			{
				set_velocity(0, 0, VELO_Z, sp);
				set_yaw(ip.yaw, sp);
			}
		}
		//The selected section is to the right of the center
		else
		{
			//The selected section is directly to the right of the center
			if(centerH == CENTER_HEIGHT)
			{
				set_velocity(0, VELO_Y, 0, sp);
				set_yaw(ip.yaw, sp);
			}
			//The selected section is above the center and to the right
			else if(centerH < CENTER_HEIGHT)
			{
				set_velocity(0, VELO_Y, VELO_Z * -1, sp);
				set_yaw(ip.yaw, sp);
			}
			//The selected section is below the center and to the right
			else
			{
				set_velocity(0, VELO_Y, VELO_Z, sp);
				set_yaw(ip.yaw, sp);
			}
		}
	}

	api.update_setpoint(sp);	//Move in the appropriate direction
	api.disable_offboard_control();	//Stop getting the commands
}

void quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}
















