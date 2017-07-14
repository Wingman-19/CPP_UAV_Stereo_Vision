#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include <iostream>
#include <fstream>

using namespace sl;
using namespace std;

#define DIS_THRESH 120	//Threshold for the disparity values. The higher the value, the closer the object
#define PER_THRESH 20	//Threshold for the percentage of pixels in a section that are above the disThresh

void printImageValues(cv::Mat&);	//Prints the values of each pixel to a text file (This is used for testing)
cv::Mat slMat2cvMat(sl::Mat& input);	//Converts a sl::Mat to a cv::Mat
void partitionCalc(cv::Mat&, float*);	//Partition the disparity map and calculate all of the percentage of pixels higher than the threshold in each section
void countPixels(cv::Mat&, const int&, const int&, const int&, const int&, int&, int&);	//Count the number of pixels in a given section that are higher than the threshold
int getPercentage(const int&, const int&);	//Returns the percentage of pixels higher than the threshold in the given section
int selectSection(const float*);	//Selects the section with the lowest percentage that is lower than the percentage threshold
void manuever(const int&); //Moves the UAV based on the section selected

int main()
{
	// Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    init_params.depth_mode = DEPTH_MODE_MEDIUM;	//Can be set to PERFORMANCE, MEDIUM, OR QUALITY
    init_params.coordinate_units = sl::UNIT_METER;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        return 1;

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_FILL; // Use STANDARD sensing mode

    // Create sl and cv Mat to get ZED left image and depth image
    // Best way of sharing sl::Mat and cv::Mat :
    // Create a sl::Mat and then construct a cv::Mat using the ptr to sl::Mat data.
    Resolution image_size = zed.getResolution();
				//This is used to see the right and left image and is only for testing
				/*sl::Mat image_zed_left(image_size,sl::MAT_TYPE_8U_C4); // Create a sl::Mat to handle Left image
				sl::Mat image_zed_right(image_size,sl::MAT_TYPE_8U_C4); // Create a sl::Mat to handle right image
				cv::Mat image_ocv_left = slMat2cvMat(image_zed_left);*/
	sl::Mat depth_image_zed(image_size, MAT_TYPE_8U_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
				//cv::Mat image_ocv_right = slMat2cvMat(image_zed_right);


    // Create OpenCV images to display (lower resolution to fit the screen)
    cv::Size displaySize(720, 404);
				//cv::Mat image_ocv_left_display(displaySize, CV_8UC4);
				//cv::Mat image_ocv_right_display(displaySize, CV_8UC4);
    cv::Mat depth_image_ocv_display(displaySize, CV_8UC4);

	// Jetson only. Execute the calling thread on 2nd core
    Camera::sticktoCPUCore(2);

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
		        zed.retrieveImage(depth_image_zed, VIEW_DEPTH); //Retrieve the depth view (image)
				//zed.retrieveMeasure(depth_image_zed, MEASURE_DISPARITY);

		        // Resize and display with OpenCV
							/*cv::resize(image_ocv_left, image_ocv_left_display, displaySize);
							imshow("Image Left", image_ocv_left_display);
							cv::resize(image_ocv_right, image_ocv_right_display, displaySize);
							imshow("Image right", image_ocv_right_display);*/

				float sectionValues[9];	//Holds the percentage of pixels that are above the threshold in each section of the disparity image
				partitionCalc(depth_image_ocv_display, sectionValues);	//Partitions the image and calculates the percent of pixels in each section
				int section = selectSection(sectionValues);		//The section that is selected
				cout << "The selected section is: " << section << endl;
				manuever(section);	//Moves the UAV in a certain direction

		        cv::resize(depth_image_ocv, depth_image_ocv_display, displaySize);	//Used to print the disparity map
				//Shows the lines that partition the image
				cv::line(depth_image_ocv_display,
								cv::Point(depth_image_ocv_display.cols / 3, 0),
								cv::Point(depth_image_ocv_display.cols / 3, depth_image_ocv_display.rows),
								0,
								1);
				cv::line(depth_image_ocv_display,
								cv::Point(depth_image_ocv_display.cols * (2.0 / 3), 0),
								cv::Point(depth_image_ocv_display.cols * (2.0 / 3), depth_image_ocv_display.rows),
								0,
								1);
				cv::line(depth_image_ocv_display,
								cv::Point(0, depth_image_ocv_display.rows / 3),
								cv::Point(depth_image_ocv_display.cols, depth_image_ocv_display.rows / 3),
								0,
								1);
				cv::line(depth_image_ocv_display,
								cv::Point(0, depth_image_ocv_display.rows * (2.0 / 3)),
								cv::Point(depth_image_ocv_display.cols ,depth_image_ocv_display.rows * (2.0 / 3)),
								0,
								1);
				//Shows the disparity image
		        imshow("Disparity Map", depth_image_ocv_display);
		        key = cv::waitKey(10);
		    }
		}
		//Print the values for the image to a txt file
		printImageValues(depth_image_ocv_display);
		imshow("Depth Selected", depth_image_ocv_display);
		cv::waitKey();
		cin >> key;
	}
	zed.close();
	return 0;
}

//Converts the sl::Mat to the cv::Mat
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
void printImageValues(cv::Mat& dispMap)
{
	ofstream myFile;
	myFile.open("imageValues.txt");
	myFile << "[ ";
	for(int y = 0; y < dispMap.rows; y++)
	{
		uchar *row = dispMap.ptr<uchar>(y);
		for(int x = 0; x < dispMap.cols; x += 4)
		{
			sl::float1 disp = row[x];
			myFile << disp << ", ";
		}
		myFile << endl;
	}
	myFile << " ]";
	cout << "Image wrote to file";
	myFile.close();
}

//Partition the disparity map and calculate all of the percentage of pixels higher than the threshold in each section
void partitionCalc(cv::Mat& disparityMap, float *sectionValues)
{
	//An array of values that seperates the disparity image across its width
	int startWidth[] = { 0, disparityMap.cols / 3, disparityMap.cols * (2.0 / 3) };
	//An array of values that seperates the disparity image across its height
	int startHeight[] = { 0, disparityMap.rows / 3, disparityMap.rows * (2.0 / 3) };

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
			endW = disparityMap.cols;	//The width ends at the overall width of the image
		else
			endW = startWidth[col + 1];	//The width ends at the point the next section starts
		if (row == 2)
			endH = disparityMap.rows;	//The height ends at the overall height of the image
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
void countPixels(cv::Mat& disparityMap, const int& startW, const int& startH, const int& endW, const int& endH, int& numAbove, int& totalPix)
{

	//cout << disparityMap.cols << " , " << disparityMap.rows <<  " , " << startW << " , " << endW << " , " << startH << " , " << endH << endl;

	//Loop through all of the pixels in the given section
	for(int y = startH; y < endH; y++)
	{
		uchar *row = disparityMap.ptr<uchar>(y);
		for(int x = startW * 4; x < endW * 4; x += 4)
		{
			totalPix++;	//Increase the total number of pixels by 1
			//Checks to see if the value is greater than the given threshold
			if(row[x] >= DIS_THRESH)
			{
				numAbove++;	//Increases the number of pixels greater than the threshold by 1
			}
		}
	}
}

//Returns the percentage of pixels that are above the threshold
int getPercentage(const int& numAbove, const int& totalPix)
{
	return ((float)(numAbove) / totalPix) * 100;
}


//Selects the section that has the smallest percentage
int selectSection(const float *sectionValues)
{
	//Start with the center section so the UAV flies to the destination
	float minPercent = sectionValues[4];
	int minPos = 4;
	if(minPercent < PER_THRESH)
		return minPos;
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
void manuever(const int& section)
{
	switch (section)
	{
		case 0:
			//Move left and increase altitude
			cout << "The UAV is moving up and left.";
			break;
		case 1:
			//Increase altitude
			cout << "The UAV is moving up.";
			break;
		case 2:
			//Move right and increase altitude
			cout << "The UAV is moving up and right.";
			break;
		case 3:
			//Move left
			cout << "The UAV is moving left.";
			break;
		case 4:
			//Continue to destination
			cout << "The UAV is flying straight.";
			break;
		case 5:
			//Move right
			cout << "The UAV is moving right.";
			break;
		case 6:
			//Move left and decrease altitude
			cout << "The UAV is moving down and left.";
			break;
		case 7:
			//Decrease altitude
			cout << "The UAV is moving down.";
			break;
		case 8:
			//Move right and decrease altitude
			cout << "The UAV is moving down and right.";
			break;
		default:
			//Turn in place or hover in place
			cout << "The UAV is turning in place.";
	}
	cout << endl;
}











