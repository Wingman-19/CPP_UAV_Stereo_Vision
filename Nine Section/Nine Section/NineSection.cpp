/*
	Creates a disparity map using the ZED stereo camera.
	Partitions the disparity map into 9 sections.
	Counts the number of pixels in each section that have disparity value higher than the set threshold.
	Selects the section with the lowest percentage (This section must still be less than the percentage threshold).
	Moves in the direction of the selected section.
	Continues to repeat this process until the destination is reached.
*/
#include <sl/Camera.hpp>
using namespace sl;

#define DIS_THRESH 120	//Threshold for the disparity values. The higher the value, the closer the object
#define PER_THRESH 20	//Threshold for the percentage of pixels in a section that are above the disThresh
#define PI 3.14159265359
#define TO_RADIANS  PI/180	//Used to convert degrees to radians
#define RADIUS_EARTH 6378037.0
#define DIST 10		//This is the distance the new waypoint will be set to the left or right of the UAV (in meters) (subject to change)

void partitionCalc(Mat, float*);	//Partition the disparity map and calculate all of the percentage of pixels higher than the threshold in each section
void countPixels(Mat, const int&, const int&, const int&, const int&, int&, int&);	//Count the number of pixels in a given section that are higher than the threshold
int getPercentage(const int&, const int&);	//Returns the percentage of pixels higher than the threshold in the given section
int selectSection(const float*);	//Selects the section with the lowest percentage that is lower than the percentage threshold
void manuever(const int&);	//Moves the UAV based on the section selected
int calcNewHeadingRight(int);	//Calculates what direction (to the right) the new waypoint will be in
int calcNewHeadingLeft(int);	//Calculates what direction (to the left) the new waypoint will be in
double getThetaRads(double);	//Returns the angle in radians
double getX(double);	//Returns the X coordinate of the new position (the current X location is 0)
double getY(double);	//Returns the Y coordinate of the new position (the current Y location is 0)

//Holds the GPS location (lat, long, alt)
struct Waypoint
{
	double lat;	//Latitude
	double lon;	//Longitude
	double alt;	//Altitude
};

int main(int argc, char **argv)
{
	bool flying = true;	//Used to continue gathering data from the stereo camera. Will be set to false when the destination is reached
						//May not need this. May be able to use an infinite loop instead and not worry about it
	//Creates a camera object so that we can get the disparity images
	Camera zed;

	//Initializes the parameters for the camera
	InitParameters init_params;
	init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	init_params.coordinate_units = UNIT_MILLIMETER;

	//Makes sure the camera is opened properly
	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS)
		exit(-1);

	//Creates the runtime parameters for the camera
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

	//Creates an object to hold the disparity map and allow us to access the individual pixels of the image
	Mat disparityMap;
	
	//Continues to gather data and select the correct section of the disparity map while the UAV is flying
	//As mentioned before, this may be able to be an infinate loop
	while (flying)
	{
		//Tries to get an image from the zed camera
		if (zed.grab() == SUCCESS)
		{
			//Gets the disparity map from the zed camera
			zed.retrieveMeasure(disparityMap, MEASURE_DISPARITY);

			float sectionValues[9];	//Holds the percentage of pixels that are above the threshold in each section of the disparity image
			partitionCalc(disparityMap, sectionValues);
			int section = selectSection(sectionValues);	//The section that is selected
			manuever(section);
		}
	}

	//Close the camera
	zed.close();
	//End the program
	return 0;
}

//Partition the disparity map and calculate all of the percentage of pixels higher than the threshold in each section
void partitionCalc(Mat disparityMap, float *sectionValues)
{
	//An array of values that seperates the disparity image across its width
	int startWidth[] = { 0, disparityMap.getWidth() / 3, disparityMap.getWidth() * (2.0 / 3) };
	//An array of values that seperates the disparity image across its height
	int startHeight[] = { 0, disparityMap.getHeight() / 3, disparityMap.getHeight() * (2.0 / 3) };

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

		//Calculates the number of pixels above the threshold for the current section
		countPixels(disparityMap, startW, startH, endW, endH, numAbove, totalPix);

		//Calculate the percentage
		sectionValues[i] = getPercentage(numAbove, totalPix);
	}
}

//Count the number of pixels in a given section that are higher than the threshold
void countPixels(Mat disparityMap, const int& startW, const int& startH, const int& endW, const int& endH, int& numAbove, int& totalPix)
{
	//Loop through all of the pixels in the given section
	for (int x = startW; x < endW; x++)
	{
		for (int y = startH; y < endH; y++)
		{
			totalPix++;	//Increase the total number of pixels by 1
			int disparityValue = 0;	//Initialize the disparity value to 0
			disparityMap.getValue(x, y, &disparityValue);	//Get the disparity value of the current pixel
			//Checks to see if the value is greater than the given threshold
			if (disparityValue > DIS_THRESH)
				numAbove++;	//Increases the number of pixels greater than the threshold by 1
		}
	}
}

//Returns the percentage of pixels that are above the threshold
int getPercentage(const int& numAbove, const int& totalPix)
{
	return (numAbove / totalPix) * 100;
}

//Selects the section that has the smallest percentage
int selectSection(const float *sectionValues)
{
	float minPercent = sectionValues[0];
	//Get the section number that has the lowest percentage
	for (int i = 1; i < 9; i++)
		if (sectionValues[i] < minPercent)
			minPercent = sectionValues[i];
	//Check the minPercent with the percentage threshold
	if (minPercent < PER_THRESH)
		return minPercent;
	return -1;	//The section with the smallest percentage has a percentage higher than the percentage threshold
}

//Move the UAV in respect to the section that was selected.
void manuever(const int& section)
{
	switch (section)
	{
		case 0:
			//Set temp waypoint left and increase alt
			break;
		case 1:
			//Increase alt
			break;
		case 2:
			//Set temp waypoint right and increase alt
			break;
		case 3:
			//Set temp waypoint left
			break;
		case 4:
			//Continue to destination
			break;
		case 5:
			//Set temp waypoint right
			break;
		case 6:
			//Set temp waypoint left and decrease alt
			break;
		case 7:
			//Decrease alt
			break;
		case 8:
			//Set temp waypoint right and decrease alt
			break;
		default:
			//Turn in place or hover in place
	}
}

//Calculates what direction (to the right) the new waypoint will be in
int calcNewHeadingRight(int heading)
{
	int newHeading = heading + 90;	//Creates a right angle (to the right) with the old heading
	//Check to see if the new angle will have a value greater than 360. Adjust the value if it is
	if (newHeading > 360)
		newHeading -= 360;
	return newHeading;
}

//Calculates what direction (to the left) the new waypoint will be in
int calcNewHeadingLeft(int heading)
{
	int newHeading = heading - 90;	//Creates a right angle (to the left) with the old heading
	//Check to see if the new heading is negative. Adjust the value if it is
	if (newHeading < 0)
		newHeading += 360;
	return newHeading;
}

//Returns the angle in radians
double getThetaRads(double heading)
{
	double theta;
	//Check to see if the heading is in any quadrant other than the first
	//Gives the angle with respect to the positive x axis so we can use the Unit Circle
	if (heading > 90)
		theta = 360 - heading + 90;
	else
		theta = 90 - heading;
	theta = theta * TO_RADIANS;	//Convert from degrees to radians
	return theta;
}

//Returns the X coordinate of the new position (the current X location is 0)
double getX(double theta)
{
	double x = cos(theta) * DIST;	//Find the new x coordinate of the new waypoint
	return x;
}

//Returns the Y coordinate of the new position (the current Y location is 0)
double getY(double theta)
{
	double y = sin(theta) * DIST;	//Find the new y coordinate of the new waypoint
	return y;
}