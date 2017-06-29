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

void partitionCalc(Mat, float*);
void countPixels(Mat, const int&, const int&, const int&, const int&, int&, int&);
int getPercentage(const int&, const int&);
int selectSection(const float*);
void manuever(const int&);

int main(int argc, char **argv)
{
	bool flying = true;
	Camera zed;

	InitParameters init_params;
	init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	init_params.coordinate_units = UNIT_MILLIMETER;

	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS)
		exit(-1);

	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

	Mat depth;
	while (flying)
	{
		if (zed.grab() == SUCCESS)
		{
			zed.retrieveMeasure(depth, MEASURE_DEPTH);

			float sectionValues[9];
			partitionCalc(depth, sectionValues);
			int section = selectSection(sectionValues);
			manuever(section);
		}
	}

	zed.close();
	return 0;
}

//Partition the disparity map and calculate all of the percentage of pixels higher than the threshold in each section
void partitionCalc(Mat depth, float *sectionValues)
{
	int startWidth[] = { 0, depth.getWidth() / 3, depth.getWidth() * (2.0 / 3) };
	int startHeight[] = { 0, depth.getHeight() / 3, depth.getHeight() * (2.0 / 3) };

	int numAbove = 0;
	int totalPix = 0;
	
	for (int i = 0; i < 9; i++)
	{
		int row = i / 3;	//Used to know the row of the current section
		int col = i % 3;	//Used to know the column of the current section
		int startW = startWidth[col];	//Used to know the starting point for the width of the current section
		int startH = startHeight[row];	//Used to know the starting point for the height of the current section
		int endW;	//Used to know where the width of the current section ends
		int endH;	//Used to know where the height of the current section ends
		if (col == 2)
			endW = depth.getWidth();	//The width ends at the overall width of the image
		else
			endW = startWidth[col + 1];	//The width ends at the point the next section starts
		if (row == 2)
			endH = depth.getHeight();	//The height ends at the overall height of the image
		else
			endH = startHeight[row + 1];	//The height ends at the point the next section starts

		//Calculates the number of pixels above the threshold for the current section
		countPixels(depth, startW, startH, endW, endH, numAbove, totalPix);

		//Calculate the percentage
		sectionValues[i] = getPercentage(numAbove, totalPix);
	}
}

//Count the number of pixels in a given section
void countPixels(Mat depth, const int& startW, const int& startH, const int& endW, const int& endH, int& numAbove, int& totalPix)
{
	for (int x = startW; x < endW; x++)
	{
		for (int y = startH; y < endH; y++)
		{
			totalPix++;
			float depthVal = 0.0f;
			depth.getValue(x, y, &depthVal);
			if (depthVal > DIS_THRESH)
				numAbove++;
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