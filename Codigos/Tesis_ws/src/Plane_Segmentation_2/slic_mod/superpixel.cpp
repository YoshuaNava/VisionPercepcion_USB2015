#include "superpixel.h"

/*
 * Constructor.
 */
Superpixel::Superpixel()
{

}

Superpixel::Superpixel(int id, int num_points)
{
	this->id = id;
	this->num_points = num_points;
	init_structures(num_points);
}


Superpixel::Superpixel(int id, int num_points, point2D center, RGBcolourFrequencyChart histogram, point2Dvec points)
{
	this->id = id;
	this->num_points = num_points;
	this->center = center;
	this->histogram = histogram;
	this->points = points;
}



void Superpixel::init_structures(int num_points)
{
	int i;

    vector<int> colour;
    for (i=0; i<3 ;i++) {
        colour.push_back(0);
    }
	for (i=0; i<256 ;i++) 
	{
        histogram.push_back(colour);
    }

    point2D invalidPoint;
    for (i=0; i<2 ;i++) {
        invalidPoint.push_back(-1);
    }

    center = invalidPoint;

	for(i=0; i<num_points ;i++)
	{
		points.push_back(invalidPoint);
	}
}

void Superpixel::print_everything()
{
	int i;
	cout << "Superpixel #" << id << "\n";
	cout << "Number of points = " << num_points << "\n";
	cout << "Center = (" << center[0] << ", " << center[1] << ")" << "\n";
	cout << "Points:\n";
	for(int i=0; i<num_points ;i++)
	{
		cout << "(" << points[i][0] << ", " << points[i][1] << ")" << "\n";
	}

	cout << "Histogram:\n";
	for(int i=0; i<256 ;i++)
	{
		cout << "Valor = " << i << ".	R = " << histogram[i][0] << ".	G = " << histogram[i][1] << ".	B = " << histogram[i][2] << ".\n";
	}
}



 /*
 * Destructor. Clear any present data.
 */
Superpixel::~Superpixel() 
{
    clear_data();
}


/*
 * Clear the data as saved by the algorithm.
 *
 * Input : -
 * Output: -
 */
void Superpixel::clear_data() 
{
	center.clear();
	histogram.clear();
	points.clear();
}


