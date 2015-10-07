#ifndef SUPERPIXEL_H
#define SUPERPIXEL_H

/* superpixel.h.
 *
 * Written by: Yoshua Nava
 *
 * This file contains the class elements of the class Superpixel. The purpose of
 * this class is to serve as a container of RGB Superpixels segmented by different
 * algorithms, so that they can be treated separately as an entity.
 *
 */


#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
//#include <float.h>
using namespace std;

#define RGBcolourFrequencyChart vector<vector<int>>	//Taken from the original slic class
#define point2D vector<double>
#define point2Dvec vector<point2D>

class Superpixel 
{
	private:
		int id;
		int num_points;
		point2D center;
		RGBcolourFrequencyChart histogram;
		point2Dvec points;

		void clear_data();
		void init_structures(int num_points);

	public:
		/* Class constructors and deconstructors. */
		Superpixel();
		Superpixel(int id, int num_points);
		Superpixel(int id, int num_points, point2D center, RGBcolourFrequencyChart histogram, point2Dvec points);
		~Superpixel();


		void print_everything();

};

#endif