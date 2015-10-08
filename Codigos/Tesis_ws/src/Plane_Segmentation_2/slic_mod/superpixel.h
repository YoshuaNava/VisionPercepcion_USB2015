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

		void init_structures(int num_points);
		void clear_data();
	public:
		/* Class constructors and deconstructors. */
		Superpixel();
		Superpixel(int id, int num_points);
		Superpixel(int id, point2D center);
		Superpixel(int id, int num_points, point2D center, RGBcolourFrequencyChart histogram, point2Dvec points);
		~Superpixel();

		point2D get_center();
		point2Dvec get_points();
		RGBcolourFrequencyChart get_histogram();

		static RGBcolourFrequencyChart init_FrequencyChart();
		
		void print_everything();
		void add_point(point2D point);
		void add_histogram_colorFrequencies(int R, int G, int B);
};

#endif