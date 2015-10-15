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
	//this->pixels = cv::Mat(50, 50, CV_8UC3, cvScalar(0,0,0));
}


Superpixel::Superpixel(int id, int num_points, cv::Point center, RGBcolourFrequencyChart histogram, point2Dvec points)
{
	this->id = id;
	this->num_points = num_points;
	this->center = center;
	this->histogram = histogram;
	this->points = points;
}


Superpixel::Superpixel(int id, cv::Point center)
{
	this->id = id;
	this->num_points = 0;
	this->center = center;
	this->histogram = init_FrequencyChart();
}

cv::Point Superpixel::get_center()
{
	return this->center;
}


RGBcolourFrequencyChart Superpixel::get_histogram()
{
	return this->histogram;
}

point2Dvec Superpixel::get_points()
{
	return this->points;
}

void Superpixel::add_point(cv::Point point)
{
	this->points.push_back(point);
	this->num_points += 1;
}


void Superpixel::add_histogram_colorFrequencies(int R, int G, int B)
{
	/*
	if((this->id == 29) || (this->id == 29))
		cout << "ID = " << id << ".		R = " << R << ".	G = " << G << ".	B = " << B << ".\n";
	*/	

	this->histogram[R][0] += 1;
	this->histogram[G][1] += 1;
	this->histogram[B][2] += 1;
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


    cv::Point invalidPoint(-1, -1);

    center = invalidPoint;

	for(i=0; i<num_points ;i++)
	{
		points.push_back(invalidPoint);
	}
}


RGBcolourFrequencyChart Superpixel::init_FrequencyChart()
{
	RGBcolourFrequencyChart histogram;
	int i;

    vector<int> colour;
    for (i=0; i<3 ;i++) {
        colour.push_back(0);
    }
	for (i=0; i<256 ;i++) 
	{
        histogram.push_back(colour);
    }
    return histogram;
}

void Superpixel::print_everything()
{
	int i;
	cout << "Superpixel #" << id << "\n";
	cout << "Number of points = " << num_points << "\n";
	cout << "Center = (" << center.x << ", " << center.y << ")" << "\n";
	cout << "Points:\n";
	for(int i=0; i<num_points ;i++)
	{
		cout << "(" << points[i].x << ", " << points[i].y << ")" << "\n";
	}

	cout << "Histogram:\n";
	for(int i=0; i<256 ;i++)
	{
		cout << "Valor = " << i << ".	R = " << histogram[i][0] << ".	G = " << histogram[i][1] << ".	B = " << histogram[i][2] << ".\n";
	}
}


void Superpixel::calculate_bounding_rect(IplImage *img)
{	
//	cout << "hola\n";
	this->bounding_rect = cv::boundingRect(this->points);
//	cout << "numero de pixels = " << this->pixels.size() << "	numero de puntos = " << this->points.size()  << "		rectangulo, ancho = " << bounding_rect.width << "   largo =" << bounding_rect.height << "\n";
	

//	cout << "listo\n";
	
	/*cv::Point pt1, pt2;
	pt1.x = rect.x;
	pt1.y = rect.y;
	pt2.x = rect.x + rect.width;
	pt2.y = rect.y + rect.height;
	rectangle(pixel_mask, pt1, pt2, CV_RGB(255,0,0), 1);
	cv::imshow("pixel_mask", pixel_mask);
	*/
}


void Superpixel::export_to_jpeg(IplImage *img)
{
	CvScalar colour;
	this->pixels = cv::Mat::zeros(this->bounding_rect.width, this->bounding_rect.height, CV_8UC3);
	int x_coord, y_coord;
	for (int i = bounding_rect.x; i < bounding_rect.x+bounding_rect.width -1; i++)
	{
		x_coord = i - bounding_rect.x;
		for (int j = bounding_rect.y; j < bounding_rect.y+bounding_rect.height -1; j++)
		{
			y_coord = j - bounding_rect.y;
			colour = cvGet2D(img, j, i);

			(this->pixels.at<cv::Vec3b>(x_coord, y_coord)).val[0] = colour.val[0];
			(this->pixels.at<cv::Vec3b>(x_coord, y_coord)).val[1] = colour.val[1];
			(this->pixels.at<cv::Vec3b>(x_coord, y_coord)).val[2] = colour.val[2];
		}
	}

	cv::imshow("superpixel", this->pixels);
	string path = "/home/mecatronica/Github_Yoshua/VisionPercepcion_USB2015/Codigos/Tesis_ws/src/Plane_Segmentation_3/superpixel_images/";
	string file_name = to_string(this->id) + ".jpg";


	imwrite(path+file_name, pixels);
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
	//center.clear();
	histogram.clear();
	points.clear();
}


