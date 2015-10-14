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


void Superpixel::calculate_img_pixel_mask(IplImage *img)
{
	this->pixel_mask = cv::Mat::zeros(img->height, img->width, CV_8UC1);
	for(int i=0; i<points.size() ;i++)
	{
		this->pixel_mask.ptr<uchar>((int)points[i].x)[(int)points[i].y] = 255;
	}

	this->bounding_rect = cv::boundingRect(this->points);

	this->pixels = cv::Mat::zeros(this->bounding_rect.height, this->bounding_rect.width, CV_8UC3);
	for (int i = bounding_rect.x; i < bounding_rect.x + bounding_rect.width; i++)
	{
		for (int j = bounding_rect.y; j < bounding_rect.y + bounding_rect.height; j++)
		{
			CvScalar colour = cvGet2D(img, j, i);
			this->pixels.at<cv::Vec3b>(j,i)[0] = colour.val[0];
			this->pixels.at<cv::Vec3b>(j,i)[1] = colour.val[1];
			this->pixels.at<cv::Vec3b>(j,i)[2] = colour.val[2];
			/*this->pixels.ptr<uchar>(i)[j] = colour.val[2];
			this->pixels.ptr<uchar>(i)[j] = colour.val[1];
			this->pixels.ptr<uchar>(i)[j] = colour.val[0];
*/
		}
	}
	cv::imshow("superpixel", this->pixels);
	/*cv::Point pt1, pt2;
	pt1.x = rect.x;
	pt1.y = rect.y;
	pt2.x = rect.x + rect.width;
	pt2.y = rect.y + rect.height;
	rectangle(pixel_mask, pt1, pt2, CV_RGB(255,0,0), 1);
	cv::imshow("pixel_mask", pixel_mask);
	*/
}


void Superpixel::export_to_jpeg()
{
	imwrite( "../../images/Gray_Image.jpg", this->pixels );
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


