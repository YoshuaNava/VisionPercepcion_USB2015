



class MyInterface
{
	private:
		vector<vector<int>> clusters;
		
		vector<Superpixel> superpixels_list;
		vector<vector<int>> superpixels_adjacency_matrix;
		vector<vector<int>> superpixels_Gsimilarity_matrix;
		cv::Mat frame, seg_image, gray, contours_image;
	public:
		virtual cv::Mat generateSuperpixels(cv::Mat image, cv::Mat gray_image);
		virtual cv::Mat outlineSuperpixelsContours(CvScalar colour);
		virtual void calculateSuperpixelCenters();
		virtual vector<Superpixel> storeSuperpixelsMemory();
		virtual cv::Mat displayCenterGrid(cv::Mat image, CvScalar colour);
};