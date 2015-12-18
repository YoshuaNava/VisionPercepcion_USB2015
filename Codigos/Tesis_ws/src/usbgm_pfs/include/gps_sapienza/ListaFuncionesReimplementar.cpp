Lista de funciones a reimplementar


img_proc_fcns.cpp

	void init_images_img_proc(CvSize S);
	void init_windows_img_proc(CvSize S);
	void release_images_img_proc( void );
	
	void GetModel(IplImage* gray_img, Features* F, Model* StatModelPtr, bool dynamic);
	
	void combine_channels(IplImage* Cr, IplImage* Cb, IplImage* a, IplImage* iic);
	// void getEdgeMagOri(IplImage* gray, IplImage* mag32, IplImage* ang32);
	// void convertGray2LBP(IplImage* gray, IplImage* LBP);
	// void GraphBasedSegmentation(IplImage* seg, IplImage* gray_source);
	void SuperPixelStats(IplImage *gbs, IplImage *gray, Statistics *sts);
	
	void ColorHistAnalysis(IplImage*src, IplImage* hsv, IplImage* col_cue, IplImage* col_prob, CvRect Safe);
	//LBP
	void LBPAnalysis(IplImage* LBP, IplImage* LBP_CUE, IplImage* LBP_PROB, CvRect Safe);
	
	void SuperPixelClassification(IplImage* gray, IplImage* prob_mask, Statistics *sts, IplImage* col_seg, IplImage* edge_seg, IplImage* lbp_seg);
	
	void FindObstacleBoundary(IplImage* Out);
	void ExtractBoundary(CvSize S, Boundary *B);
	void CalculateDistances(CvSize S, Boundary *B, CamCalib camera, BotCalib bot);
	void InterpretDepthArray(CvSize S, Boundary *B, BotCalib bot);
	
	void Contours(IplImage* binary);
	void DrawContours(IplImage* contour, CvScalar color, CvRect S);
	void ReleaseContours(void);
	
	void ProbAnalysis2(Features *F, Statistics* sts, IplImage* gbs);
	void UpdatePrior(IplImage *gbs, Statistics *sts, Features *F);
	void UpdateModel(Model* Current, Model* Prev);
	void UpdatePrevModel(Model* Current, Model* Prev);
	void FeatureAnalysis(Features *F, Model* M, Statistics *sts, IplImage *gbs, bool dynamic);
	bool CheckConvergence(Statistics* S, int em);
	
	void UpdateParams(IplImage* bin, Statistics *S, Features *F, bool dynamic);




init_structures.h

	void init_stats(CvSize Img_Size, Statistics * S, bool init);
	void init_boundary(CvSize S, Boundary * B);
	void init_model(CvSize S, CvRect SafeRegion, Model *M);
	void init_features(CvSize S, Features * F);