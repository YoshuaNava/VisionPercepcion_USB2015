Lista de funciones a reimplementar


img_proc_fcns.cpp

	// void init_images_img_proc(CvSize S);
	// void init_windows_img_proc(CvSize S);
	// void release_images_img_proc( void );
	// void combine_channels(IplImage* Cr, IplImage* Cb, IplImage* a, IplImage* iic);
	// void getEdgeMagOri(IplImage* gray, IplImage* mag32, IplImage* ang32);
	// void convertGray2LBP(IplImage* gray, IplImage* LBP);
	// void GraphBasedSegmentation(IplImage* seg, IplImage* gray_source);
		
	// void SuperPixelStats(IplImage *gbs, IplImage *gray, Statistics *sts);
	// void UpdatePrior(IplImage *gbs, Statistics *sts, Features *F);
	// void GetModel(IplImage* gray_img, Features* F, Model* StatModelPtr, bool dynamic);
	// static inline void PrintHistogram(int hist_size, CvHistogram *Hist, IplImage* Hist_img, const char * Window, bool flag, int X, int Y);
	// static inline void PrintGHistogram(int hist_size, CvHistogram *Hist, IplImage* Hist_img, const char * Window, bool flag, int X, int Y);
	void FeatureAnalysis(Features *F, Model* M, Statistics *sts, IplImage *gbs, bool dynamic);
	void ProbAnalysis2(Features *F, Statistics* sts, IplImage* gbs);
	void UpdateParams(IplImage* bin, Statistics *S, Features *F, bool dynamic);
	
	// void ColorHistAnalysis(IplImage*src, IplImage* hsv, IplImage* col_cue, IplImage* col_prob, CvRect Safe);
	//LBP
	// void LBPAnalysis(IplImage* LBP, IplImage* LBP_CUE, IplImage* LBP_PROB, CvRect Safe);
	
	// void SuperPixelClassification(IplImage* gray, IplImage* prob_mask, Statistics *sts, IplImage* col_seg, IplImage* edge_seg, IplImage* lbp_seg);
		
	
	// void UpdateModel(Model* Current, Model* Prev);
	// void UpdatePrevModel(Model* Current, Model* Prev);
	
	// bool CheckConvergence(Statistics* S, int em);
	
	

	// void FindObstacleBoundary(IplImage* Out);
	// void ExtractBoundary(CvSize S, Boundary *B);
	// void CalculateDistances(CvSize S, Boundary *B, CamCalib camera, BotCalib bot);
	// void InterpretDepthArray(CvSize S, Boundary *B, BotCalib bot);
	
	// void Contours(IplImage* binary);
	// void DrawContours(IplImage* contour, CvScalar color, CvRect S);
	// void ReleaseContours(void);



init_structures.h

	// void init_stats(CvSize Img_Size, Statistics * S, bool init);
	void init_boundary(CvSize S, Boundary * B);
	// void init_model(CvSize S, CvRect SafeRegion, Model *M);
	// void init_features(CvSize S, Features * F);
	
	
main.cpp

	int main(int argc, char** argv)
	{
	// CargarParametros();
		//parse command line options
	//     if (argc > 1){
	//         if( p.parse_cmd_options(argc,argv) != 0 ) return 0;
	//     }
	
	//     static int key=0; //holds user-input keystroke
	//     static int i=0,j=0;
	
	//     //LOOPS for generating results on datasets
	//     //for (int id=14;id<=14;id++){ // do for all the 15 video sequences [0-14]
	//     //for (int id=1;id<=1;id++){ //do once!
	//     //for (int T = -14;T<=14;T+=2){ // do for a range of Log posterior thresholds to generate a ROC curve
	//     //for (int run = 1; run<=400; run++){ do for several runs on the static traversability dataset
	//     //for (int T = 0;T<=1;T+=2){// do once!
	
	//     //local variables
	//     int T=0;
	//     if(p.verb){ printf("T=%d\n",T); }
	
	
	
	//     //j=-1; //image index number for image sequences without stopping
	//     j=0;
	
	//     p.log_post_thres_zero_position = T*10 + 300; //thres represents the log posterior threshold on a opencv `trackbar', 0 => 300
	
	//     //INITIALISATIONS
	//     FrameSize = cvSize(cvRound(p.proc_W), cvRound(p.proc_H)); //define image size
	//     source_img = cvCreateImage(FrameSize, 8, 3);
	
	
	// // VerificarOrigenImagenes();
	//     if (p.video_data){
	//         if (p.webcam){
	//             if( !initCapture() )            return 0;
	//         } //capture from webcam
	//     else{
	//         if( initVideoCapture() != 0)    return 1;} //capture video from file or network stream
	//     }
	
	// // InicializarContenedoresImagenesYalgoritmosEstimacion();
	//     input_image = cvCreateImage(p.capture_size,8,3);
	
	//     init_images_main(FrameSize);
	//     init_images_img_proc(FrameSize);
	//     //if (p.disp_img){ init_windows_img_proc(FrameSize); }
	
	//     init_safe_region();
	
	//     init_boundary(FrameSize, BoundPtr);
	//     init_model(FrameSize, SafeRegion, PrevModelPtr);
	//     init_model(FrameSize, SafeRegion, StatModelPtr);
	//     init_stats(FrameSize, StatsPtr, 1);
	//     init_features(FrameSize, FPtr);
	
	//     printf( "\n*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n"
	//     "This software runs a vision-based autonomous guidance algorithm\n"
	//     "for a mobile robot equipped with a low-quality monocular camera.\n"
	//     "The vision system allows a mobile robot to autonomously guide itself\n"
	//     "past static or dynamic obstacles in both indoor or outdoor natural environments\n"
	//     "in a real-time, reactive manner.\n\n"
	//     "To begin, make sure all parameters are correct and press 'Enter'\n\n"
	//     "To manually reinitialise the system press '*TODO*'\n\n"
	//     "To exit, click inside one of the display windows,\n"
	//     "then press the 'q' key\n\n\n"
	//     "***WARNING***\n"
	//     "--Prolonged use of this software may cause injuries--"
	//     "\n*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n");
	//     //fgetc(stdin);
	//     cvWaitKey(0);
	
	
	
	//     //MAIN PROCESSING LOOP
	//     //j = 363;
	//     int flag=1;
	
	//     while(1)
	//     {
	//         //SELECTION OF INPUT//
	//         static int frame_no = 0;
	//         //static int Im_ID_max = 53; //58
	
	// // GrabFrame();
	//         //--START-->CAPTURE FRAME//
	//         if(p.video_data){
	//             NextFrame(&input_image); //captures a video frame from webcam or video file
	
	//         }else if(!p.video_data){ //gets an image from file
	//             j = loop_through(j, key); //control test image sequence
	
	//             char buffer [250];
	//             sprintf(buffer,"static_traversability_dataset/source_%d.bmp", j);
	
	//             input_image = cvLoadImage(buffer);
	//             if (input_image == NULL){
	//                     fprintf(stderr, "Did not load image:\n");
	//                     fprintf(stderr, "%s\n", buffer);
	//                     cvWaitKey(0);
	//                     input_image = cvLoadImage("corridor.jpg");
	//             }
	//         }// end else if(!video_data)
	//         //--END-->CAPTURE FRAME//
	
			//if(!p.dynamic) init_stats(FrameSize, StatsPtr, 0);
			//--START-->IMAGE PROCESSING//
			//if(!finished){ //finished == 1 if something goes wrong with capture
	//         CV_TIMER_START(X)  //start timer
	//         if(p.verb){ printf("\nSTART OF LOOP\n"); }
	
	// // PreprocesamientoImagen();
	//         //IMAGE PRE-PROCESSING//
	//         cvResize(input_image, source_img, CV_INTER_AREA); 			//Resize
	//         //cvSmooth(source_img, source_img, CV_GAUSSIAN, 3, 3);//Smooth image
	//         if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, source_img, 0 , 0); }
	
	//         if (p.write_output_to_disk){
	//         char buffer1[150];
	//         sprintf(buffer1,"results/source_%d.bmp",frame_no);
	//         cv::Mat image(source_img);
	//         cv::imwrite(buffer1, image); //at time of writing imwrite was better/faster at saving images
	//         }
	
	//         cvCvtColor( source_img, gray_img, CV_BGR2GRAY);	//Convert Color to gray RGB[A]->Gray: Y<-0.299*R + 0.587*G + 0.114*B
	//         if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, gray_img, 1, 0); }
	
	//         cvCvtColor( source_img, F.hsv, 	CV_BGR2HSV );	//Convert to HSV
	//         cvCvtColor( source_img, F.YCrCb, CV_BGR2YCrCb );	//Convert to YCrCb
	//         cvCvtColor( source_img, F.lab, 	CV_BGR2Lab );	//Convert to Lab
	
	//         cvCvtPixToPlane( F.hsv, F.hue, F.sat, 0, 0 ); //cvCvtPixToPlane Divides a multi-channel array into separate single-channel arrays
	//         cvCvtPixToPlane( F.YCrCb, 0, F.Cr, F.Cb, 0 );
	//         cvCvtPixToPlane( F.lab, 0, F.a, 0, 0 );
	// //        cv::namedWindow( "Prueba", 1 );
	// //        cvShowImage("Prueba",F.hue);
	
	//         //view iamges separately
	//         if (p.disp_img){	DISPLAY_IMAGE_XY(p.refresh, F.hue, 2, 0);
	//                             DISPLAY_IMAGE_XY(p.refresh, F.sat, 3, 0); }
	
	//         combine_channels(F.Cr, F.Cb, F.a, F.iic); //D = (A + B + 2*C)/4 //illumination invariant color channel combination
	//         //cvEqualizeHist(F.iic, F.iic);
	//         cvNormalize(F.iic, F.iic, 0, 255, CV_MINMAX);
	//         if (p.disp_img){ 	DISPLAY_IMAGE_XY(p.refresh, F.iic, 4, 0); }
	
	//         convertGray2LBP(gray_img, F.lbp); //Convert to LBP
	
	//         //getEdgeMagOri(gray_img, F.mag32, F.ang32); //Edge Detection
	//         getEdgeMagOri(F.sat, F.mag, F.ang32); //Extract edge magnitudes and orientation
	//         if (p.disp_img){ 	DISPLAY_IMAGE_XY(p.refresh, F.mag, 5, 0);
	//                             DISPLAY_IMAGE_XY(p.refresh, F.ang32, 6, 0);
	//         }
	//         CV_TIMER_STOP(A, "Resize, Smooth, CvtColor, LBP, EDGE, IIC");
	
	//         //DISPLAY IMAGES
	//         if (p.disp_img){
	//             //DISPLAY_IMAGE_XY(p.refresh, source_img, 0 , 0);
	//             //DISPLAY_IMAGE_XY(p.refresh, gray_img, 1 , 0);
	//             DISPLAY_IMAGE_XY(p.refresh, F.lbp, 7, 0);
	//         //cvShowImage( GRAY_SRC, gray_img);
	//         //cvShowImage( LBP_IMG,  F.lbp);
	//         }
	
	// // SegmentacionGrafos();
	//         //SPLIT IMAGE INTO SUPERPIXELS USING FELZENSZWALB GRAPH-BASED SEGMENTATION
	//         GraphBasedSegmentation( gbs_img, gray_img);
			// CV_TIMER_STOP(B, "Felzenszwalb Graph Based Segmentation")
	
			//Superpixel Statistics
			SuperPixelStats(gbs_img, gray_img, StatsPtr);
			// if(p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, Stats.prior_img, 0 , 2); //cvShowImage(PRIOR, Stats.prior_img);
			// }
			CV_TIMER_STOP(C, "SuperPixel Statistics (mean, stdDev, size, position)")
	
	
	// ActualizarModeloProbabilistico();
			//if(p.move){
			if(!p.dynamic){
				GetModel(gray_img, FPtr, StatModelPtr, p.dynamic);
				CV_TIMER_STOP(D, "Get Model Statistics")
	
			}else if(p.dynamic){
				if( flag == 1 ){
	
					UpdatePrior(gbs_img, StatsPtr, FPtr);
					//UpdatePrevModel(StatModelPtr, PrevModelPtr);//
	
					//Get Ground Statistical Model
					GetModel(gray_img, FPtr, StatModelPtr, p.dynamic);
					CV_TIMER_STOP(D, "Get Model Statistics")
	
					//UpdateModel(StatModelPtr, PrevModelPtr);//
					flag=0;
				}
				flag++;
			}
			//}
	
			//for(int em=0; em<10;em++){
			//static bool converged = 0;
			FeatureAnalysis(FPtr, StatModelPtr, StatsPtr, gbs_img, p.dynamic);
			CV_TIMER_STOP(G, "Feature analysis")
	
			ProbAnalysis2(FPtr, StatsPtr, gbs_img);
			// if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, F.post1, 0 , 4);
			//cvShowImage(POSTERIOR, F.post1);
	
			}
			CV_TIMER_STOP(H, "Prob Analysis")
	
			UpdateParams(F.bin_class_result, StatsPtr, FPtr, p.dynamic);
			CV_TIMER_STOP(I, "EM Analysis")
	
	
	// EstimarFronteraObstaculo();
			//FIND OBSTACLE BOUNDARY AND COMPUTE STEERING DIRECTION
			//cvCopy(gbs_img,Bound.Bimg);
			cvMerge(F.bin_class_result, F.bin_class_result, F.bin_class_result, NULL, result_img);
			cvCopy(F.bin_class_result,Bound.Bimg);
	
			//cvSmooth(Bound.Bimg, Bound.Bimg, CV_MEDIAN, 3, 3);//Smooth image
			// cvDilate(Bound.Bimg,Bound.Bimg, NULL, 1);
			// cvErode(Bound.Bimg,Bound.Bimg, NULL, 3);
			// cvSmooth( Bound.Bimg,  Bound.Bimg, CV_MEDIAN, 3, 3);
			// FindObstacleBoundary(Bound.Bimg);
			// if (p.disp_img){ DISPLAY_IMAGE_XY(p.refresh, Bound.Bimg, 1 , 4); //cvShowImage(OBSTACLE_BOUND, Bound.Bimg);
	
			// }
			// cvCopy(Bound.Bimg, bin_img);
	
			// //UpdateParams(Bound.Bimg, StatsPtr, FPtr, p.dynamic);
	
			// ExtractBoundary(FrameSize, BoundPtr);
			// CalculateDistances(FrameSize, BoundPtr, p.camera, p.bot);
	
			// InterpretDepthArray(FrameSize, BoundPtr, p.bot);
			// CV_TIMER_STOP(J, "Path analysis")
	
			// Contours(F.bin_class_result); //Find contours
			// DrawContours(contour[0], CV_RGB( 0, 150, 0 ), SafeRegion); //Draw contours
			// Contours(Bound.Bimg); //Find contours
			// DrawContours(contour[1], CV_RGB( 255, 0, 0 ), SafeRegion); //Draw contours
	
			// cvAddWeighted( contour[0], 0.5, contour[1], 1, 0, contour[0] );
			// cvAddWeighted( contour[0], 0.5, source_img, 1, 0, source_img );
			// // //checking for memory leaks
			// DISPLAY_IMAGE(source_img)
			//cvNamedWindow("test");
			//cvShowImage("test", source_img);
	
	
	
			// if(frame_no%25 == 0 && frame_no>0){
			//     frame_no=j;
			//         char buffer1 [150];
			//         char buffer2 [150];
			//         //char buffer3 [150];
			//         //cvNamedWindow("result");
			//         //cvShowImage("result", result_img);
			//     sprintf(buffer1,"results/%s/%d/%s_source_%d.bmp", name[id-1],T,name[id-1],frame_no);
			//     sprintf(buffer2,"results/%s/%d/%s_result_%d.bmp", name[id-1],T,name[id-1],frame_no);
			//     //sprintf(buffer3,"results/%s/%d/%d/%s_bound_%d_%d.bmp", name[id-1],T,frame_no,name[id-1],frame_no,run);
			//     cvSaveImage(buffer1, source_img);
			//     cvSaveImage(buffer2, result_img);
			//     //cvSaveImage(buffer3, bin_img);
			// }
	
			// ReleaseContours();
	
			////cvReleaseImage(&input_image);
	
	
			// frame_no++;
			// CV_TIMER_STOP(K, "End of Loop")
			// if(p.verb){ printf("\n\n"); }
	
			// //}
			// //printf("key = %d\n", key);
	
			// if(!p.debug) 	key = cvWaitKey(10);
			// else 		key = cvWaitKey(p.debug_delay_ms);
	
			//j++; //loop through image sequences automatically
			//if(j>100) finished=1;
			//if(j>Im_ID_max) finished=1;
			//if(j>Im_ID_max) j=0;
			//static int delay = 10;
			//cvWaitKey(delay);
			//	cvCreateTrackbar("D", POST_RATIO, &delay, 1500, NULL );
			//	cvCreateTrackbar("I", POST_RATIO, &j, Im_ID_max, NULL );
			//}// end if !finished
			if(key == 'q' || key == 'x' || key == 1048689 || key == 1048603)
			break;
			p.refresh=0;
	//        cv::waitKey(0);
		}
	
	
		//}// end for T
		//} //end for id
		//release_images
	//    comm_thread.join();
	
		exitProgram(EXIT_SUCCESS);
	
	}//end main
	
	
	
	// double randdouble()
	// {
	// 	return rand()/(double(RAND_MAX)+1);
	// }
	
	// //generates a psuedo-random double between min and max
	// double randdouble(double min, double max)
	// {
	// 	if (min>max)
	// 	{
	// 		return randdouble()*(min-max)+max;
	// 	}
	// 	else
	// 	{
	// 		return randdouble()*(max-min)+min;
	// 	}
	// }
	
	void init_images_main(CvSize S){
	
		safe_region_mask 	= cvCreateImage( S, 8, 1);
		source_img 		= cvCreateImage( S, 8, 3);
		result_img 		= cvCreateImage( S, 8, 3);
		gray_img 		= cvCreateImage( S, 8, 1);
		gbs_img 		= cvCreateImage( S, 8, 1);
		contour[0] 		= cvCreateImage( S, 8, 3);
		contour[1] 		= cvCreateImage( S, 8, 3);
		prior_prob	 	= cvCreateImage( S, 32, 1);
		prior_rgb	 	= cvCreateImage( S, 8, 3);
		prior	 	= cvCreateImage( S, 8, 1);
		bin_img	 	= cvCreateImage( S, 8, 1);
	
	}
