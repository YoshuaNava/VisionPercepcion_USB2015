#include <stdio.h> 
#include "opencv/cv.h" //opencv `C' core functions
#include "opencv/highgui.h"
#include "capture.h"
//#include <gst/gst.h>
#include "init_structures.h"

extern char name[10][30];
extern int id;
extern Params p; 
int frames;

CvCapture *capture = 0; // Structure for getting video from camera or avi
CvVideoWriter *writer;

//Initialize capture from avi
int initVideoCapture()
{

    printf("Capturing from %s\n", p.video_capture_source);
    capture = cvCreateFileCapture(p.video_capture_source);

    frames = (int) cvGetCaptureProperty(
            capture,
            CV_CAP_PROP_FRAME_COUNT
            );
    double fps = cvGetCaptureProperty(
                    capture,
                    CV_CAP_PROP_FPS);
    CvSize size = cvSize(
            (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH),
            (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT)
            );
    p.capture_size = cvSize(
            (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH),
            (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT)
            );
    int isColor = 1;

    // 	writer = cvCreateVideoWriter(
    // 
    // 	"/home/mikesapi/Videos/msc_experiments/Datasets/Cranfield/1_input_video_data/Complex_Scene_Posterior_Ratio.avi",
    // 		CV_FOURCC('P','I','M','1'),
    // 		30,
    // 		size,
    // 		isColor
    // 		);

    printf("no of frames = %d\n", frames);

    if( !frames )
    {
            fprintf(stderr, "failed to initialize camera capture\n");
            return 1;
    }

    return 0;
}

// Initialize capture from camera
int initCapture()
{
    // Initialize video capture
    //capture = cvCaptureFromCAM( CV_CAP_ANY );
    capture = cvCaptureFromCAM(1);
    //capture = cvCaptureFromCAM( 0 );
    if( !capture )
    {
        fprintf(stderr, "failed to initialize camera capture\n");
        return 0;
    }
    //cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, capture_W );
    //cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, capture_H );

    return 1;
}



// closeCapture()
void closeCapture()
{
    // Terminate video capture and free capture resources
    cvReleaseCapture( &capture );
    cvReleaseVideoWriter( &writer );

    return;
}

// Get the next frame from the camera
int NextFrame(IplImage **frame)
{
    *frame = cvQueryFrame( capture );

    if( !*frame ){
        fprintf(stderr, "failed to get a video frame\n");
        //frame_number = frame_number -1;
        return 1;
    }

    return 0;
}

void printResults(float roll[], float yaw[], float pitch[])
{
    FILE *fp;
    int i;

    /* open the file */
    //fp = fopen("C:/Documents and Settings/mikesapi/Desktop/Thesis Material/Ground Truth/ssm/my_ssm7.txt", "w");
    //fp = fopen("C:/Documents and Settings/mikesapi/Desktop/Thesis Material/Ground Truth/jam/my_jam4.txt", "w");
            //fp = fopen("C:/Documents and Settings/mikesapi/Desktop/Thesis Material/Screenshots/me3.txt", "w");
        
    if (fp == NULL) {
        printf("I couldn't open results.dat for writing.\n");
        exit(0);
    }

    /* write to the file */
    for (i=1; i<=frames+1; ++i)
                
        fprintf(fp, "%d,		%0.6f,		%0.6f,			%0.6f\n", i, roll[i], yaw[i], pitch[i]);

    /* close the file */
    fclose(fp);

    return;
}

void writeVideo( IplImage* frame)
{
	cvWriteFrame( writer, frame);
}
