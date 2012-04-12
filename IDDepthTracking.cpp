
 
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <algorithm>
#include <vector>
 
#include <stdio.h>
#include "ARToolKitPlus/TrackerSingleMarkerImpl.h"
 
 
CvScalar target_color[4] = { // in BGR order
		{{   0,   0, 255,   0 }},  // red
		{{   0, 255,   0,   0 }},  // green
		{{ 255,   0,   0,   0 }},  // blue
		{{   0, 255, 255,   0 }}   // yellow
};
ARToolKitPlus::ARMarkerInfo * marker_info;
using namespace std;
using namespace cv;

typedef struct markerinfo {
	int valid;
	float x;
	float y;
} markerinfo;

class MyLogger : public ARToolKitPlus::Logger
{
    void artLog(const char* nStr)
    {
        printf(nStr);
    }
};
void ARstuff() {
	    const bool    useBCH = true;
		IplImage* adaptiveImg;
   marker_info = new ARToolKitPlus::ARMarkerInfo[200];
		
	int unwarped[8][8];

    const int     width = 640, height = 480, bpp = 1,
                  numPixels = width*height*bpp;
    size_t        numBytesRead;

    MyLogger      logger;

    ARToolKitPlus::TrackerSingleMarker *tracker = new ARToolKitPlus::TrackerSingleMarkerImpl<6,6,6,
                                                         ARToolKitPlus::PIXEL_FORMAT_LUM, 1, 180>(width,height);
	

    // set a logger so we can output error messages
    //
    tracker->setLogger(&logger);

    // load a camera file. two types of camera files are supported:
    //  - Std. ARToolKit
    //  - MATLAB Camera Calibration Toolbox
    //tracker->init("data/LogitechPro4000.dat", 1.0f, 1000.0f);   // load std. ARToolKit camera file
    tracker->init("no_distortion.cal", 1.0f, 1000.0f);          // load MATHLAB file

    // define size of the marker
    //tracker->setPatternWidth(80);

    // the marker in the BCH test image has a thiner border...
   // tracker->setBorderWidth(useBCH ? 0.125f : 0.250f);
	tracker->setBorderWidth(0.125f );
    // set a threshold. we could also activate automatic thresholding
    tracker->setThreshold(150);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    // RPP is more robust than ARToolKit's standard pose estimator
    //tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

    // do the OpenGL camera setup
    //glMatrixMode(GL_PROJECTION)
    //glLoadMatrixf(tracker->getProjectionMatrix());
	 CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
   if ( !capture ) {
     fprintf( stderr, "ERROR: capture is NULL \n" );
     getchar();
     return;
   }
   // Create a window in which the captured images will be presented
   cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
   cvNamedWindow( "mywindowb", CV_WINDOW_AUTOSIZE );
   // Show the image captured from the camera in the window and repeat
   markerinfo markers[1000];
   for (int i = 0; i < 1000; i++)
	   markers[i].valid = 0;
   IplImage* grayImage;
   while ( 1 ) {
     // Get one frame
     IplImage* frame = cvQueryFrame( capture );
	 
     if ( !frame ) {
       fprintf( stderr, "ERROR: frame is null...\n" );
       getchar();
       break;
     }
	 grayImage = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );

     cvCvtColor( frame, grayImage, CV_BGR2GRAY );
	    adaptiveImg = cvCreateImage(cvSize(grayImage->width,grayImage->height),IPL_DEPTH_8U, 1);
   cvAdaptiveThreshold( grayImage, adaptiveImg, 255,
                         CV_ADAPTIVE_THRESH_MEAN_C , CV_THRESH_BINARY,
                          15, -4 );
   int marker_num;
   
   tracker->arDetectMarkerLite((unsigned char*)(adaptiveImg->imageData), 160, &marker_info, &marker_num);
   bool assigned[1000];
   for (int i = 0; i < 1000; i++)
	   assigned[i] = false;
   vector<int> unassigned;
   for (int i = 0; i < marker_num; i++) {
	   if (marker_info[i].id != -1 && marker_info[i].id < 1000) {
		   markers[marker_info[i].id].valid = 1;
		   markers[marker_info[i].id].x = marker_info[i].pos[0];
		   markers[marker_info[i].id].y = marker_info[i].pos[1];
		   assigned[marker_info[i].id] = true;
	   }
	   else
		   unassigned.push_back(i);
   }
   cout << marker_info[0].id << ' ';
   while (!unassigned.empty()) {
	   int ind = unassigned.back();
	   unassigned.pop_back();
	   int minID = -1;
	   float minDsq = 100;
	   for (int j = 0; j < 1000; j++) {
		   if (!markers[j].valid || assigned[j])
			   continue;
		   float Dsq = pow(marker_info[ind].pos[0]-markers[j].x,2) + pow(marker_info[ind].pos[1]-markers[j].y,2);
		   if (Dsq < minDsq)
			   minID = j;
	   }
	   if (minID != -1) {
		   marker_info[ind].id = minID;
		   markers[minID].x = marker_info[ind].pos[0];
		   markers[minID].y = marker_info[ind].pos[1];
	   }
   }
   cout << marker_info[0].id << endl;

   if (marker_num == 0)
	   continue;
    for( int i = 0; i < 4; i++) {
			int radius = 480/50;

			cvCircle(grayImage,
					cvPoint((int)(marker_info[0].vertex[i][0] + 0.5f),(int)(marker_info[0].vertex[i][1] + 0.5f)),
					radius,
					target_color[0]);
		}
	
	
     cvShowImage( "mywindow", grayImage);

     if ( (cvWaitKey(10) & 255) == 27 ) break;
   }
   
	system("pause");
   
   cvReleaseCapture( &capture );

   cvDestroyWindow( "mywindow" );

   
    //delete cameraBuffer;
}
 
 
int main(int argc, char *argv[]) {
	while(1)
		ARstuff();
	return 0;
}