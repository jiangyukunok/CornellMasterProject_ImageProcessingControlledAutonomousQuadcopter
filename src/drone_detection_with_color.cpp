#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        printf("Failed to initialize.\n");
        return -1;
    }

	// get battery
	 printf("Battery = %d%%\n", ardrone.getBatteryPercentage());

    // Thresholds
    int minH = 0, maxH = 255;
    int minS = 0, maxS = 255;
    int minV = 0, maxV = 255;

    // Create a window
    cvNamedWindow("binalized");
/*    cvCreateTrackbar("H max", "binalized", &maxH, 255);
    cvCreateTrackbar("H min", "binalized", &minH, 255);
    cvCreateTrackbar("S max", "binalized", &maxS, 255);
    cvCreateTrackbar("S min", "binalized", &minS, 255);
    cvCreateTrackbar("V max", "binalized", &maxV, 255);
    cvCreateTrackbar("V min", "binalized", &minV, 255);
    cvResizeWindow("binalized", 0, 0);
*/
	// test index
	//int i = 0;

	// position 
	int px = 0;
	int py = 0;


    // Main loop
    while (1) {
        // Key input
        int key = cvWaitKey(33);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

		// Smooth the image
		cvSmooth(image, image, CV_GAUSSIAN, 3, 3);

        // HSV image
        IplImage *hsv = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
        cvCvtColor(image, hsv, CV_BGR2HSV_FULL);



        // Binalized image
        IplImage *binalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

        // Binalize
/*        CvScalar lower = cvScalar(minH, minS, minV);
		CvScalar upper = cvScalar(maxH, maxS, maxV);
*/
		// orange
		CvScalar lower = cvScalar(0, 100, 170);
		CvScalar upper = cvScalar(18, 169, 255);

        cvInRangeS(hsv, lower, upper, binalized);

        // Show result
        cvShowImage("binalized", binalized);

        // De-noising
		// cvSmooth(binalized, binalized, CV_GAUSSIAN,3,3);
        cvMorphologyEx(binalized, binalized, NULL, NULL, CV_MOP_CLOSE);
 
        // Detect contours
        CvSeq *contour = NULL, *maxContour = NULL;
        CvMemStorage *contourStorage = cvCreateMemStorage();
        cvFindContours(binalized, contourStorage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Find largest contour
        double max_area = 0.0;
        while (contour) {
            double area = fabs(cvContourArea(contour));
            if (area > max_area) {
                maxContour = contour;
                max_area = area;
            }
            contour = contour->h_next;
        }


		// Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

		

        // Object detected
        if (maxContour) {
            // Show result
            CvRect rect = cvBoundingRect(maxContour);
            CvPoint minPoint, maxPoint;
            minPoint.x = rect.x;
            minPoint.y = rect.y;
            maxPoint.x = rect.x + rect.width;
            maxPoint.y = rect.y + rect.height;

			px = minPoint.x + rect.width/2;
			py = minPoint.y + rect.height/2;

			double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
			if ( px > 360 )	vr =  -0.3;//printf("right\n");
			if ( px < 180 )	vr =  0.3;//printf("left\n");
			if ( py > 200 ) vz =  -0.3;//printf("down\n");
			if ( py < 120 )	vz =  0.3;//printf("up\n");
			ardrone.move3D(vx, vy, vz, vr);

			//printf("x = %d\t", (minPoint.x + rect.width/2));
			//printf("y = %d\n\n\n\n", (minPoint.y + rect.height/2));
            
			cvRectangle(image, minPoint, maxPoint, CV_RGB(0,255,0));
        }

/*
		printf("frame index = %d\n\n", i);
		i++;
		if ( i >= 10000 )	i = 0;
*/

        // Release memory
        cvReleaseMemStorage(&contourStorage);

        // Display the image
        cvShowImage("camera", image);

        // Release images
        cvReleaseImage(&hsv);
        cvReleaseImage(&binalized);
    }

    // See you
    ardrone.close();

    return 0;
}