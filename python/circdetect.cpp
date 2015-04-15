#include "opencv/cv.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    CvCapture* cap = cvCaptureFromCAM(CV_CAP_ANY);
    Mat img_1 = imread( "roombatest.jpg" );
    if( !img_1.data )
    { 
        cout<< " --(!) Error reading images " << endl; 
        return -1; 
    }

    //-- Step 1: Detect the keypoints using SURF Detector
    const int kMinHessian = 400;
    SurfFeatureDetector detector( kMinHessian );
    vector<KeyPoint> keypoints_1, keypoints_2;

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_1, descriptors_2;
    extractor.compute( img_1, keypoints_1, descriptors_1 );

    while (true) {
        Mat img_2 = cvQueryFrame(cap);

        detector.detect( img_2, keypoints_2 );
        extractor.compute( img_2, keypoints_2, descriptors_2 );

        //-- Step 3: Matching descriptor vectors using FLANN matcher

        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_1, descriptors_2, matches );

        double max_dist = 0; double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptors_1.rows; i++ )
        { double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        std::vector< DMatch > good_matches;

        for( int i = 0; i < descriptors_1.rows; i++ )
        { 
            if( matches[i].distance <= max(2*min_dist, 0.02) )
            { 
                good_matches.push_back( matches[i]); 
            }
        }

        //-- Draw only "good" matches
        Mat img_matches;
        drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Show detected matches
        imshow( "Roomba Detection", img_matches);

        for( int i = 0; i < (int)good_matches.size(); i++ ) {
            cout << "found match " << endl; 
        }

        if(waitKey(10) >= 0)
            break;
    }
}
