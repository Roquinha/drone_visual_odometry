#include "SurfMatching.h"

//Constructor
SurfMatching::SurfMatching(){

}

//Desconstructor
SurfMatching::~SurfMatching()
{

}

// Algorithm iteration
Mat SurfMatching::findHomographySurf(Mat& frame1, Mat& frame2)
{

    // Detect last features
    vector<KeyPoint> last_keypoints;
    Mat last_descriptor;
    Ptr<SIFT> SIFTdetector1 = SIFT::create(0, 4, 0.04, 9.0, 1.4);

    // Detect new features
    vector<KeyPoint> new_keypoints;
    Mat new_descriptor;
    Ptr<SIFT> SIFTdetector2 = SIFT::create(0, 4, 0.04, 9.0, 1.4);

    /* Single thread detection: 20-30ms slower, 4 times more keypoints */
    // cout << "--------------------------------_" << endl;
    // t_start = clock();
    // Ptr<SIFT> DetectorFeature = SIFT::create(0, 4, 0.04, 9.0, 1.4);
    // DetectorFeature->detect(new_frame, new_keypoints);
    // t_end = clock();
    // elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    // cout << "\tdetect time: " << elapsed_secs << endl;
    // cout << "\t New keypoints: " << new_keypoints.size() << endl;

    t_start = clock();
    ParallelDetectFeatures(frame1, last_keypoints, 1000, 8, 8);
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    ParallelDetectFeatures(frame2, new_keypoints, 1000, 8, 8);

    if (debug_) cout << "\t detect time: " << elapsed_secs << endl;
    if (debug_) cout << "\t New keypoints: " << new_keypoints.size() << endl;

    t_start = clock();
    ParallelComuteDescriptors(frame1, last_keypoints, SIFTdetector1, last_descriptor);
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tcompute time: " << elapsed_secs << endl;
    ParallelComuteDescriptors(frame2, new_keypoints, SIFTdetector2, new_descriptor);

    // Match
    t_start = clock();
    vector<Point2f> last_pntImage, new_pntImage;
    featureMatching(this->last_keypoints, new_keypoints, this->last_descriptor, new_descriptor, this->frame2.size().width, this->frame2.size().height, last_pntImage, new_pntImage);
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    
    if (debug_) cout << "\tmatch time: " << elapsed_secs << endl;

    // Store match points fordebug
    //this->fileDebug << last_keypoints.size() << "\t" << new_keypoints.size() << "\t" << last_pntImage.size() << "\t" << new_pntImage.size() << endl;


    t_start = clock();
    Mat F2FHomography;
    if (last_pntImage.size() < 6 || new_pntImage.size() < 6)
    {
        //F2FHomography = NULL;
        //F2FHomography=cv::Mat.eye(3);
        F2FHomography = Mat_<double>::eye(3, 3);
    }
    else
    {
        //F2FHomography = findHomography(last_pntImage, new_pntImage, RANSAC, 6, noArray(), 2000, 0.800);
        F2FHomography = findHomography(last_pntImage, new_pntImage, RANSAC, 6, noArray(), 2000, 0.996);
    }
    //cout << "\tPoints 1: " << last_pntImage.size() << "\t" << "Points 2: " << new_pntImage.size() << endl;
    //cout << "H: " << F2FHomography << endl;

    return F2FHomography;
}


