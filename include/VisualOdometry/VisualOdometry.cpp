#include "VisualOdometry.h"

VisualOdometry::VisualOdometry(bool debug)
{
    debug_ = debug;

    // Start windows
    if (debug_)
    {
        // Init windows
        namedWindow("OUTPUT", CV_WINDOW_AUTOSIZE);
        namedWindow("Plot", CV_WINDOW_AUTOSIZE);

        // Open log files
        string path_h = ros::package::getPath("drone_visual_odometry") + "/logs/results_H.txt";
        //string path_h = "/home/lsi/catkin_ws/src/drone/drone_visual_odometry/logs/results_H.txt";
        string path_POSE = ros::package::getPath("drone_visual_odometry") + "/logs/results_POSE.txt";
        //string path_POSE = "/home/lsi/catkin_ws/src/drone/drone_visual_odometry/logs/results_POSE.txt";
        string path_Debug = ros::package::getPath("drone_visual_odometry") + "/logs/debug.txt";
        //string path_Debug = "/home/lsi/catkin_ws/src/drone/drone_visual_odometry/logs/debug.txt";

        this->fileResult_H.open(path_h, ios::out | ios::app | ios::binary);
        this->fileResult_POSE.open(path_POSE, ios::out | ios::app | ios::binary);
        //this->fileDebug.open(path_Debug, ios::out | ios::app | ios::binary);
        // And write headers
        this->fileResult_H << "R00" << "\t" << "R01" << "\t" << "R02" << "\t" << "R10" << "\t" << "R11" << "\t" << "R12" << "\t" << "R20" << "\t" << "R21" << "\t" << "R22" << "\t" << "Tx" << "\t" << "Ty" << "\t" << "Tz" << endl;
        this->fileResult_POSE << "OldPosX" << "\t" << "CrntPosX" << "\t" << "OldPosY" << "\t" << "CrntPosY" << "\t" << "OldPosZ" << "\t" << "CrntPosZ" << "\t" << "Roll" << "\t" << "Pitch" << "\t" << "Yaw" << "\t" << "Acumtheta" << endl;
        //this->fileDebug << "lastKeypoints\tnewKeypoints\tlastPnt\tnewPnt" << endl;
    }

    // Initialize calibration parameters
    //Kinect2 Camera
    //this->matCamCalibCV = (Mat_<double>(3, 3) << 1.0452185559613902e+03, 0000.00000, 9.8401361827405481e+02, 0000.00000, 1.0469337382811082e+03, 5.4313859532209221e+02, 0000.00000, 0000.00000, 0001.00000);
    //this->invmatCamCalib = (Mat_<double>(3, 3) << 0.000957, 0.000000, -0.941443, 0.000000, 0.000955, -0.518790, 0.000000, 0.000000, 1.000000);
    //this->distCoeffs = (Mat_<double>(1, 5) << 2.7681826911914333e-04, 1.4958095817401962e-01, 2.7810933607229344e-04, 4.4391612573540249e-03, -3.2725416008790553e-01); //0.00279   0.00387   0.00034   0.00033  0.00000 
    //SJ4000 Camera
    this->matCamCalibCV = (Mat_<double>(3, 3) << 0376.39840, 0000.00000, 308.63302, 0000.00000, 0376.42829, 238.26719, 0000.00000, 0000.00000, 0001.00000);
    this->invmatCamCalib = (Mat_<double>(3, 3) << 0.002657, 0.000000, -0.819964, 0.000000, 0.002657, -0.632968, 0.000000, 0.000000, 1.000000);
    this->distCoeffs = (Mat_<double>(1, 5) << -0.31780, 0.10674, 0.00010, -0.00033, 0.00000); //0.00279   0.00387   0.00034   0.00033  0.00000

    // Distances
    this->wall_distance = 2000;
    this->TotalDistance = 0;
    // Initial Yaw
    this->CamYaw = 0;
    // Position
    this->Xacc = 0;
    this->Yacc = 0;
    this->Zacc = 0;
    this->OldXacc = 0;
    this->OldYacc = 0;
    this->OldZacc = 0;

    this->MatRTInitial = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, this->wall_distance);
    this->InitialHomographyWorld = this->matCamCalibCV * this->MatRTInitial;

}

VisualOdometry::~VisualOdometry()
{
    join_threads();

}

double VisualOdometry::round_to_digits(double value, int digits)
{
    if (value == 0.0) // otherwise it will return 'nan' due to the log10() of zero
        return 0.0;

    double factor = pow(10.0, digits - ceil(log10(fabs(value))));
    return round(value * factor) / factor;
}

void VisualOdometry::join_threads()
{
    for (int i = 0; i < this->threads.size(); ++i)
    {
        this->threads[i].join();
    }
    this->threads.clear();
}


// Initiazlize the algorithm with the first frame
void VisualOdometry::init_frame(Mat& frame)
{
    Mat undistortedFrame = frame.clone();
    //undistort(frame, undistortedFrame, this->matCamCalibCV, this->distCoeffs, this->matCamCalibCV);
    //resize(undistortedFrame, undistortedFrame, Size(640, 480));

    this->threads.push_back(thread([&] {this->IMG_OUT = undistortedFrame.clone(); }));
    this->threads.push_back(thread([&] {this->last_frame_c = undistortedFrame.clone(); }));
    this->threads.push_back(thread([&] {cvtColor(undistortedFrame, this->last_frame, COLOR_BGR2GRAY); }));
    join_threads();


    //CONVERTIR MAT IMAGES A GPUMAT IMAGES
    GpuMat last_framegpu;
    last_framegpu.upload(last_frame);

    // detecting keypoints & computing descriptors
    GpuMat last_keypointsGPU;

    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());


    SURF_CUDA surf;
    surf(last_framegpu, GpuMat(), last_keypointsGPU, this->last_descriptorsGPU);

    // downloading results

    surf.downloadKeypoints(last_keypointsGPU, this->last_keypoints);

    cout << "FOUND " << last_keypointsGPU.cols << " keypoints on first image" << endl;

    //Ptr<SIFT> SIFTdetector = SIFT::create(0, 4, 0.04, 9.0, 1.4);
    //ParallelDetectFeatures(this->last_frame, this->last_keypoints, 1000, 8, 8);
    //ParallelComuteDescriptors(this->last_frame, this->last_keypoints, SIFTdetector, this->last_descriptor);
}


// Algorithm iteration
void VisualOdometry::compute_next(Mat& frame)
{

    clock_t t_start = clock();
    Mat undistortedFrame = frame.clone();
    //undistort(frame, undistortedFrame, this->matCamCalibCV, this->distCoeffs, this->matCamCalibCV);

    Mat new_frame_c = undistortedFrame.clone();
    //resize(new_frame_c, new_frame_c, Size(640, 480));

    // we work with grayscale images
    Mat new_frame;
    cvtColor(undistortedFrame, new_frame, COLOR_BGR2GRAY);
    clock_t t_end = clock();
    double elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tinit time: " << elapsed_secs << endl;

    //frame_i-1, frame_i
    clock_t begin = clock();

    // Detect new features
    //vector<KeyPoint> new_keypoints;
    //Mat new_descriptor;
    //Ptr<SIFT> SIFTdetector = SIFT::create(0, 4, 0.04, 9.0, 1.4);


    t_start = clock();

    //CONVERTIR MAT IMAGES A GPUMAT IMAGES
    GpuMat new_framegpu;
    new_framegpu.upload(new_frame);

    // detecting keypoints & computing descriptors
    GpuMat new_keypointsGPU, last_keypointsGPU;
    GpuMat new_descriptorsGPU;

    SURF_CUDA surf;
    surf(new_framegpu, GpuMat(), new_keypointsGPU, new_descriptorsGPU);

    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;

    if (debug_) cout << "\t detect and descriptor time: " << elapsed_secs << endl;
    if (debug_) cout << "\t New keypoints: " << new_keypointsGPU.cols << endl;

    // Match
    t_start = clock();
    // vector<Point2f> last_pntImage, new_pntImage;
    // featureMatching(this->last_keypoints, new_keypoints, this->last_descriptor, new_descriptor, this->last_frame.size().width, this->last_frame.size().height, last_pntImage, new_pntImage);
    
    // matching descriptors
    Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(surf.defaultNorm());
    vector<DMatch> matches;
    matcher->match(this->last_descriptorsGPU, new_descriptorsGPU, matches);
    
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    
    if (debug_) cout << "\tmatch time: " << elapsed_secs << endl;

    
    // downloading results
    vector<KeyPoint> new_keypoints;

    vector<float> new_descriptors;

    surf.downloadKeypoints(new_keypointsGPU, new_keypoints);
    surf.downloadDescriptors(new_descriptorsGPU, new_descriptors);

    this->last_descriptorsGPU = new_descriptorsGPU.clone();

    //¿GOOD MATCHES DISTANCE???

    // draw the results
    //Mat img_matches;
    //drawMatches(Mat(img1), keypoints1, Mat(img2), keypoints2, matches, img_matches);
    //imwrite("seaman_result.jpg", img_matches);

    //namedWindow("matches", 0);
    //imshow("matches", img_matches);
	

    //Single thread detection: 20-30ms slower, 4 times more keypoints 
    // cout << "--------------------------------_" << endl;
    // t_start = clock();
    // Ptr<SIFT> DetectorFeature = SIFT::create(0, 4, 0.04, 9.0, 1.4);
    // DetectorFeature->detect(new_frame, new_keypoints);
    // t_end = clock();
    // elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    // cout << "\tdetect time: " << elapsed_secs << endl;
    // cout << "\t New keypoints: " << new_keypoints.size() << endl;
    
    //ParallelDetectFeatures(new_frame, new_keypoints, 1000, 8, 8);

    //t_start = clock();
    // ParallelComuteDescriptors(new_frame, new_keypoints, SIFTdetector, new_descriptor);
    // t_end = clock();
    // elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    // if (debug_) cout << "\tcompute time: " << elapsed_secs << endl;
    
    // Match
    // t_start = clock();
    // vector<Point2f> last_pntImage, new_pntImage;
    // featureMatching(this->last_keypoints, new_keypoints, this->last_descriptor, new_descriptor, this->last_frame.size().width, this->last_frame.size().height, last_pntImage, new_pntImage);
    // t_end = clock();
    // elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    
    // if (debug_) cout << "\tmatch time: " << elapsed_secs << endl;

    // Store match points fordebug
    //this->fileDebug << last_keypoints.size() << "\t" << new_keypoints.size() << "\t" << last_pntImage.size() << "\t" << new_pntImage.size() << endl;


    // HOMOGRAPHY FRAME TO FRAME
    t_start = clock();

    //Coordinates of the points in the original and target plane
    vector<Point2f> last_pntImage, new_pntImage;
    for (int i = 0; i < matches.size(); i++) {
    		last_pntImage.push_back(this->last_keypoints[matches[i].queryIdx].pt);
    		new_pntImage.push_back(new_keypoints[matches[i].trainIdx].pt);
    }

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
         cout << "F2FHomography = " << endl << " " << F2FHomography << endl << endl;
    }
    //cout << "\tPoints 1: " << last_pntImage.size() << "\t" << "Points 2: " << new_pntImage.size() << endl;
    //cout << "H: " << F2FHomography << endl;

/* 
    SurfMatching ej = SurfMatching();
    Mat F2FHomography  = ej.findHomographySurf(this->last_frame,new_frame);
    cout<<"H: "<<F2FHomography<<endl; */


    Mat FinalHomography = Mat_<double>(3, 3);
    // Calculate R Matrix and T Vector from Homography matrix
    // 1- Extract Homography
    if (F2FHomography.data == NULL || F2FHomography.empty())
    {
        FinalHomography = Mat_<double>::zeros(3, 3);
    }
    else
    {
        for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
        {
            FinalHomography.ptr<double>(r)[c] = round_to_digits(F2FHomography.at<double>(r, c), 6);
        }
    }

    //iHw=iHi-1 * i-1Hw
    Mat InstantHomographyWorld = FinalHomography*this->InitialHomographyWorld;
    //cout << "Inst H: " << InstantHomographyWorld << endl << endl;
    //fileDebug << InstantHomographyWorld.at<double>(0, 0) << "\t" << InstantHomographyWorld.at<double>(0, 1) << "\t" << InstantHomographyWorld.at<double>(0, 2)
    //    << "\t" << InstantHomographyWorld.at<double>(1, 0) << "\t" << InstantHomographyWorld.at<double>(1, 1) << "\t" << InstantHomographyWorld.at<double>(1, 2) << "\t"
    //    << InstantHomographyWorld.at<double>(2, 0) << "\t" << InstantHomographyWorld.at<double>(2, 1) << "\t" << InstantHomographyWorld.at<double>(2, 2) << endl;
    
    //cv::multiply(F2FHomography, InitialHomographyWorld, InstantHomographyWorld, 1, 1);

    Vec3d H1, H2, H3;
    this->threads.push_back(thread([&] {H1 = InstantHomographyWorld.col(0).clone(); }));
    this->threads.push_back(thread([&] {H2 = InstantHomographyWorld.col(1).clone(); }));
    this->threads.push_back(thread([&] {H3 = InstantHomographyWorld.col(2).clone(); }));
    join_threads();

    //cout << H1 << endl << endl;
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tHomography time: " << elapsed_secs << endl;



    // Calculate lambda
    t_start = clock();
    double lambda;
    Mat AAA = Mat_<double>(3, 1), BBB = Mat_<double>(3, 1);
    AAA = this->invmatCamCalib * Mat(H1); //lambda=1 / ||K^-1 * H1||
    BBB = this->invmatCamCalib * Mat(H2);//lambda=1 / ||K^-1 * H2||
    //cout << AAA << "\t" << BBB << endl << endl;
    if (AAA.at<double>(0, 0) == 0 && AAA.at<double>(1, 0) == 0 && AAA.at<double>(2, 0) == 0)
    {
        lambda = 1;
    }
    else
    {
        double A1, A2;
        A1 = cv::norm(AAA);// sqrt(pow(AAA.at<double>(0, 0), 2) + pow(AAA.at<double>(1, 0), 2) + pow(AAA.at<double>(2, 0), 2));
        A2 = cv::norm(BBB);// sqrt(pow(BBB.at<double>(0, 0), 2) + pow(BBB.at<double>(1, 0), 2) + pow(BBB.at<double>(2, 0), 2));
        lambda = 1 / (0.5 * (A1 + A2));//;AAA.Norm(AAA);
        //cout << "A1: " << A1 << "\t" << "A2: " << A2 << "\t" << "lambda: " << lambda << endl << endl;
    }
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tlambda time: " << elapsed_secs << endl;

    // 2- Calculate R,T
    t_start = clock();
    Mat P = Mat_<double>(3, 3);
    //Mat TTEEMM;

    Vec3d R1_m, R2_m, R3_m, T_m;

    P = lambda * (this->invmatCamCalib * InstantHomographyWorld); // RT = lambda * K-1 * iHw 
    
    //cout << "P: " << P << endl;

    R1_m = P.col(0).clone();
    R2_m = P.col(1).clone();
    T_m = P.col(2).clone();

    R3_m = R1_m.cross(R2_m);

    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tR,T time: " << elapsed_secs << endl;

    
    //cout << "R1_m \t" << R1_m << endl << endl;
    //cout << "R2_m \t" << R2_m << endl << endl;
    //cout << "R3_m \t" << R3_m << endl << endl;
    //cout << "T_m \t" << T_m << endl << endl;
    

    // 3- SVD(R)
    t_start = clock();

    Mat matRsvd = Mat_<double>(3, 3);

    threads.push_back(thread([&] {matRsvd.ptr<double>(0)[0] = R1_m[0]; }));
    threads.push_back(thread([&] {matRsvd.ptr<double>(0)[1] = R2_m[0]; }));
    threads.push_back(thread([&] {matRsvd.ptr<double>(0)[2] = R3_m[0]; }));

    threads.push_back(thread([&] {matRsvd.ptr<double>(1)[0] = R1_m[1]; }));
    threads.push_back(thread([&] {matRsvd.ptr<double>(1)[1] = R2_m[1]; }));
    threads.push_back(thread([&] {matRsvd.ptr<double>(1)[2] = R3_m[1]; }));

    threads.push_back(thread([&] {matRsvd.ptr<double>(2)[0] = R1_m[2]; }));
    threads.push_back(thread([&] {matRsvd.ptr<double>(2)[1] = R2_m[2]; }));
    threads.push_back(thread([&] {matRsvd.ptr<double>(2)[2] = R3_m[2]; }));
    join_threads();


    Mat U, V, s;
    SVD::compute(matRsvd, s, U, V);
    //cout << "RSVD:" << matRsvd << endl << "U: " << U << endl << "s: " << s << endl << "V: " << V << endl;
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tSVD time: " << elapsed_secs << endl;

    // 4- Calculate New R
    Mat Rnew = Mat_<double>(3, 3);
    Rnew = U * Mat_<double>::eye(3, 3) * V; //V.t();
    //cout << "Rnew: " << Rnew << endl << endl;

    // 5- Calculate Motion
    t_start = clock();
    double VisPitch, VisRoll, VisYaw;

    // Euler Angles
    VisPitch = round_to_digits(atan2(Rnew.at<double>(1, 2), Rnew.at<double>(2, 2)), 5); //Instant Pitch Angle
    double c2 = sqrt(pow(Rnew.at<double>(0, 0), 2) + pow(Rnew.at<double>(0, 1), 2));
    VisRoll = round_to_digits(atan2(-Rnew.at<double>(0, 2), c2), 5); //Instant Roll Angle
    double s1 = sin(VisPitch);
    double c1 = cos(VisPitch);
    VisYaw = (-1) * round_to_digits(atan2(((s1 * Rnew.at<double>(2, 0)) - (c1 * Rnew.at<double>(1, 0))), ((c1 * Rnew.at<double>(1, 1)) - (s1 * Rnew.at<double>(2, 1)))), 5); //Instant Yaw Angle
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tmotion time: " << elapsed_secs << endl;

    // Kalman Filter
    t_start = clock();
    double CamTx = 0, CamTy = 0, CamTz = 0, InsHomRoll = 0, InsHomPitch = 0, InsHomYaw = 0, Acumtheta = 0;

    threads.push_back(thread([&] { CamTy = (-1) * Xupdate(T_m[0]); }));
    threads.push_back(thread([&] { CamTx = Yupdate(T_m[1]); }));
    threads.push_back(thread([&] { CamTz = Zupdate(T_m[2]); }));
    threads.push_back(thread([&] { InsHomRoll = Rupdate(VisRoll); }));
    threads.push_back(thread([&] { InsHomPitch = Pupdate(VisPitch); }));
    join_threads();

    if (abs(CamTx) < 0.8) { CamTx = 0; }
    if (abs(CamTy) < 0.8) { CamTy = 0; }

    InsHomYaw = VisYaw;

    this->CamYaw = round_to_digits((this->CamYaw + InsHomYaw), 3);//Accum. theta
    
    /* OLD: Not used because IniAng is 0 and CamYaw is also init to 0, so same result.
    if (intProcessIteration == 1) CamYaw = IniAng + round_to_digits(InsHomYaw, 3);
    else CamYaw = round_to_digits((CamYaw + InsHomYaw), 3);//Accum. theta
    */

    //if (CamYaw > MaxAng)
    //{
    //    CamYaw = Math.Round((CamYaw - MaxAng), 3);
    //}

    Acumtheta = Yawupdate(this->CamYaw);
    if (debug_) cout << "CamYaw: " << this->CamYaw << endl;
    if (debug_) cout << "Acumtheta: " << Acumtheta << endl;
    /*
    * Rnew[0,0]    Rnew[0,1]    Rnew[0,2]    TT1[0,2]
    * Rnew[1,0]    Rnew[1,1]    Rnew[1,2]    TT1[0,2]
    * Rnew[2,0]    Rnew[2,1]    Rnew[2,2]    TT1[0,2]
    */

    threads.push_back(thread([&] { this->Xacc = XPosupdate((CamTx * cos(Acumtheta)) - (CamTy * sin(Acumtheta)) + (this->OldXacc)); }));
    threads.push_back(thread([&] { this->Yacc = YPosupdate((CamTx * sin(Acumtheta)) + (CamTy * cos(Acumtheta)) + (this->OldYacc)); }));
    threads.push_back(thread([&] { this->Zacc = (0) + (0) + (T_m[2]); }));
    join_threads();

    this->TotalDistance += sqrt(pow((this->Xacc - this->OldXacc), 2) + pow((this->Yacc - this->OldYacc), 2)) / 1000;
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\tkalman time: " << elapsed_secs << endl;
    if (debug_) cout << "\tTotalDistance: " << TotalDistance << endl;
    // Draw Path
    //plotPath(plotArea, maxRows, OldXacc, OldYacc, Xacc, Yacc);
    
    if (debug_) dynamicPlotPath(plotArea, maxRows, round_to_digits(this->Xacc / 10, 2), round_to_digits(this->Yacc / 10, 2), this->TotalDistance);
    
    //dynamicPlotPath1(plotArea, maxRows, Xacc, Yacc, OldXacc, OldYacc);

    //          int x = int(t_f.at<double>(0)) + 300;
    //          int y = int(t_f.at<double>(2)) + 100;
    /*circle(traj, Point(Xacc + 300, Yacc + 100), 1, CV_RGB(0, 255, 0), 2);
    line(traj, Point(Xacc + 300, Yacc + 100), Point(OldXacc + 300, OldYacc + 100), Scalar(255, 0, 0), 2, 4);*/
    ////
    //          rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
    //          sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    //          putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);






    t_start = clock();
    this->last_frame_c.copySize(this->IMG_OUT);
    addWeighted(this->last_frame_c, 0.5, new_frame_c, 0.5, 0.0, this->IMG_OUT, -1);

    for (int i = 0; i < last_pntImage.size(); i++)
    {
        circle(this->IMG_OUT, last_pntImage[i], 2, Scalar(0, 255, 0), 2);
        circle(this->IMG_OUT, new_pntImage[i], 2, Scalar(0, 0, 255), 2);
        line(this->IMG_OUT, last_pntImage[i], new_pntImage[i], Scalar(255, 0, 0), 2, 4);
    }
    t_end = clock();
    elapsed_secs = double(t_end - t_start) / CLOCKS_PER_SEC;
    if (debug_) cout << "\t draw time: " << elapsed_secs << endl;

    clock_t end = clock();
    elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    if (debug_) cout << "Total time: " << elapsed_secs << "s" << endl;
    
    //imshow("AA", distorImg1out);
    //imshow("BB", distorImg2out);
    if (debug_) imshow("OUTPUT", this->IMG_OUT);
    



    //Actualizacion frames
    this->last_frame_c = new_frame_c.clone();
    this->last_frame = new_frame.clone();
    this->last_keypoints = new_keypoints;

    //Write to log files
    if (debug_) 
    {
        this->fileResult_H << Rnew.at<double>(0, 0) << "\t" << Rnew.at<double>(0, 1) << "\t" << Rnew.at<double>(0, 2)
            << "\t" << Rnew.at<double>(1, 0) << "\t" << Rnew.at<double>(1, 1) << "\t" << Rnew.at<double>(1, 2) << "\t"
            << Rnew.at<double>(2, 0) << "\t" << Rnew.at<double>(2, 1) << "\t" << Rnew.at<double>(2, 2) << "\t"
            << T_m[0] << "\t" << T_m[1] << "\t" << T_m[2] << endl;
        this->fileResult_POSE << this->OldXacc << "\t" << this->Xacc << "\t" << this->OldYacc << "\t" << this->Yacc << "\t" << this->OldZacc << "\t" << Zacc << "\t"
            << InsHomRoll << "\t" << InsHomPitch << "\t" << InsHomYaw << "\t" << Acumtheta << endl;

        this->OldXacc = this->Xacc;
        this->OldYacc = this->Yacc;
    }

    surf.releaseMemory();
	matcher.release();
	new_framegpu.release();
}


// Update distance to wall
void VisualOdometry::set_wall_distance(double distance)
{
    this->wall_distance = distance;
    
    // Update Homography matrix here, so it is only computed when the wall distance changes
    this->MatRTInitial = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, this->wall_distance);
    this->InitialHomographyWorld = this->matCamCalibCV * this->MatRTInitial;
}

// Get current x coordinate
double VisualOdometry::get_x()
{
    return this->Xacc;
}

// Get current y coordinate
double VisualOdometry::get_y()
{
    return this->Yacc;
}

// Get current z coordinate
double VisualOdometry::get_z()
{
    return this->wall_distance;
}

// Get current Yaw
double VisualOdometry::get_yaw()
{
    return this->CamYaw;
}


