/* Compute the odometry using images from a directory */

#include "VisualOdometry/VisualOdometry.h"


int main(int argc, char **argv)
{
    namedWindow("RGB", CV_WINDOW_AUTOSIZE);

    VisualOdometry odom;

   

    int image_count = 0;

    // Grab the first frame and init the odometry first values
    Mat frame;
    frame = imread("/home/roquinha/drone_visual_odometry/src/devel/lib/drone_visual_odometry/OdometryBagfiles/image1.jpg", IMREAD_COLOR);
    if (frame.empty()){
        cout << "No se ha podido leer la primera imagen." << endl;
        return -1;
    }

    odom.init_frame(frame);

   //cout << "Frame: " << (images_dir + to_string(image_count++) + ".jpg") << endl;

    char keypressed = 0;

    while (keypressed != 'q')
    {
        frame = imread("/home/roquinha/drone_visual_odometry/src/devel/lib/drone_visual_odometry/OdometryBagfiles/image2.jpg", IMREAD_COLOR);
         if (frame.empty()){
             cout << "No se ha podido leer la imagen." << endl;
        return -1;
    }
        if (!frame.data)
        {
            break;
        }
        // cout << "frame type: " << frame.type() << endl;
        // cout << "frame dims: " << frame.dims << endl;
        odom.compute_next(frame);
        keypressed = waitKey(1) & 0x00ff;
    }
    destroyAllWindows();
    return 0;
}