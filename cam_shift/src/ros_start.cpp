#include "ros_start.h"

static Rect trackWindow;
static int hsize = 16;
static float hranges[] = {0, 180};
static const float *phranges = hranges;
static Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
static bool paused = false;
RotatedRect trackBox;
std_msgs::UInt8 state;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = false;
static Point origin;
static Rect selection;
int vmin , vmax , smin;
Mat image;
static int imgWidth, imgHeight;
static int initDestArea; // Initialized to 1 to avoid DIV by 0 errors

static ros::Publisher robotAngleVar;
static ros::Publisher imagePrStatePub;
static ros::Publisher destAreaPub;
bool success = false;

Ptr<Tracker> tracker = TrackerCSRT::create();
static void onMouse(int event, int x, int y, int, void *)
{
    ROS_INFO("Mouse detected");
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
        initDestArea = selection.area(); // duo
    }

    switch (event)
    {
    case EVENT_LBUTTONDOWN:
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        success = true;
        if (selection.width > 0 && selection.height > 0)
            trackObject = -1;
        break;
    }

}

void trackArea(Rect window)
{
    std_msgs::Float32 destArea;
    destArea.data = (float)window.area() / (imgHeight * imgWidth);
    destAreaPub.publish(destArea);
}

void calcAngle(Point2f destCentre)
{
    std_msgs::Float32 normAngle;
    // If we have started tracking the object
    if (trackObject != 0)
    {
        normAngle.data = (destCentre.x - ((float)imgWidth / 2)) / ((float)imgWidth / 2);
        robotAngleVar.publish(normAngle);
    }
}

void shift(Mat inImg)
{
     if (!paused)
    {
        // cap >> frame;
        if (inImg.empty())
        {
            ROS_INFO("Camera image empty");
            return; // break;
        }
    }

    // Use the input image as the reference
    // Only a shallow copy, so relatively fast
    image = inImg;

    if (!paused)
    {
        // Convert the colour space to HSV
        cvtColor(image, hsv, CV_BGR2HSV);

        // If the destination coordinates have been received, then start the tracking
        // trackObject is set when the destination coordinates have been received
        if (trackObject)
        {
            int _vmin = vmin, _vmax = vmax;

            inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
                    Scalar(180, 256, MAX(_vmin, _vmax)), mask);
            int ch[] = {0, 0};
            hue.create(hsv.size(), hsv.depth());
            mixChannels(&hsv, 1, &hue, 1, ch, 1);

            // Do the following steps only for the first time
            if (trackObject < 0)
            {
                // Publish that we have started tracking
                std_msgs::UInt8 state;          //
                state.data = 1;                 //
                imagePrStatePub.publish(state); //
                // Set the Region of interest and the mask for it
                Mat roi(hue, selection), maskroi(mask, selection);
                // Calculate the histogram of this
                calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                normalize(hist, hist, 0, 255, CV_MINMAX);

                trackWindow = selection;
                trackObject = 1;

                histimg = Scalar::all(0);
                int binW = histimg.cols / hsize;
                Mat buf(1, hsize, CV_8UC3);
                for (int i = 0; i < hsize; i++)
                    buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                cvtColor(buf, buf, CV_HSV2BGR);

                for (int i = 0; i < hsize; i++)
                {
                    int val = saturate_cast<int>(hist.at<float>(i) * histimg.rows / 255);
                    rectangle(histimg, Point(i * binW, histimg.rows),
                              Point((i + 1) * binW, histimg.rows - val),
                              Scalar(buf.at<Vec3b>(i)), -1, 8);
                }
            }

            calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
            backproj &= mask;
            trackBox = CamShift(backproj, trackWindow,
                                TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
            if (trackWindow.area() <= 1)
            {
                // Notify that the destination has been lost
                std_msgs::UInt8 state;          //
                state.data = 2;                 //
                imagePrStatePub.publish(state); //
                ROS_INFO("*********DESTINATION LOST in CAMSHIFT************");
                ROS_INFO("track height %d width %d", trackWindow.height, trackWindow.width);
                trackObject = 0; // //Disable tracking to avoid termination of node due to negative heights TBD
                int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
                trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                   trackWindow.x + r, trackWindow.y + r) &
                              Rect(0, 0, cols, rows);
            }

            if (backprojMode)
                cvtColor(backproj, image, CV_GRAY2BGR);
            
            ellipse( image, trackBox, Scalar(0,0,255), 3, LINE_AA );
        }
    }
    else if (trackObject < 0)
    {
        // If a new destination has been selected stop pausing
        paused = false;
    }

    // Code to display an inverted image of the selected region
    // Remove this in the fall validation expt TBD
    if (selectObject && selection.width > 0 && selection.height > 0)
    {
        Mat roi(image, selection);
        bitwise_not(roi, roi);
    }

    imshow("CamShift Demo", image);

    char c = (char)waitKey(1);
    if (c == 27)
        ROS_INFO("Exit boss"); // break;
    switch (c)
    {
    case 'b':
        backprojMode = !backprojMode; // 黑白图像
        break;
    case 'c': // 刷新，从新取图
        trackObject = 0;
        histimg = Scalar::all(0);
        break;
    case 'p': // 暂停检测或继续
        paused = !paused;
        break;
    default:
        break;
    }
    setMouseCallback("CamShift Demo", onMouse, 0); //
                                                   //   createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
                                                   //   createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
                                                   //   createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );

    // Find the area of the destination and publish it
    trackArea(trackWindow);
    // Find the angle of the destination wrt to the robot and publish that
    calcAngle(trackBox.center);
}



camShift_ros::camShift_ros(ros::NodeHandle nh_) :it(nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    n_p.param<std::string>("image_topic",image_topic,"/rgb/image_raw");
    n_p.param<std::string>("depth_topic",depth_topic,"/depth_ro_rgb/image_raw");
    n_p.param<bool>("mm_to_m",mm_to_meters,false);
    image_sub.subscribe(nh,image_topic,1);
    depth_sub.subscribe(nh,depth_topic,1);
    ts_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, depth_sub);
    ts_sync->registerCallback(boost::bind(&camShift_ros::imageDepthCb, this, _1, _2));

}


void camShift_ros::imageDepthCb(const sensor_msgs::ImageConstPtr& img_msg,const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // Always copy, returning a mutable CvImage
        // OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        // if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg,sensor_msgs::image_encodings::TYPE_32FC1);
        cout << "get depth" <<endl;
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("depth image get error : %s",e.what());
        return;
    }
    shift(cv_ptr->image);

}


// This function is called everytime a new image_info message is published
void camInfoCallback(const sensor_msgs::CameraInfo &camInfoMsg)
{
    // Store the image width for calculation of angle
    imgWidth = camInfoMsg.width;
    imgHeight = camInfoMsg.height;
}

// This function is called everytime a new image is published

/**
 * This is ROS node to track the destination image
 */
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "cam_shift");
    ROS_INFO("-----------------");
   
    ros::NodeHandle nh;
    // Create an ImageTransport instance, initializing it with our NodeHandle.
    camShift_ros cr(nh);
    namedWindow("CamShift Demo", 1);
    ros::Subscriber camInfo = nh.subscribe("/rgb/camera_info", 1, camInfoCallback);     // style="background-color: rgb(255, 255, 0);"//Kinect Topic</span>

    robotAngleVar = nh.advertise<std_msgs::Float32>("robot_angle", 100);
    imagePrStatePub = nh.advertise<std_msgs::UInt8>("improc_state", 10);
    destAreaPub = nh.advertise<std_msgs::Float32>("dest_area", 10);

    state.data = 0;
    imagePrStatePub.publish(state);
    ros::spin();
    // OpenCV HighGUI call to destroy a display window on shut-down.
    // destroyWindow(WINDOW);
    // destroyWindow("Histogram");
    // destroyWindow("CamShift Demo");

    /**
     * In this application all user callbacks will be called from within the ros::spin() call.
     * ros::spin() will not return until the node has been shutdown, either through a call
     * to ros::shutdown() or a Ctrl-C.
     */
    // ros::spin();

    // ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}