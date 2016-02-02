#include "gripper_detector/gripper_detector.h"

#include <visp/vpDetectorQRCode.h>
#include <visp/vpImageIo.h>

// rgbd
#include <rgbd/Client.h>
#include <rgbd/View.h>

// visp includes
#include <visp/vpIoTools.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpMbEdgeKltTracker.h>
#include <visp/vpKeyPoint.h>
#include <visp/vpKltOpencv.h>
#include <visp_bridge/3dpose.h>
#include <visp/vpImageConvert.h>

// detectors
#include <visp/vpDetectorDataMatrixCode.h>
#include <visp/vpDetectorQRCode.h>

//// dot tracker
//#include <visp/vpDot2.h>

gripper_detector::gripper_detector():
    nh_("~"),
    iter_(0),
    tracking_(false)
{
    client_.intialize("top_kinect/rgbd");
    detector_ = new vpDetectorQRCode;
//    detector_ = new vpDetectorDataMatrixCode;
    tracker_ = new vpMbEdgeKltTracker;
    keypoint_learning_ = new vpKeyPoint;
    keypoint_detection_ = new vpKeyPoint;
}

gripper_detector::~gripper_detector()
{
    delete detector_;
    delete tracker_;
    delete keypoint_detection_;
    delete keypoint_learning_;
}

bool gripper_detector::configure()
{
    // Set if display is on or off and if features should be displayed

    // TODO: make configurable
    // TODO: set tracker model

    // TODO: Read marker messages from config and relate them to their grippers

    vpMe me;
    me.setMaskSize(5);
    me.setMaskNumber(180);
    me.setRange(8);
    me.setThreshold(10000);
    me.setMu1(0.5);
    me.setMu2(0.5);
    me.setSampleStep(4);
    me.setNbTotalSample(250);

    tracker_->setMovingEdge(me);
//    tracker_->setMaskBorder(5);

    vpKltOpencv klt_settings;
    klt_settings.setMaxFeatures(300);
    klt_settings.setWindowSize(5);
    klt_settings.setQuality(0.015);
    klt_settings.setMinDistance(8);
    klt_settings.setHarrisFreeParameter(0.01);
    klt_settings.setBlockSize(3);
    klt_settings.setPyramidLevels(3);

//    tracker_->setKltOpencv(klt_settings);

//    cam_.initPersProjWithDistortion();
//    cam_.initPersProjWithoutDistortion(839, 839, 325, 243);
    cam_.initPersProjWithoutDistortion(538.6725257330964, 502.5794530135827, 320, 240);

    std::cout << "Hier 0" << std::endl;

    tracker_->setCameraParameters(cam_);
    tracker_->setAngleAppear( vpMath::rad(70) );
    tracker_->setAngleDisappear( vpMath::rad(80) );
    tracker_->setNearClippingDistance(0.1);
    tracker_->setFarClippingDistance(100.0);
    tracker_->setClipping(tracker_->getClipping() | vpMbtPolygon::FOV_CLIPPING);
    tracker_->setOgreVisibilityTest(false);
    tracker_->setDisplayFeatures(true);

    std::string detectorName = "FAST";
    std::string extractorName = "ORB";
    std::string matcherName = "BruteForce-Hamming";

    keypoint_learning_->setDetector(detectorName);
    keypoint_learning_->setExtractor(extractorName);
    keypoint_learning_->setMatcher(matcherName);

    keypoint_detection_->setDetector(detectorName);
    keypoint_detection_->setExtractor(extractorName);
    keypoint_detection_->setMatcher(matcherName);
    keypoint_detection_->setFilterMatchingType(vpKeyPoint::ratioDistanceThreshold);
    keypoint_detection_->setMatchingRatioThreshold(0.8);
    keypoint_detection_->setUseRansacVVS(true);
    keypoint_detection_->setUseRansacConsensusPercentage(true);
    keypoint_detection_->setRansacConsensusPercentage(20.0);
    keypoint_detection_->setRansacIteration(200);
    keypoint_detection_->setRansacThreshold(0.005);


    if ( vpIoTools::checkFilename("teabox_learning_data.bin") )
    {
        keypoint_detection_->loadLearningData("teabox_learning_data.bin", true);
        tracking_ = true;
    }

    return true;
}

void gripper_detector::update()
{
    rgbd::Image image;

    double error;
    bool click_done = false;

    if (client_.nextImage(image))
    {
        vpImageConvert::convert(image.getRGBImage(),vp_image_gray_);

        if( !d_.isInitialised() )
            d_.init(vp_image_gray_);

        vpDisplay::setTitle(vp_image_gray_,"title");
        vpDisplay::display(vp_image_gray_);
        vpHomogeneousMatrix vp_pose;


        if ( tracking_ )
        {

            if (false)
            {
                try
                {
                    tracker_->track(vp_image_gray_);
                    tracker_->getPose(vp_pose);
                    tracker_->getCameraParameters(cam_);

                    geometry_msgs::Pose pose = visp_bridge::toGeometryMsgsPose(vp_pose);

                    std::cout << "Pose: " << pose << std::endl;

                    tracker_->display(vp_image_gray_,vp_pose,cam_,vpColor::darkRed, 2);
                    vpDisplay::displayFrame(vp_image_gray_, vp_pose, cam_, 0.025, vpColor::none, 3);
                    vpDisplay::flush(vp_image_gray_);
                }
                catch ( std::exception e )
                {
                    std::cout << "ERROR!, lost tracking!" << std::endl;
                    tracking_ = false;
                }
            }
            else
            {

                double elapsedTime;
                if ( keypoint_detection_->matchPoint(vp_image_gray_, cam_, vp_pose, error, elapsedTime) )
                {
                    tracker_->setPose(vp_image_gray_, vp_pose);
                    tracker_->display(vp_image_gray_, vp_pose, cam_, vpColor::red, 2);
                    vpDisplay::displayFrame(vp_image_gray_, vp_pose, cam_, 0.025, vpColor::none, 3);
                    vpDisplay::flush(vp_image_gray_);
                    vpDisplay::getClick(vp_image_gray_);
                }
            }

        }
        else
        {
//            std::vector<vpPoint> model_points(4);
//            model_points[0].setWorldCoordinates(-0.026, -0.026, 0);
//            model_points[1].setWorldCoordinates( 0.026, -0.026, 0);
//            model_points[2].setWorldCoordinates( 0.026,  0.026, 0);
//            model_points[3].setWorldCoordinates(-0.026,  0.026, 0);

//            tracker_->loadModel("left.cao");
//            tracker_->initClick( vp_image_gray_,model_points);
//            tracking_ = true;
            // Marker is still to be detected, so do that
            if ( detector_->detect(vp_image_gray_) )
            {
                std::string message;
                // Go through the detected markers, and get the correct one
                for ( int i = 0; i < detector_->getNbObjects() ; ++i )
                {
                    std::vector<vpImagePoint> bbox = detector_->getPolygon(i);
                    vpDisplay::displayPolygon(vp_image_gray_,bbox,vpColor::red);

                    message = detector_->getMessage(i);
                    std::cout << "Point list of marker with message \"" << message << "\":" << std::endl;

//                    std::vector<vpDot2> dot(4);

                    for(unsigned int i = 0; i < bbox.size(); ++i)
                    {
//                        dot[i].initTracking(vp_image_gray_, bbox[i]);
                        std::cout << "\t" << bbox[i] << std::endl;
                    }
                    std::cout << std::endl;

                    tracker_->loadModel(message + ".cao");

                    std::vector<vpPoint> model_points(4);
                    model_points[0].setWorldCoordinates(-0.03, -0.03, 0);
                    model_points[1].setWorldCoordinates( 0.03, -0.03, 0);
                    model_points[2].setWorldCoordinates( 0.03,  0.03, 0);
                    model_points[3].setWorldCoordinates(-0.03,  0.03, 0);

                    std::cout << "model_points: " << std::endl;
                    for (std::vector<vpPoint>::iterator it = model_points.begin(); it!=model_points.end(); ++it )
                        std::cout << it->getWorldCoordinates() << std::endl << std::endl;

                    std::cout << "Initializing tracker..." << std::endl;
                    tracker_->initFromPoints( vp_image_gray_, bbox, model_points );
                    std::cout << "Tracker initialized" << std::endl;
                    tracker_->track(vp_image_gray_);

                    tracking_ = true;

                    std::vector<cv::KeyPoint> trainKeyPoints;
                    double elapsedTime;
                    keypoint_learning_->detect(vp_image_gray_, trainKeyPoints, elapsedTime);

                    std::vector<vpPolygon> polygons;
                    std::vector<std::vector<vpPoint> > roisPt;
                    std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker_->getPolygonFaces(false);
                    polygons = pair.first;
                    roisPt = pair.second;

                    std::vector<cv::Point3f> points3f;
                    tracker_->getPose(vp_pose);
                    vpKeyPoint::compute3DForPointsInPolygons(vp_pose, cam_, trainKeyPoints, polygons, roisPt, points3f);
                    keypoint_learning_->buildReference(vp_image_gray_, trainKeyPoints, points3f);
                    keypoint_learning_->saveLearningData("teabox_learning_data.bin", true);

                    for(std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
                        vpDisplay::displayCross(vp_image_gray_, (int) it->pt.y, (int) it->pt.x, 4, vpColor::red);
                    }
                    vpDisplay::displayText(vp_image_gray_, 10, 10, "Learning step: keypoints are detected on visible teabox faces", vpColor::red);
                    vpDisplay::displayText(vp_image_gray_, 30, 10, "Click to continue with detection...", vpColor::red);
                    vpDisplay::flush(vp_image_gray_);
                    vpDisplay::getClick(vp_image_gray_, true);

                    break;
                }
            }
//            vpDisplay::display(vp_image_gray_);
            vpDisplay::flush(vp_image_gray_);
        }

//            // Show depth image
//            if (image.getDepthImage().data)
//                cv::imshow("depth_original", image.getDepthImage() / 8);

//            // Show rgb image
//            if (image.getRGBImage().data)
//                cv::imshow("rgb_original", image.getRGBImage());

//            // Show combined image
//            if (image.getDepthImage().data && image.getRGBImage().data)
//            {
//                cv::Mat image_hsv;
//                cv::cvtColor(image.getRGBImage(), image_hsv, CV_BGR2HSV);

//                rgbd::View view(image, image_hsv.cols);

//                cv::Mat canvas_hsv(view.getHeight(), view.getWidth(), CV_8UC3, cv::Scalar(0, 0, 0));

//                for(int y = 0; y < view.getHeight(); ++y)
//                {
//                    for(int x = 0; x < view.getWidth(); ++x)
//                    {
//                        float d = view.getDepth(x, y);
//                        if (d == d)
//                        {
//                            cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(y, x);
//                            hsv[2] = 255 - (d / 8 * 255);
//                            canvas_hsv.at<cv::Vec3b>(y, x) = hsv;
//                        }
//                    }
//                }

//                cv::Mat canvas_bgr;
//                cv::cvtColor(canvas_hsv, canvas_bgr, CV_HSV2BGR);
//                cv::imshow("image + depth", canvas_bgr);
//            }

//            cv::waitKey(3);
    }
    else
        std::cout << "No rgbd image" << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_detector");

    std::cout << "Initializing gripper detector..." << std::endl;
    gripper_detector gripperdetector;

    if ( !gripperdetector.configure() )
    {
        std::cout << "Something went wrong configuring the gripper detector!" << std::endl;
        return 1;
    }

    ros::Rate r(30);
    while (ros::ok())
    {
        gripperdetector.update();
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
