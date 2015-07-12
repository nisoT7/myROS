#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<ros/package.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/legacy/legacy.hpp>
#include<opencv2/legacy/compat.hpp>
#include<robocup_perception/akaze/akaze_features.h>
#include<opencv2/nonfree/features2d.hpp>
#include<opencv2/nonfree/nonfree.hpp>

using namespace cv;

#define SINGLE
#define SURF
#define IMAGENUM 3  //読み込む画像枚数

namespace{
    static const std::string OPENCV_WINDOW = "Image_Window";
    bool detect_flag = false;
    std::vector<Point2f> obj_corners(4);
    int imagenum;
    int flag;
}

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:

#ifdef MULTI
    Mat object_img[IMAGENUM];
    Mat gray_image[IMAGENUM];
    std::vector<Mat> trainImgCollection;
    std::vector<std::vector<KeyPoint> > trainPointCollection;
    std::vector<KeyPoint> queryPoints;
    std::vector<Mat> trainDescCollection;
    //Mat trainDescCollection[IMAGENUM];
#endif
    std::string path;
    std::string detect_image;
    std::string detect_object;

#ifdef SINGLE
    //静止画用の宣言
    Mat object_img;
    Mat gray_image;
    std::vector<std::vector<KeyPoint> > keypoint;
    std::vector<KeyPoint> keypoint_0;
    std::vector<KeyPoint> keypoint_1;
    std::vector<KeyPoint> keypoint_2;
#endif

    Mat descriptors[IMAGENUM];
    Mat descriptors_0;


    public:
    ImageConverter() : it_(nh_)
    { 
#ifdef SINGLE
        imagenum = 0;
        flag = 0;
        ImageLoader(imagenum, flag);
#endif

#ifdef MULTI
        Imageloader();
#endif
        image_sub_ = it_.subscribe("/camera/rgb/image_raw",1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("image_converter/output_video",1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

#ifdef SURF
    void computeSURF(Mat image, std::vector<KeyPoint> &keypoint, Mat &descriptors)
    {   
        SurfFeatureDetector detector(50);
        detector.detect(image, keypoint);

        SurfDescriptorExtractor extractor;
        extractor.compute(image, keypoint, descriptors);
    }
#endif

#ifdef AKAZE
    void computeAKAZE(Mat image, std::vector<KeyPoint> &keypoint, Mat &descriptors)
    {   
        Ptr<FeatureDetector> detector = FeatureDetector::create("AKAZE");
        detector->detect(image, keypoint);

        Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("AKAZE");
        extractor->compute(image, keypoint, descriptors);

    }

    void drawAKAZEKeypoint(Mat *image, 
            const std::vector<KeyPoint> &keypoints,
            const Scalar &color){
        std::vector<KeyPoint>::const_iterator it = keypoints.begin();
        std::vector<KeyPoint>::const_iterator it_end = keypoints.end();
        for(;it != it_end; ++it){
            circle(*image, Point(it->pt.x, it->pt.y),
                    saturate_cast<int>(it->size*0.25),
                    color);
        }
    }
#endif


#ifdef SINGLE
    void imageHandler(int imagenum, int flag)
    {
        cvtColor(object_img, gray_image, CV_BGR2GRAY);
        if(flag == 0){
            computeSURF(gray_image, keypoint_0, descriptors_0);
            //drawAKAZEKeypoint(&object_img, keypoint_0, Scalar(0,255,255));
        }else if(flag == 1){
            computeSURF(gray_image, keypoint_0, descriptors[imagenum]);
            //drawAKAZEKeypoint(&object_img, keypoint_0, Scalar(0,255,255));
        }else if(flag == 2){
            computeSURF(gray_image, keypoint_0, descriptors[imagenum]);
            //drawAKAZEKeypoint(&object_img, keypoint_0, Scalar(0,255,255));
        }else{
            std::cout<<"no image"<<std::endl;
        }
    }
#endif


#ifdef SINGLE
    /*
     * @brief 1枚画像の読み込み
     */

    void ImageLoader(int imagenum, int flag){
        std::stringstream loadfile;
        std::stringstream number;
        loadfile << "image_" << std::setw(3) << std::setfill('0') <<imagenum <<".jpg";
        path = ros::package::getPath("robocup_perception")+"/object_image/";
        detect_image = loadfile.str();
        detect_object = path + detect_image;
        object_img = imread(detect_object);
        if(!object_img.empty()){
            std::cerr<<"Success load image "<< detect_image <<std::endl;
        }else{
            std::cerr<<"Faild load image, load miss image is "<< detect_image <<std::endl;
        }
        imageHandler(imagenum, flag);
    }
#endif

    /*
     * @brief 複数画像の読み込み
     * 実装失敗
     */
#ifdef MULTI
    void Imageloader()
    {
        for(imagenum = 0; imagenum < IMAGENUM; imagenum++){
            std::stringstream loadfile;
            std::stringstream number;
            loadfile << "image_" << std::setw(3) << std::setfill('0') <<imagenum <<".jpg";
            number << imagenum;
            path = ros::package::getPath("robocup_perception")+"/object_image/";
            detect_image = loadfile.str();
            detect_object = path + detect_image;
            object_img[imagenum] = imread(detect_object);
            if(!object_img[imagenum].empty()){
                std::cerr<<"Success load image "<< detect_image <<std::endl;
            }else{
                std::cerr<<"Faild load image, load miss image is "<< detect_image <<std::endl;
            }
        }
    }
#endif


    /*
     * @brief 静止画用ハンドラー
     * @注意 vector型のkeypointは読み込む画像枚数分作成する必要あり
     */

#ifdef MULTI
    void imageHandler()
    {

        for(int i = 0; i < IMAGENUM; i++){
            cvtColor(object_img[i], gray_image[i], CV_RGB2GRAY);
            //             cvtColor(object_img[0], gray_image[0],CV_RGB2GRAY);
            //             cvtColor(object_img[1], gray_image[1],CV_RGB2GRAY);
            //             cvtColor(object_img[2], gray_image[2],CV_RGB2GRAY);
            trainImgCollection.push_back(gray_image[i]);

            //             trainImgCollection.push_back(gray_image[0]);
            //             trainImgCollection.push_back(gray_image[1]);
            //             trainImgCollection.push_back(gray_image[2]);
        }

        //         computeAKAZE(gray_imge[0], keypoint_0, descriptors[0]);
        //         computeAKAZE(gray_imge[1], keypoint_1, descriptors[1]);
        //         computeAKAZE(gray_imge[2], keypoint_2, descriptors[2]);
        //
        //         drawAKAZEKeypoint(&object_img[0], keypoint_0, Scalar(0, 0, 255));
        //         drawAKAZEKeypoint(&object_img[1], keypoint_1, Scalar(0, 0, 255));
        //         drawAKAZEKeypoint(&object_img[2], keypoint_2, Scalar(0, 0, 255));

        Ptr<FeatureDetector> detector = FeatureDetector::create("AKAZE");
        detector->detect(trainImgCollection, trainPointCollection);
        Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("AKAZE");
        extractor->compute(trainImgCollection, trainPointCollection, trainDescCollection);
    }
#endif

    /*
     * @brief コールバック（動画処理の基本呼び出し部分)
     */
    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cap;
        Mat in_img;
        Mat grayframe;
        Mat img_matches;
        Mat cap_descriptors;
        Mat H;
        Mat lastimage;

        std::vector<KeyPoint> cap_keypoint;
        std::vector<std::vector<DMatch> > matches;
        std::vector<DMatch> good_matches;
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        std::vector<Point2f> scene_corners(4);

        //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
        FlannBasedMatcher matcher;

        try{
            cap = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            in_img = cap->image;
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_brige exception: %s", e.what());
            return;
        }

        if(in_img.empty()) {
            std::cerr<<"No capture frame img"<<std::endl;
        }
        else{
            cvtColor(cap->image, grayframe, CV_BGR2GRAY);
            computeSURF(grayframe, cap_keypoint, cap_descriptors);
            //drawAKAZEKeypoint(&cap->image, cap_keypoint, Scalar(0, 0, 255));
            obj_corners[0] = cvPoint(0,0);
            obj_corners[1] = cvPoint( in_img.cols, 0 );
            obj_corners[2] = cvPoint( in_img.cols, in_img.rows );
            obj_corners[3] = cvPoint( 0, in_img.rows );

            std::cout<<imagenum<<std::endl;
            matcher.knnMatch(cap_descriptors, descriptors_0, matches, 2);
            for(int i = 0; i < min(descriptors_0.rows-1,(int) matches.size()); i++){
                if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0)){
                    good_matches.push_back(matches[i][0]);
                }
            }

            drawMatches(grayframe, cap_keypoint, object_img, keypoint_0, good_matches, img_matches);

            if(good_matches.size() >= 4){
                for(int i = 0; i < good_matches.size(); i++){
                    obj.push_back(keypoint_0[good_matches[i].queryIdx].pt);
                    scene.push_back(cap_keypoint[good_matches[i].trainIdx].pt);
                }
            }

            H = findHomography(obj, scene, CV_RANSAC);
            (obj_corners, scene_corners, H);
            line( img_matches, scene_corners[0] + Point2f( object_img.cols, 0), scene_corners[1] + Point2f( object_img.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( object_img.cols, 0), scene_corners[2] + Point2f( object_img.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( object_img.cols, 0), scene_corners[3] + Point2f( object_img.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( object_img.cols, 0), scene_corners[0] + Point2f( object_img.cols, 0), Scalar( 0, 255, 0), 4 );



#ifdef MULTI
            for(int i = 0; i < IMAGENUM; i++){
                matcher->match(cap_descriptors, trainDescCollection[i], matches);
                drawMatches(cap->image, cap_keypoint, trainImgCollection[i], trainPointCollection[i], matches, img_matches);
            }
#endif

        }

        imshow(OPENCV_WINDOW, img_matches);
        waitKey(3);
        image_pub_.publish(cap->toImageMsg());
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_object_detector");
    ImageConverter ic;
    ros::spin();
    return 0;
}
