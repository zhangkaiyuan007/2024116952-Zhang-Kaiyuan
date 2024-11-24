#include "commen.h"
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

//为了防止有其他装甲板干扰，使得算法可以追踪，加入限幅滤波
filter_type filter(filter_type effective_value, filter_type new_value, filter_type delat_max);
// 为辅助筛选装甲板，提高算法运行速度，做一次筛选预处理
RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode);
//输出ROI
void ROIimg(Mat imgOriginal, int x, int y);
int main()
{
    //植入随机种子，利用彩色图框，更容易察觉识别位置的变化
    srand(time(NULL));

    // 读取相机标定信息
    YAML::Node camera_info = YAML::LoadFile("/home/robert_kaiyuan/鸡架大师/陀螺/camera_info.yaml");

    // 获取相机内参矩阵
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = camera_info["camera_matrix"]["data"][0].as<double>();
    camera_matrix.at<double>(0, 1) = camera_info["camera_matrix"]["data"][1].as<double>();
    camera_matrix.at<double>(0, 2) = camera_info["camera_matrix"]["data"][2].as<double>();
    camera_matrix.at<double>(1, 0) = camera_info["camera_matrix"]["data"][3].as<double>();
    camera_matrix.at<double>(1, 1) = camera_info["camera_matrix"]["data"][4].as<double>();
    camera_matrix.at<double>(1, 2) = camera_info["camera_matrix"]["data"][5].as<double>();
    camera_matrix.at<double>(2, 0) = camera_info["camera_matrix"]["data"][6].as<double>();
    camera_matrix.at<double>(2, 1) = camera_info["camera_matrix"]["data"][7].as<double>();
    camera_matrix.at<double>(2, 2) = camera_info["camera_matrix"]["data"][8].as<double>();

    // 获取畸变系数
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    dist_coeffs.at<double>(0) = camera_info["distortion_coefficients"]["data"][0].as<double>();
    dist_coeffs.at<double>(1) = camera_info["distortion_coefficients"]["data"][1].as<double>();
    dist_coeffs.at<double>(2) = camera_info["distortion_coefficients"]["data"][2].as<double>();
    dist_coeffs.at<double>(3) = camera_info["distortion_coefficients"]["data"][3].as<double>();
    dist_coeffs.at<double>(4) = camera_info["distortion_coefficients"]["data"][4].as<double>();
    
    //打开视频文件
    VideoCapture capture("/home/robert_kaiyuan/鸡架大师/陀螺/output_0.avi");
    while(1)
    {
        //第一步，导入图像
        Mat imgOriginal;
        capture >> imgOriginal;
        if (imgOriginal.empty())
        {
            cout << "Over" << endl;
            return -1;
        }

        //第二步，转HSV图像，重新合并两通道，为了方便后面寻找图像中所有的轮廓
        Mat imgHSV;
        vector<Mat> hsvSplit;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);//将RGB转换为HSV，便于识别
        split(imgHSV, hsvSplit);
        equalizeHist(hsvSplit[2], hsvSplit[2]);
        merge(hsvSplit, imgHSV);//重新合并

        //第三步，图像二值化处理，摒弃环境光干扰
        Mat thresHold;
        threshold(hsvSplit[2],thresHold,240,245,THRESH_BINARY);

        //第四步，模糊/膨胀处理，让图像变得圆润
        blur(thresHold, thresHold, Size(3,3));
        Mat element = getStructuringElement(MORPH_ELLIPSE,Size(3,3));//膨胀
        dilate(thresHold, element, element);
        imshow("二值化", thresHold);
        //第五步，开始寻找轮廓
        vector<RotatedRect> vc;
        vector<RotatedRect> vRec;
        vector<vector<Point>> Light_Contour;// 发现的轮廓
        findContours(element.clone(),Light_Contour, RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

        //第六步，从面积上选轮廓
        for (int i = 0;i < Light_Contour.size();i++)
        {
            //求轮廓面积
            float Light_Contour_Area = contourArea(Light_Contour[i]);
            //去除较小轮廓fitllipse的限制条件
            if (Light_Contour_Area < 15 || Light_Contour[i].size() <= 10)
                continue;
            // 用椭圆拟合区域得到外接矩形
            RotatedRect Light_Rec = fitEllipse(Light_Contour[i]);
            Light_Rec = adjustRec(Light_Rec, ANGLE_TO_UP);

            if (Light_Rec.angle > 10 )
                continue;
            // 长宽比和轮廓面积比限制
            if (Light_Rec.size.width / Light_Rec.size.height > 2
                    || Light_Contour_Area / Light_Rec.size.area() < 0.68)
                continue;
            // 扩大灯柱的面积
            Light_Rec. size.height *= 1.1;
            Light_Rec.size.width *= 1.1;
            vc.push_back(Light_Rec);
        }
        //第七步，从灯条长宽比上来筛选轮廓
        int x,y;
        for (size_t i = 0; i < vc.size(); i++)
        {
            for (size_t j = i + 1; (j < vc.size()); j++)
            {
                //判断是否为相同灯条
                float Contour_angle = abs(vc[i].angle - vc[j].angle); //角度差
                if (Contour_angle >= 7)
                    continue;
                //长度差比率
                float Contour_Len1 = abs(vc[i].size.height - vc[j].size.height) / max(vc[i].size.height, vc[j].size.height);
                //宽度差比率
                float Contour_Len2 = abs(vc[i].size.width - vc[j].size.width) / max(vc[i].size.width, vc[j].size.width);
                if (Contour_Len1 > 0.25 || Contour_Len2 > 0.25)
                    continue;


                RotatedRect ARMOR;
                ARMOR.center.x = (vc[i].center.x + vc[j].center.x) / 2.; //x坐标
                ARMOR.center.y = (vc[i].center.y + vc[j].center.y) / 2.; //y坐标
                ARMOR.angle = (vc[i].angle + vc[j].angle) / 2.; //角度
                float nh, nw, yDiff, xDiff;
                nh = (vc[i].size.height + vc[j].size.height) / 2; //高度
                // 宽度
                nw = sqrt((vc[i].center.x - vc[j].center.x) * (vc[i].center.x - vc[j].center.x) + (vc[i].center.y - vc[j].center.y) * (vc[i].center.y - vc[j].center.y));
                float ratio = nw / nh; //匹配到的装甲板的长宽比
                xDiff = abs(vc[i].center.x - vc[j].center.x) / nh; //x差比率
                yDiff = abs(vc[i].center.y - vc[j].center.y) / nh; //y差比率
                if (ratio < 1.0 || ratio > 5.0 || xDiff < 0.5 || yDiff > 2.0)
                    continue;
                ARMOR.size.height = nh;
                ARMOR.size.width = nw;
                vRec.push_back(ARMOR);
                Point2f point1;
                Point2f point2;
                point1.x=vc[i].center.x;point1.y=vc[i].center.y+30;
                point2.x=vc[j].center.x;point2.y=vc[j].center.y-30;
                int xmidnum = (point1.x+point2.x)/2;
                int ymidnum = (point1.y+point2.y)/2;
                //此时轮廓已筛选完毕，为了方便输出，我们将得到的数据就此输出处理
                ARMOR.center.x = filter(ARMOR.center.x,xmidnum, DELAT_MAX);
                ARMOR.center.y = filter(ARMOR.center.y,ymidnum, DELAT_MAX);
                //随机颜色拟合线
                Scalar color(rand() & 255, rand() & 255, rand() & 255);
                rectangle(imgOriginal, point1,point2, color, 2);//将装甲板框起来

                line(imgOriginal, Point(ARMOR.center.x - 36, ARMOR.center.y - 36),
                    Point(ARMOR.center.x + 36, ARMOR.center.y + 36), Scalar(100, 25, 150), 5);
                line(imgOriginal, Point(ARMOR.center.x + 36, ARMOR.center.y - 36),
                    Point(ARMOR.center.x - 36, ARMOR.center.y + 36), Scalar(100, 25, 150), 5);

                line(imgOriginal, Point(ARMOR.center.x - 10, ARMOR.center.y - 10),
                    Point(ARMOR.center.x + 10, ARMOR.center.y + 10), Scalar(255, 255, 0), 5);
                line(imgOriginal, Point(ARMOR.center.x + 10, ARMOR.center.y - 10),
                    Point(ARMOR.center.x - 10, ARMOR.center.y + 10), Scalar(255, 255, 0), 5);
                circle(imgOriginal,ARMOR.center,7.5,Scalar(0, 0, 255),5);//在装甲板中心画一个圆
                cout << "中心点位置" << "["<<ARMOR.center.x << ", "<< ARMOR.center.y << "]" << endl;
                x = ARMOR.center.x, y = ARMOR.center.y;
            }
        }
        ROIimg(imgOriginal, x, y);
        imshow("Armor", imgOriginal);
        waitKey(30);
    }

    return 0;

}

