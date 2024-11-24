#include"commen.h"

//为了防止有其他装甲板干扰，使得算法可以追踪，加入限幅滤波
filter_type filter(filter_type effective_value, filter_type new_value, filter_type delat_max)
{
    if ( ( new_value - effective_value > delat_max ) || ( effective_value - new_value > delat_max ))
    {
        new_value=effective_value;
        return effective_value;
    }
    else
    {
        new_value=effective_value;
        return new_value;
    }
}
// 为辅助筛选装甲板，提高算法运行速度，做一次筛选预处理
RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
   {
       using std::swap;

       float& width = rec.size.width;
       float& height = rec.size.height;
       float& angle = rec.angle;

       if (mode == WIDTH_GREATER_THAN_HEIGHT)
       {
           if (width < height)
           {
               swap(width, height);
               angle += 90.0;
           }
       }

       while (angle >= 90.0) angle -= 180.0;
       while (angle < -90.0) angle += 180.0;

       if (mode == ANGLE_TO_UP)
       {
           if (angle >= 45.0)
           {
               swap(width, height);
               angle -= 90.0;
           }
           else if (angle < -45.0)
           {
               swap(width, height);
               angle += 90.0;
           }
       }
   return rec;
}//筛去竖着的轮廓
//输出ROI
void ROIimg(Mat imgOriginal, int x, int y)
{
    //选取ROI
        //创造一个空白图框
        Mat backGround = Mat::zeros(imgOriginal.size(),CV_8UC3);
        //设置选取位置
        vector<Rect> rectROI;
        Rect rect1(x-30, y-30, 60, 60);
        //截取
        Mat ROI1 = imgOriginal(rect1);
        rectROI.push_back(rect1);
        for(int i = 0; i < rectROI.size(); i++)
        {
            imgOriginal(rectROI[i]).copyTo(backGround(rectROI[i]));
        }
        //展示ROI
        imshow("ROI", backGround);
}

