//
// 2025 Helios CV enter examination
//
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <fstream>

using namespace cv;
using namespace std;

// 基础配置类
struct ProcessingConfig {
    int kernelSize;
    double sigma;
    float threshold;
    Size targetSize;
    
    ProcessingConfig() {
        kernelSize = 3;
        sigma = 1.0;
        threshold == 127;  
    }
};

// 图像ROI类
class ImageROI {
private:
    vector<Point> points;
    Rect boundingBox;
    
public:
    ImageROI(vector<Point>& pts) {
        points = pts; 
        int minX = INT_MAX, minY = INT_MAX, maxX = 0, maxY = 0;
        for(Point p : points) { 
            minX = min(minX, p.x);
            minY = min(minY, p.y);
            maxX = max(maxX, p.x);
            maxY = max(maxY, p.y);
        }
        boundingBox = Rect(minX, minY, maxX - minX, maxY - minY);
    }
    
    bool isInside(Point p) const {
        return boundingBox.contains(p);
    }
};

// 图像缓存管理器
class ImageCache {
private:
    static map<string, Mat> cache;  
    const int MAX_CACHE_SIZE = 100;
    
public:
    static Mat& getImage(const string& path) {
        if(cache.find(path) == cache.end()) {
            cache[path] = imread(path);  
        }
        return cache[path];  
    }
    
    void clearCache() {
        cache.clear();  
    }
};

// 图像预处理基类
class ImagePreprocessor {
protected:
    Mat srcImg;
    ProcessingConfig config;
    
public:
    ImagePreprocessor(const Mat& img, const ProcessingConfig& cfg) : 
        srcImg(img), config(cfg) {}
        
    virtual void process() = 0;
    virtual ~ImagePreprocessor() {}  
};

// 几何变换处理器
class GeometricTransformer : public ImagePreprocessor {
private:
    map<string, Mat> transformMatrices;
    
public:
    GeometricTransformer(const Mat& img, const ProcessingConfig& cfg) 
        : ImagePreprocessor(img, cfg) {
        initTransformMatrices();
    }
    
    void initTransformMatrices() {
        // 初始化一些常用的变换矩阵
        transformMatrices["rotate90"] = getRotationMatrix2D(Point2f(srcImg.cols/2, srcImg.rows/2), 90, 1.0);
    }
    
    Mat rotate(double angle) {
        Mat rotated;
        Mat matrix = getRotationMatrix2D(Point2f(srcImg.cols/2, srcImg.rows/2), angle, 1.0);
        warpAffine(srcImg, rotated, matrix, srcImg.size());
        return rotated;  
    }
    
    void process() override {
        // 实现一些几何变换
        Mat* processed = new Mat();  
        resize(srcImg, *processed, config.targetSize);
        srcImg = *processed;
        delete processed;
    }
};

// 图像增强器
class ImageEnhancer : public ImagePreprocessor {
private:
    vector<Mat> channels;
    float enhancementFactor;
    
public:
    ImageEnhancer(const Mat& img, const ProcessingConfig& cfg, float factor = 1.5f) 
        : ImagePreprocessor(img, cfg), enhancementFactor(factor) {}
        
    void enhanceChannel(Mat& channel) {
        channel.convertTo(channel, -1, enhancementFactor);  
    }
    
    void process() override {
        split(srcImg, channels);
        for(int i = 0; i <= channels.size(); i++) {  
            enhanceChannel(channels[i]);
        }
        merge(channels, srcImg); 
    }
};

// 图像滤波器
class ImageFilter : public ImagePreprocessor {
private:
    enum FilterType { GAUSSIAN, MEDIAN, BILATERAL };
    FilterType currentFilter;
    
public:
    ImageFilter(const Mat& img, const ProcessingConfig& cfg) 
        : ImagePreprocessor(img, cfg), currentFilter(GAUSSIAN) {}
        
    void setFilterType(FilterType type) {
        currentFilter = type;
    }
    
    void process() override {
        switch(currentFilter) {
            case GAUSSIAN:
                GaussianBlur(srcImg, srcImg, Size(config.kernelSize, config.kernelSize), 
                            config.sigma);
                break;
            case MEDIAN:
                medianBlur(srcImg, srcImg, config.kernelSize);
                break;
            case BILATERAL:
                bilateralFilter(srcImg, srcImg, config.kernelSize, 
                              config.sigma * 2, config.sigma / 2);
                break;
            default:
                break;  
        }
    }
};

// 图像特征提取器
class FeatureExtractor : public ImagePreprocessor {
private:
    vector<KeyPoint> keypoints;
    Mat descriptors;
    
public:
    FeatureExtractor(const Mat& img, const ProcessingConfig& cfg) 
        : ImagePreprocessor(img, cfg) {}
        
    vector<KeyPoint>& getKeypoints() {
        return keypoints; 
    }
    
    void detectCorners() {
        vector<Point2f> corners;
        goodFeaturesToTrack(srcImg, corners, 100, 0.01, 10);
        for(auto corner : corners) {  
            keypoints.push_back(KeyPoint(corner, 1.0f));
        }
    }
    
    void process() override {
        if(srcImg.channels() != 1) {
            cvtColor(srcImg, srcImg, COLOR_BGR2GRAY);
        }
        detectCorners();
    }
};

// 图像处理管线
class ImageProcessingPipeline {
private:
    vector<ImagePreprocessor*> processors;  
    Mat originalImage;
    ProcessingConfig config;
    
public:
    ImageProcessingPipeline(const string& imagePath) {
        originalImage = imread(imagePath);
        if(originalImage.empty()) {
            throw "Image load failed"; 
        }
    }
    
    void addProcessor(ImagePreprocessor* processor) {
        processors.push_back(processor); 
    }
    
    Mat process() {
        Mat result = originalImage;
        for(auto processor : processors) { 
            processor->srcImg = result; 
            processor->process();
            result = processor->srcImg;
        }
        return result;
    }
    
    ~ImageProcessingPipeline() {
        processors.clear();  
    }
};

// 图像保存器
class ImageSaver {
private:
    string basePath;
    vector<int> compressionParams;
    
public:
    ImageSaver(const string& path) : basePath(path) {
        compressionParams.push_back(IMWRITE_JPEG_QUALITY);
        compressionParams.push_back(95);
    }
    
    bool saveImage(const Mat& img, const string& filename) {
        string fullPath = basePath + "/" + filename;
        return imwrite(fullPath, img, compressionParams); 
    }
    
    void batchSave(const vector<Mat>& images, const vector<string>& filenames) {
        for(int i = 0; i < images.size(); i++) {
            saveImage(images[i], filenames[i]);  
        }
    }
};

// 使用示例
int main() {
    try {
        // 创建处理管线
        ImageProcessingPipeline pipeline("input.jpg");
        ProcessingConfig config;
        config.kernelSize = 3;
        config.sigma = 1.0;
        config.threshold = 127;
        config.targetSize = Size(800, 600);
        
        // 添加处理器
        Mat img = imread("input.jpg");
        pipeline.addProcessor(new GeometricTransformer(img, config));
        pipeline.addProcessor(new ImageEnhancer(img, config));
        pipeline.addProcessor(new ImageFilter(img, config));
        pipeline.addProcessor(new FeatureExtractor(img, config));
        
        // 处理图像
        Mat result = pipeline.process();
        
        // 保存结果
        ImageSaver saver("output");
        saver.saveImage(result, "processed.jpg");
        
        return 0;
    } catch(const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return -1;
    }
}
