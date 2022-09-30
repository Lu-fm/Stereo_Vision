#pragma once
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

using namespace std;
using namespace cv;

#define BOX_DIM_WIDTH 600//1200
#define BOX_DIM_HEIGHT 400//800
#define BOX_TEMP_SIDE 60//120
#define OVER_SCALE_SIDE 40//80

typedef struct stBoxStep
{
    float fRotAngle;
    int nX;
    int nY;
    int nMatch;
}stBoxStep;

typedef struct stICPResult
{
    float angle;
    float x_offset;
    float y_offset;
    int result;
}stICPResult;

#define RACK_TEMP_X 200
#define RACK_TEMP_Y 200
#define RACK_TEMP_Z 200
typedef struct stRackTemp
{
    unsigned char data[RACK_TEMP_X][RACK_TEMP_Y][RACK_TEMP_Z];
    int h_partition_y[4];
    int v_partition_z[4];
    int d_partition_x;
    int b_back;
}stRackTemp;

class CBoxDim
{
public:
    CBoxDim();
    ~CBoxDim();
    void InitShape();
    void InitCV();
    void InitTestPos();
    void LoadTemp(const char* filename);
    void ScalePoint(int inX, int inY);
    int CalPositionResult(stICPResult& inResult);
    void FindBoxPosition();
    bool OneMoreStep();
    void RotBox(float inRotAngle , cv::Mat& inMat);
    bool CalBoxPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud);
    float GetMaxHeight();
    float GetMinHeight();
    void ShowOver();
    stRackTemp* pRackTemp;
    stICPResult curResult;
    stICPResult boxPosition;
    bool bBoxTracked;
};