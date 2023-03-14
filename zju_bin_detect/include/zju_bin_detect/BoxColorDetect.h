#pragma once
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

typedef struct stHSVThresdhold
{
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
}stHSVThresdhold;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

class CBoxColorDetect
{
public:
    CBoxColorDetect();
    ~CBoxColorDetect();
    void Init();
    void ColorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud);
    bool SingleFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inSrcCloud, int inBoxIndex, bool bGetBox);
    void GetBox(int inIndex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inFiltedCloud);
    void RGB2HSI(int inR,int inG,int inB,int* outH, int* outS, int* outI);
    float CalDist(float x1 ,float y1, float z1, float x2, float y2, float z2);
    stHSVThresdhold thHSV[3];
    vector<stBoxMarker> arBox[3];
    int nIndexBox[3];
    float fCenterX;
    float fCenterY;
    float fCenterZ;
   
};