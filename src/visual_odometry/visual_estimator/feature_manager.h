#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 8, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        depth = _point(7);
        cur_td = td;
    }
    double cur_td;
    Vector3d point; // 三维点（地图点）
    Vector2d uv;  // 特征点的图像坐标
    Vector2d velocity;  // 特征点的速度信息
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
    double depth; // lidar depth, initialized with -1 from feature points in feature tracker node
};

class FeaturePerId
{
  public:
    const int feature_id; // 特征点的ID
    int start_frame;  // 滑窗中第几帧能够跟踪到当前特征点
    vector<FeaturePerFrame> feature_per_frame; // 对应ID下的所有特征点

    int used_num; // 对应ID下的所有特征点数目
    bool is_outlier;
    bool is_margin;
    double estimated_depth;

    // 新添加的激光雷达属性
    bool lidar_depth_flag;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame, double _measured_depth)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), lidar_depth_flag(false), solve_flag(0) 
    {
        if (_measured_depth > 0)
        {
            estimated_depth = _measured_depth;
            lidar_depth_flag = true;
        }
        else
        {
            estimated_depth = -1;
            lidar_depth_flag = false;
        }
    }

    int endFrame();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();

    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, double td);
    void debugShow();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();

    // 特征点列表
    list<FeaturePerId> feature;
    int last_track_num;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif