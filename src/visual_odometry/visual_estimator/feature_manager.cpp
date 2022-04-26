#include "feature_manager.h"
//获取最后一个观测到该特征的图像帧对应的索引
int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}
//构造函数先将左右目相机到imu的旋转矩阵设为单位阵
FeatureManager::FeatureManager(Matrix3d _Rs[])
        : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}
//设置左右目相机到imu的旋转矩阵
void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}
//清空容器内所有信息
void FeatureManager::clearState()
{
    feature.clear();
}
//获取窗口内被跟踪的特征点数量
int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature) //遍历所有的feature
    {

        it.used_num = it.feature_per_frame.size(); //所有特征点被观测到的帧数
        // 如果该特征点被两帧及以上图像观测到了，并且不是在滑窗最后两帧才观测到的
        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            //有效特征点数量++
            cnt++;
        }
    }
    return cnt;
}
/**
 * @brief   把特征点放入feature的list容器中，计算每一个点跟踪次数和它在次新帧和次次新帧间的视差，返回是否是关键帧
 * @param[in]   frame_count 窗口内帧的个数
 * @param[in]   image 存储帧中所有特征点构成的map,索引为feature_id，值为[camera_id,[x,y,z,u,v,vx,vy]]
 * @param[in]   td cam和imu之间的时间差
 * @return  bool true：次新帧是关键帧;false：非关键帧
 */
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, double td)
{
//特征点数量
ROS_DEBUG("input feature: %d", (int)image.size());
//窗口内被跟踪的特征点数量
ROS_DEBUG("num of feature: %d", getFeatureCount());
double parallax_sum = 0; //所有特征点视差和
int parallax_num = 0;
last_track_num = 0; //被跟踪的个数
// 挨个遍历image map中的特征点
for (auto &id_pts : image)
{
//构造特征点对象
FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

// find feature id in the feature bucket
//迭代器寻找feature list中是否有这个feature_id
int feature_id = id_pts.first; //当前image中对应的特征点id
// 在feature容器中找到feature_id（来自image map） 然后返回对应的FeatureManager格式
auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
{ return it.feature_id == feature_id; });
//遍历到feature容器最后还是没有找到，则新建一个，并在尾部添加该元素
if (it == feature.end())
{
// this feature in the image is observed for the first time, create a new feature object
feature.push_back(FeaturePerId(feature_id, frame_count, f_per_fra.depth));
feature.back().feature_per_frame.push_back(f_per_fra);
}
//如果能找到则把图像帧添加进去
else if (it->feature_id == feature_id)
{
// this feature in the image has been observed before
it->feature_per_frame.push_back(f_per_fra);
last_track_num++; //该帧有多少相同的特征点被跟踪
// sometimes the feature is first observed without depth
// (initialize initial feature depth with current image depth is not exactly accurate if camera moves very fast, then lines bebow can be commented out)
//如果由lidar测量的深度值大于0则将lidar改为可用
if (f_per_fra.depth > 0 && it->lidar_depth_flag == false)
{
it->estimated_depth = f_per_fra.depth;
it->lidar_depth_flag = true;
it->feature_per_frame[0].depth = f_per_fra.depth;
}
}
}
// 特征点没有出现在至少两帧图像里，或者图像中被追踪的特征点数量小于20，则次新帧设置为关键帧
// 因为再不设置的话要跟丢了
if (frame_count < 2 || last_track_num < 20)
return true;
// 计算每个特征在次新帧和次次帧中的视差
for (auto &it_per_id : feature)
{
//观测到该特征点的起始帧小于倒数第三帧并且结束帧大于倒数第二帧，保证至少有两帧能观测到
if (it_per_id.start_frame <= frame_count - 2 &&
it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
{
//该特征点在所有观测到它的图像帧里连续两帧之间的总和
// parallax_sum = 1^para_0 + 4^para_1 + 5^para_4 ---> 5^para_0
parallax_sum += compensatedParallax2(it_per_id, frame_count);
parallax_num++; //个数
}
}
//没有在别的图像帧中观测到，说明当前图像帧是新加进去的，为关键帧
if (parallax_num == 0)
{
return true;
}
else
{
ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
//或者平均视差大于阈值的位关键帧
return parallax_sum / parallax_num >= MIN_PARALLAX;
}
}
//调试输出
void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ", j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}
//得到给定两帧之间对应特征点的3D坐标
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature) //遍历feature中的所有特征点
    {
        //如果给定的两帧在窗口范围内
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            //存储点在归一化坐标系下的位置
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame; //图像帧l减去观测到该特征点的起始帧
            int idx_r = frame_count_r - it.start_frame; //图像帧r减去观测到该特征点的起始帧

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}
//设置特征点的逆深度估计值
void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;//赋值为-1 ++之后变为0
    for (auto &it_per_id : feature)//遍历feature容器中所有特征点
    {
        //至少两帧观测得到该特征点 且 首次观测到该特征点的图像帧在滑动窗口内
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //求解逆深度
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //深度小于0
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2; //估计失败
        }
        else
            it_per_id.solve_flag = 1; //估计成功
    }
}
//如果估计失败则剔除
void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        // 估计失败则剔除该特征点
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}
//清除存储的深度
void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        it_per_id.lidar_depth_flag = false;
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        //至少两帧观测得到这个特征点  且  首次观测到该特征点的图像帧在滑动窗范围内
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        // optimized depth after ceres maybe negative, initialize them with default value for this optimization
        if (it_per_id.estimated_depth > 0)
            dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
        else
            dep_vec(++feature_index) = 1. / INIT_DEPTH;//5.0
    }
    return dep_vec;
}
//三角化特征点深度
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    //遍历feature容器中所有特征信息
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        //至少两帧观测得到这个特征点  且 不能是滑窗中的最后两帧
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        // depth is available, skip triangulation (trust the first estimate)
        if (it_per_id.estimated_depth > 0)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;
        // R0 t0为第i帧cam--->world的变换矩阵
        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            // R1 t1为第j帧cam--->world的变换矩阵
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            // R t为cam(j)--->cam(i)的变换矩阵
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            // P: cam(i)--->cam(j)的变换矩阵
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            //获取归一化坐标系下的位置
            //只保留方向信息 去除尺度信息
            Eigen::Vector3d f = it_per_frame.point.normalized();
            //P = [P1 P2 P3]^T
            //AX=0      A = [A(2*i) A(2*i+1) A(2*i+2) A(2*i+3) ...]^T
            //A(2*i)   = x(i) * P3 - z(i) * P1
            //A(2*i+1) = y(i) * P3 - z(i) * P2
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        //对A的SVD分解得到其最小奇异值对应的单位奇异向量(x,y,z,w)，深度为z/w
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];

        // update depth from triangulation
        // 得到的深度值实际上就是第一个观察到这个特征点的相机坐标系下的深度值
        it_per_id.estimated_depth = svd_method;
        // check if triangulation failed
        if (it_per_id.estimated_depth < 0)//太近 vinsmono--->0.1 ???
        {
            it_per_id.estimated_depth = INIT_DEPTH;//5.0
        }
    }
}

void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}
//边缘化最老帧时，处理特征点保存的帧号，将起始帧是最老帧的特征点的深度值进行转移
// marg_R、marg_P为被边缘化的位姿，new_R、new_P为在这下一帧的位姿
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        // 没有被移除的帧看到，则直接将帧号减一（滑窗滑掉了，往后移动一位）
        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            // feature point and depth in old local camera frame
            //特征点起始帧为最老帧
            //取出归一化坐标系
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            double depth = -1;
            //如果lidar测量好使则取lidar的值
            if (it->feature_per_frame[0].depth > 0)
                // if lidar depth available at this frame for feature
                depth = it->feature_per_frame[0].depth;
                //如果三角化的好使则用三角化的值
            else if (it->estimated_depth > 0)
                // if estimated depth available
                depth = it->estimated_depth;

            // delete current feature in the old local camera frame
            // 该特征点被观测到的帧中减去一个
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            // 特征点在当前滑窗中只被观测到了一次或没有被观测则直接剔除
            if (it->feature_per_frame.size() < 2)
            {
                // delete feature from feature manager
                feature.erase(it);
                continue;
            }
            else//深度信息改变了（以最先观测到该特征点为起点计算的）
            {
                // 特征点在实际相机坐标系下的三维坐标
                Eigen::Vector3d pts_i = uv_i * depth;                          // feature in cartisian space in old local camera frame
                // 特征点在世界坐标系下的三维坐标
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;             // feautre in cartisian space in world frame
                // 转换到新的最老帧相机坐标系下的三维坐标
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P); // feature in cartisian space in shifted local camera frame
                double dep_j = pts_j(2);

                // after deletion, the feature has lidar depth in the first of the remaining frame
                if (it->feature_per_frame[0].depth > 0)
                {
                    it->estimated_depth = it->feature_per_frame[0].depth;
                    it->lidar_depth_flag = true;
                }
                    // calculated depth in the current frame
                else if (dep_j > 0) //检查深度是否有效
                {
                    // 有效的话就得到在现在最老帧下的深度值
                    it->estimated_depth = dep_j;
                    it->lidar_depth_flag = false;
                }
                    // non-positive depth, invalid
                else
                {
                    // 无效就设置默认值
                    it->estimated_depth = INIT_DEPTH;//5.0
                    it->lidar_depth_flag = false;
                }
            }
        }
    }
}
//初始化还没结束，此时边缘化最老帧时，直接将特征点所保存的帧号向前滑动，
// 不进行地图点新的深度的换算
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        // 如果特征点起始帧号不为0则直接减一
        if (it->start_frame != 0)
            it->start_frame--;
        else//否则剔除feature_per_frame容器的头
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            //如果feature_per_frame为空则直接剔除该特征点
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}
//边缘化次新帧时，对特征点在次新帧的信息进行移除处理
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;
        // 如果起始帧为最新帧则将其滑动为次新帧
        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            //
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}
//计算某个特征点it_per_id在次新帧和次次新帧的视差
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point; //倒数第二帧j的3D路标点

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point; //倒数第三帧i的3D路标点
    Vector3d p_i_comp;

    // int r_i = frame_count - 2;
    // int r_j = frame_count - 1;
    // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    //             cam^r_imu * imu^Rs_wolrd_j * wolrd^Rs_imu_i * imu^r_cam *  cam^P_point
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}