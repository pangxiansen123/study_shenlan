#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"

/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * https://blog.csdn.net/u010429424/article/details/77834046
 * 2D画线算法　来进行计算两个点之间的grid cell
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    // 如果 Δx > Δy ，说明x轴的最大差值大于y轴的最大差值，x轴方向为步进的主方向，xStep = 1，yStep = k；
    // 如果 Δy> Δx，说明y轴的最大差值大于x轴的最大差值，y轴方向为步进的主方向，yStep = 1，xStep = 1 / k。
    // 这个if就是谁是主方向
    if (steep)
    {
        // 都有交换相同类型的两个变量内容的需要
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    // 由降变为升
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            pointX = y;
            pointY = x;
        }
        else
        {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
            y += ystep;
            error -= deltaX;
        }

        //不包含最后一个点．
        if (pointX == x1 && pointY == y1)
            continue;

        //保存所有的点
        tmpIndex.SetIndex(pointX, pointY);

        gridIndexVector.push_back(tmpIndex);
    }

    return gridIndexVector;
}

void SetMapParams(void)
{
    mapParams.width = 1000;
    mapParams.height = 1000;
    mapParams.resolution = 0.05;

    //每次被击中的log变化值，覆盖栅格建图算法需要的参数
    mapParams.log_free = -1;
    mapParams.log_occ = 2;

    //每个栅格的最大最小值．
    mapParams.log_max = 100.0;
    mapParams.log_min = 0.0;

    mapParams.origin_x = 0.0;
    mapParams.origin_y = 0.0;

    //地图的原点，在地图的正中间
    mapParams.offset_x = 500;
    mapParams.offset_y = 500;

    pMap = new unsigned char[mapParams.width * mapParams.height];

    //计数建图算法需要的参数
    //每个栅格被激光击中的次数
    pMapHits = new unsigned long[mapParams.width * mapParams.height];
    //每个栅格被激光通过的次数
    pMapMisses = new unsigned long[mapParams.width * mapParams.height];

    //TSDF建图算法需要的参数
    pMapW = new unsigned long[mapParams.width * mapParams.height];
    pMapTSDF = new double[mapParams.width * mapParams.height];

    //初始化
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        pMap[i] = 50;
        pMapHits[i] = 0;
        pMapMisses[i] = 0;
        pMapW[i] = 0;
        pMapTSDF[i] = -1;
    }
}

//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(double x, double y)
{
    GridIndex index;

    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
    return linear_index;
}

//判断index是否有效
bool isValidGridIndex(GridIndex index)
{
    if (index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

void DestoryMap()
{
    if (pMap != NULL)
        delete pMap;
}

//
void OccupanyMapping(std::vector<GeneralLaserScan> &scans, std::vector<Eigen::Vector3d> &robot_poses)
{
    std::cout << "开始建图，请稍后..." << std::endl;
    //枚举所有的激光雷达数据
    for (int i = 0; i < scans.size(); i++)
    {
        GeneralLaserScan scan = scans[i];
        Eigen::Vector3d robotPose = robot_poses[i];

        //机器人的下标
        GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));

        for (int id = 0; id < scan.range_readings.size(); id++)
        {
            double dist = scan.range_readings[id];
            double angle = -scan.angle_readings[id]; // 激光雷达逆时针转，角度取反

            if (std::isinf(dist) || std::isnan(dist))
                continue;

            //计算得到该激光点的雷达坐标系的坐标
            double theta = -robotPose(2); // 激光雷达逆时针转，角度取反
            double laser_x = dist * cos(angle);
            double laser_y = dist * sin(angle);
            // 激光点在世界坐标系下的位置
            double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);
            double world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);

            //start of TODO 对对应的map的cell信息进行更新．（1,2,3题内容）
            //先判断激光点是否合法
            // GridIndex lidar_index = ConvertWorld2GridIndex(world_x, world_y);
            // if (!isValidGridIndex(lidar_index))
            //     continue;
            // pMap[GridIndexToLinearIndex(lidar_index)] += mapParams.log_occ;
            // if (pMap[GridIndexToLinearIndex(lidar_index)] > 100)
            // {
            //     pMap[GridIndexToLinearIndex(lidar_index)] = 100;
            // }
            // // 计算激光点与机器人位置之间的cell
            // std::vector<GridIndex> point_line = TraceLine(robotIndex.x, robotIndex.y, lidar_index.x, lidar_index.y);
            // for (int i = 0; i < point_line.size(); i++)
            // {
            //     if (!isValidGridIndex(point_line[i]))
            //         return false;
            //     pMap[GridIndexToLinearIndex(point_line[i])] += mapParams.log_free;
            //     if (pMap[GridIndexToLinearIndex(point_line[i])] < 0)
            //     {
            //         pMap[GridIndexToLinearIndex(point_line[i])] = 0;
            //     }
            // }
            //end of TODO

            // 题目二
            // 先判断激光点是否合法
            GridIndex lidar_index = ConvertWorld2GridIndex(world_x, world_y);
            if (!isValidGridIndex(lidar_index))
                continue;
            pMapHits[GridIndexToLinearIndex(lidar_index)]++;
            // 计算激光点与机器人位置之间的cell
            std::vector<GridIndex> point_line = TraceLine(robotIndex.x, robotIndex.y, lidar_index.x, lidar_index.y);
            for (int i = 0; i < point_line.size(); i++)
            {
                if (!isValidGridIndex(point_line[i]))
                    return false;
                pMapMisses[GridIndexToLinearIndex(point_line[i])]++;
            }
            // 题目二

            // 题目三
            // GridIndex lidar_index = ConvertWorld2GridIndex(world_x, world_y);
            // if (!isValidGridIndex(lidar_index))
            //     continue;
            // pMapTSDF[GridIndexToLinearIndex(lidar_index)] += pMapW[i] * pMapTSDF[GridIndexToLinearIndex(lidar_index)] + (scan.range_readings[id] - sqrt(power((ConvertWorld2GridIndex(world_x) - ConvertWorld2GridIndex(laser_x)),2),power((ConvertWorld2GridIndex(world_y) - ConvertWorld2GridIndex(laser_y)),2)));
            // pMapTSDF[GridIndexToLinearIndex(point_line.size() - 2)]+= pMapW[i] * pMapTSDF[GridIndexToLinearIndex(point_line.size() - 2)] + (scan.range_readings[id] - sqrt(power((point_line[point_line.size() - 2].x - ConvertWorld2GridIndex(laser_x)),2),power((point_line[point_line.size() - 2].y - laser_y),2)));
            // pMapTSDF[GridIndexToLinearIndex(point_line.size() - 3)]+= pMapW[i] * pMapTSDF[GridIndexToLinearIndex(point_line.size() - 3)] + (scan.range_readings[id] - sqrt(power((point_line[point_line.size() - 3].x - ConvertWorld2GridIndex(laser_x)),2),power((point_line[point_line.size() - 3].y - laser_y),2)));
            double t = 0.3;
            double behind_dist = dist + t;
            if (std::isinf(behind_dist) || std::isnan(behind_dist))
                continue;

            //计算得到该激光点的世界坐标系的坐标
            double theta = -robotPose(2); // 激光雷达逆时针转，角度取反
            double behind_laser_x = behind_dist * cos(angle);
            double behind_laser_y = behind_dist * sin(angle);
            double behind_world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);
            double behind_world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);
            GridIndex behind_lidar_index = ConvertWorld2GridIndex(behind_world_x, behind_world_y);
            double behind_gripper_x = (behind_lidar_index.x - mapParams.offset_x) * mapParams.resolution + mapParams.origin_x;
            double behind_gripper_y = (behind_lidar_index.y - mapParams.offset_y) * mapParams.resolution + mapParams.origin_y;
            double behind_sdf = dist - std::sqrt(std::power((behind_gripper_x - robotPose(0)), 2), std::power((behind_gripper_y - robotPose(1)), 2));
            double behind_tsdf = std::max(-1, std::min(1, behind_sdf / t));
            pMapTSDF[GridIndexToLinearIndex(behind_lidar_index)] += (pMapW[i] * pMapTSDF[GridIndexToLinearIndex(behind_lidar_index)] + behind_tsdf) / (pMapW[GridIndexToLinearIndex(behind_lidar_index)] + 1);
            pMapW[GridIndexToLinearIndex(behind_lidar_index)]++;
            std::vector<GridIndex> point_line = TraceLine(robotIndex.x, robotIndex.y, behind_lidar_index.x, behind_lidar_index.y);
            for (int i = 0; i < point_line.size(); i++)
            {
                if (!isValidGridIndex(point_line[i]))
                    return false;
                double behind_i_x = (point_line[i].x - mapParams.offset_x) * mapParams.resolution + mapParams.origin_x;
                double behind_i_y = (point_line[i].y - mapParams.offset_y) * mapParams.resolution + mapParams.origin_y;
                double behind_i_sdf = dist - std::sqrt(std::power((behind_i_x - robotPose(0)), 2), std::power((behind_i_y - robotPose(1)), 2));
                double behind_i_tsdf = std::max(-1, std::min(1, behind_i_sdf / t));
                pMapTSDF[GridIndexToLinearIndex(point_line[i])] += (pMapW[i] * pMapTSDF[GridIndexToLinearIndex(point_line[i])] + behind_i_tsdf) / (pMapW[GridIndexToLinearIndex(point_line[i])] + 1);
                pMapW[GridIndexToLinearIndex(point_line[i])]++;
            }
            // 题目三
        }
    }
    //start of TODO 通过计数建图算法或TSDF算法对栅格进行更新（2,3题内容）
    //第二题
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        // 这里范围在[0 - 1]所以要乘以100
        pMap[i] = 100 * (pMapHits[i]) / (pMapMisses[i] + pMapHits[i]);
        if (pMap[i] > 100)
        {
            pMap[i] = 100;
        }
        else if (pMap[i] < 0)
        {
            pMap[i] = 0;
        }
    }
    // 第二题
    //第三题
    for (int i = 0; i < mapParams.height; i++)
    {
        pMap[i*mapParams.width] = 0;
        for (int j = 1; j < mapParams.width; j++)
        {
            if (pMapTSDF[i*mapParams.width + j - 1] * pMapTSDF[i*mapParams.width + j] < 0)
            {
                if (std::abs(pMapTSDF[i*mapParams.width + j - 1]) < std::abs(pMapTSDF[i*mapParams.width + j]))
                {
                    pMap[i*mapParams.width + j - 1] = 100;
                }else{
                    pMap[i*mapParams.width + j] = 100;
                }
                
            }else{
                 pMap[i*mapParams.width + j] = 0;
            }
            
        }
        
    }
    // 第三题
    //end of TODO
    std::cout << "建图完毕" << std::endl;
}

//发布地图．
void PublishMap(ros::Publisher &map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    //0~100
    int cnt0, cnt1, cnt2;
    cnt0 = cnt1 = cnt2 = 100;
    // 遍历每一个点
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        if (pMap[i] == 50)
        {
            rosMap.data[i] = -1.0;
        }
        else
        {

            rosMap.data[i] = pMap[i];
        }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OccupanyMapping");

    ros::NodeHandle nodeHandler;

    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map", 1, true);

    std::vector<Eigen::Vector3d> robotPoses;
    std::vector<GeneralLaserScan> generalLaserScans;

    std::string basePath = "/home/eventec/OccupanyMappingProject/src/data";

    std::string posePath = basePath + "/pose.txt";
    std::string anglePath = basePath + "/scanAngles.txt";
    std::string scanPath = basePath + "/ranges.txt";

    //读取数据
    ReadPoseInformation(posePath, robotPoses);

    ReadLaserScanInformation(anglePath,
                             scanPath,
                             generalLaserScans);

    //设置地图信息
    SetMapParams();

    OccupanyMapping(generalLaserScans, robotPoses);

    PublishMap(mapPub);

    ros::spin();

    DestoryMap();

    std::cout << "Release Memory!!" << std::endl;
}
