//
// Created by liuk on 2021/7/20.
//

#ifndef DAA_LIDAR_ALGO_POINTCLOUD_COMMON_H
#define DAA_LIDAR_ALGO_POINTCLOUD_COMMON_H
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "iostream"
#include "math.h"
#include "vector"
#include "unordered_map"
#include "map"
#include "unordered_set"
#include "GRANSAC.hpp"
#include "LineModel.hpp"
namespace pointcloud_common {
    struct point_type{
        float x;
        float y;
        float z;
        float intensity;
        point_type(){};
        point_type(float x, float y, float z, float intensity) : x(x), y(y), z(z), intensity(intensity){}
    };
    struct grid_feature{
        float minheight;
        float maxheight;
        float x;
        float y;
        uint32_t total_points;
        uint8_t  class_index;
        uint32_t cluster_index = 1;
    };
    struct KEY
    {
        float x;
        float y;
        float z;
        KEY(float x, float y, float z) : x(x), y(y), z(z){}

        bool operator==(const KEY &oth) const
        {
            return x==oth.x&&y==oth.y&&z==oth.z;
        }
    };

    struct HashFunc
    {
        std::size_t operator()(const KEY &key) const
        {
            using std::size_t;
            using std::hash;

            return ((hash<float>()(key.x)
                     ^ (hash<float>()(key.y) << 1)) >> 1)
                   ^ (hash<float>()(key.z) << 1);
        }
    };
    struct EqualKey
    {
        bool operator () (const KEY &lhs, const KEY &rhs) const
        {
            return lhs.x  == rhs.x
                   && lhs.y == rhs.y
                   && lhs.z  == rhs.z;
        }

    };
    typedef std::unordered_map<KEY, grid_feature, HashFunc, EqualKey> keymap;
    typedef std::vector<point_type> pointcloud_type;
    struct pointcloud_t{
        pointcloud_type points;
        uint32_t width;
    };
    struct pointfeature {
        std::map<float, int> totalpoints;
        std::vector<float> cluster_feature;
    };
    class Feature_map
    {
    public:
        Feature_map(){};
        Feature_map(Eigen::Vector3f map_resulotion)
        {
            this->map_resulotion = map_resulotion;
        }
        keymap static_map;
        std::vector<keymap > dynamatic_grid;
        keymap noise_grid;
        void map_clear()
        {
            static_map.clear();
            dynamatic_grid.clear();
            noise_grid.clear();
        }
    private:
        Eigen::Vector3f map_resulotion;
    };
    class occupied_grid3d {
    public:
        occupied_grid3d(Eigen::Vector3f mapsize, Eigen::Vector3f resolution);

        Eigen::Vector3f resolution;

        void CreateMap(const pointcloud_t &InputPointcloud);

        std::unordered_map<int, pointfeature > result_map;

        virtual  Feature_map GetFeatuerMap();

        Eigen::Vector3f GetPositionByindex(int index);

        keymap cluster_map;

    private:
        Eigen::Vector3i mapsize;
        float cluster_value;
        int map_width;
        int max_index;
        uint32_t map_offset_;

        int GetIndexbyPosition(point_type);

        Feature_map FlyingObjectFilter(const std::vector<std::vector<int> > cluster, keymap& group_num);
        std::vector<uint8_t > ODFilterByself(Feature_map& map);
        std::vector<uint8_t > ODFilterByneighbour(Feature_map& map);
        std::vector<uint8_t > ODFilterByPlanemodel(Feature_map& map);
        void filterimplenent(std::vector<uint8_t > filterarr, Feature_map& map);
        std::vector<std::map<float, int> > Cluster1D(std::map<float, int> items, float diff_max);


    };

    class ClusterRegionExpand{
    public:
        ClusterRegionExpand();
        static std::vector<std::vector<int> > grid_grouping ( Eigen::Vector3i num, int minpts,  keymap& group_num, Eigen::Vector3f mapSize, std::unordered_map<int, pointfeature >& result_map,
                                                              Eigen::Vector3f resolution);
    private:
        static int dbscan_expand(  Eigen::Vector3i num, int minpts, KEY pointnum, int clusterId,
                                   keymap& group_num, Eigen::Vector3f mapSize, std::unordered_map<int, pointfeature >& result_map,
                                   Eigen::Vector3f resolution);
        static int dbscan_distinguish( Eigen::Vector3i num, KEY pointnum, std::vector< KEY> & densty,
                                       keymap& group_num,Eigen::Vector3f mapSize, std::unordered_map<int, pointfeature >& result_map,
                                       Eigen::Vector3f resolution);
        static int dbscan_distinguish_( Eigen::Vector3i num, KEY pointnum, std::vector< KEY> & densty,
                                         keymap& group_num, Eigen::Vector3f mapSize,
                                         std::unordered_map<int, pointfeature >& result_map, Eigen::Vector3f resolution);
    };


    class pointcloud_common {
    public:
        pointcloud_common();
        static void Rotationpoints(Eigen::Vector3d angle, pointcloud_t& points);

    };
    class Bbox{
    public:
        struct BoxFeature
        {
            Eigen::Vector3f center;
            Eigen::Vector3f size;
            Eigen::Vector3f direction;
            Eigen::Vector3f speed;
            int total_points;
            float probility = 1.0;
            int track_id = 0;
        };
        typedef std::vector<BoxFeature> ODvector_type;
        ODvector_type getboundingbox( std::vector<keymap> object,Eigen::Vector3f resolution);
    };
}

#endif //DAA_LIDAR_ALGO_POINTCLOUD_COMMON_H


