//
// Created by liuk on 2021/7/20.
//


#include "pointcloud_common.h"
void pointcloud_common::pointcloud_common::Rotationpoints(Eigen::Vector3d angle, pointcloud_t &points) {
    Eigen::Map< Eigen::MatrixXf> mat((float*)points.points.data(),4,  points.width);
    Eigen::Matrix3f r_z = Eigen::AngleAxisd(angle[2], Eigen::Vector3d::UnitZ()).toRotationMatrix().cast<float>();
    Eigen::Matrix3f r_y = Eigen::AngleAxisd(angle[1], Eigen::Vector3d::UnitY()).toRotationMatrix().cast<float>();
    Eigen::Matrix3f r_x = Eigen::AngleAxisd(angle[0], Eigen::Vector3d::UnitX()).toRotationMatrix().cast<float>();
    mat.block(0, 0, 3, points.width) = r_z *r_y *r_x * mat.block(0, 0, 3, points.width);
}
pointcloud_common::occupied_grid3d::occupied_grid3d(Eigen::Vector3f mapsize, Eigen::Vector3f resolution) {
    this->mapsize[0] = mapsize[0] / resolution[0];
    this->mapsize[1] = mapsize[1] / resolution[1];
    this->mapsize[2] = mapsize[2] / resolution[2];
    this->resolution = resolution;
    this->map_width = this->mapsize[1];
    this->max_index = this->mapsize[0] * this->mapsize[1];
    this->map_offset_ = this->map_width / 2;
    cluster_value = 0.5;
}
int pointcloud_common::occupied_grid3d::GetIndexbyPosition(point_type position)
{
    int index;
    int16_t x = floor(position.x / resolution[0]);
    int16_t y = floor(position.y / resolution[1]);
    index = (int)(x << 16) | (uint16_t)y;
    return index;
}
Eigen::Vector3f pointcloud_common::occupied_grid3d::GetPositionByindex(int index)
{
    Eigen::Vector3f poisition;
    int16_t y = (index&0xffff);
    int16_t x = (index>>16);
    poisition[0] = x * 1.0 * resolution[0];
    poisition[1] = y * 1.0 * resolution[1];
    poisition[2] = 0;
    return poisition;
}
void pointcloud_common::occupied_grid3d::CreateMap(const pointcloud_t &InputPointcloud) {
    this->result_map.clear();
    for (int i = 0; i < InputPointcloud.width; i++)
    {
        int index = this->GetIndexbyPosition(InputPointcloud.points[i]);
        std::unordered_map<int, pointfeature >::iterator findresult = result_map.find(index);
        if (findresult != result_map.end())
        {
            findresult->second.totalpoints.insert(std::pair<float, int>(InputPointcloud.points[i].z, i));
        }
        else
        {
            pointfeature item;
            item.totalpoints.insert(std::pair<float, int>(InputPointcloud.points[i].z, i));
            this->result_map.insert(std::pair<int, pointfeature>(index, item));
        }
    }
}
pointcloud_common::Feature_map pointcloud_common::occupied_grid3d::FlyingObjectFilter(const std::vector<std::vector<int> > cluster, keymap& group_num)
{
    std::unordered_map<int, int> filter_arr;
    std::unordered_map<int, int>::iterator itgf ;
    Feature_map featuremap_t(this->resolution);
    int number_t = 0;
    for(int i = 0; i < cluster.size(); i ++) {
        itgf = filter_arr.find(cluster[i][0]);

        if ((itgf == filter_arr.end()) && (cluster[i][1] < 10)) {
            filter_arr.insert(std::pair<int, int>(cluster[i][0], number_t));
            number_t++;
        }
    }
    featuremap_t.dynamatic_grid.resize(filter_arr.size());
    keymap::iterator itg = group_num.begin();
    int number_total =0;
    std::unordered_map<int, KEY>  cluster_filter_item;
    int number =0;
    while(itg != group_num.end())
    {

        std::unordered_map<int, int>::iterator itg2 = filter_arr.find(itg->second.cluster_index);
        if ( itg2 != filter_arr.end() )
        {
            featuremap_t.dynamatic_grid[itg2->second].insert(*itg);
            number ++;
        }
        else if (itg->second.cluster_index != 0)
        {
            KEY key(itg->first.x,itg->first.y,0);
            keymap ::iterator it = featuremap_t.static_map.find(key);
            if (it == featuremap_t.static_map.end())
               featuremap_t.static_map.insert(std::pair<KEY, grid_feature>(key, itg->second));
            else
            {
                float max = std::max(it->second.maxheight, itg->second.maxheight);
                float min = std::min(it->second.minheight, itg->second.minheight);
                it->second.minheight = min;
                it->second.maxheight = max;
                it->second.total_points += itg->second.total_points;
            }
        }
        else
        {
            featuremap_t.noise_grid.insert(*itg);
        }
        itg++;
    }

//    std::vector<uint8_t>filterresult = ODFilterByself(featuremap_t);
//    filterimplenent(filterresult, featuremap_t );
//    std::vector<uint8_t>filterresult2 = ODFilterByneighbour(featuremap_t);
//    filterimplenent(filterresult2, featuremap_t );
//    std::vector<uint8_t>filterresult3 = ODFilterByPlanemodel(featuremap_t);
//    filterimplenent(filterresult3, featuremap_t );
    return  featuremap_t;
}
void pointcloud_common::occupied_grid3d::filterimplenent(std::vector<uint8_t > filterarr, Feature_map& map)
{
    std::vector<uint8_t>::iterator itl = filterarr.begin();
    std::vector<keymap>::iterator ifeature = map.dynamatic_grid.begin();
    while (itl != filterarr.end())
    {
        if(*itl == 1)
        {
            keymap::iterator itdy2 = ifeature->begin();
            while(itdy2 != ifeature->end())
            {
                KEY key(itdy2->first.x,itdy2->first.y,0);
                keymap ::iterator it = map.static_map.find(key);
                if (it == map.static_map.end())
                    map.static_map.insert(std::pair<KEY, grid_feature>(key, itdy2->second));
                else
                {
                    float max = std::max(it->second.maxheight, itdy2->second.maxheight);
                    float min = std::min(it->second.minheight, itdy2->second.minheight);
                    it->second.minheight = min;
                    it->second.maxheight = max;
                    it->second.total_points += itdy2->second.total_points;
                }
                itdy2++;
            }
            map.dynamatic_grid.erase(ifeature);
            filterarr.erase(itl);
        }
        else
        {
            ++itl;
            ++ifeature;
        }
    }

}
std::vector<uint8_t > pointcloud_common::occupied_grid3d::ODFilterByself(Feature_map& map) {
    std::vector<keymap>::iterator itl = map.dynamatic_grid.begin();
    std::vector<uint8_t > result(map.dynamatic_grid.size(), 0);
    int k_number = 0;
    while (itl != map.dynamatic_grid.end()) {
        keymap::iterator itdy = itl->begin();
        float maxt = itdy->second.maxheight;
        float mint = itdy->second.minheight;
        while (itdy != itl->end()) {
            maxt = std::max(maxt, itdy->second.maxheight);
            mint = std::min(mint, itdy->second.minheight);
            itdy++;
        }
        if ((maxt - mint) > 1.5)
        {
            result[k_number] = 1;
        }
        itl++;
        k_number ++;
    }
    return result;
}
std::vector<uint8_t > pointcloud_common::occupied_grid3d::ODFilterByneighbour(Feature_map& map) {
    std::vector<keymap>::iterator itl = map.dynamatic_grid.begin();
    std::unordered_set<KEY,HashFunc, EqualKey> item_tmep;
    std::vector<uint8_t > result(map.dynamatic_grid.size(), 0);
    int k_number = 0;
    while (itl != map.dynamatic_grid.end()) {
        item_tmep.clear();
        keymap::iterator itdy = itl->begin();
        float maxt = itdy->second.maxheight;
        float mint = itdy->second.minheight;
        while (itdy != itl->end()) {
            float minx = itdy->first.x - 3.0;
            float maxx = itdy->first.x + 3.0;
            float miny = itdy->first.y - 3.0;
            float maxy = itdy->first.y + 3.0;
            maxt = std::max(maxt, itdy->second.maxheight);
            mint = std::min(mint, itdy->second.minheight);
            for(int i = minx; i < maxx; i++ )
            {
                for(int j = miny; j < maxy; j++)
                {
                    item_tmep.insert(KEY(i, j, 0));
                }
            }
            itdy++;
        }
        std::unordered_set<KEY,HashFunc, EqualKey>::iterator lgtg = item_tmep.begin();
        keymap ::iterator rearr;
        while (lgtg != item_tmep.end())
        {
            rearr = map.static_map.find(KEY(lgtg->x, lgtg->y, 0));
            if (rearr != map.static_map.end())
            {
                    if(mint < rearr->second.maxheight + 1.5)
                    {
                         result[k_number] = 1;
                    }
            }
            lgtg++;
        }
        itl++;
        k_number ++;
    }
    return result;

}

std::vector<uint8_t > pointcloud_common::occupied_grid3d::ODFilterByPlanemodel(Feature_map& map) {
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints2;
    std::vector<keymap>::iterator itl = map.dynamatic_grid.begin();
    std::vector<uint8_t > result(map.dynamatic_grid.size(), 0);
    int k_number = 0;
    while (itl != map.dynamatic_grid.end()) {
        keymap::iterator itdy = itl->begin();
        while (itdy != itl->end()) {
          std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point3D>(itdy->first.x * resolution[0], itdy->first.y * resolution[1], itdy->first.z * resolution[2]);//继承兼容性原则
          CandPoints2.push_back(CandPt);
          itdy++;
        }
        itl++;
        k_number ++;
    }
    if ( k_number < 20) std::cout<< "can not filter by planemodel, because there are too few points"<< std::endl;
    else
    {
        GRANSAC::RANSAC<SurfaceModel, 3> Estimator2;
        Estimator2.Initialize(0.5, 50); // Threshold, iterations
        Estimator2.Estimate(CandPoints2);
        auto bestmodel2 = Estimator2.GetBestModel();
        Eigen::Vector3d pae_f2(bestmodel2->getm_a(), bestmodel2->getm_b(), bestmodel2->getm_c());
        Eigen::Vector4d pae_f3(bestmodel2->getm_a(), bestmodel2->getm_b(), bestmodel2->getm_c(), bestmodel2->getm_e());
        pae_f2.head(3).normalize();
        double lenth = pae_f2.dot(Eigen::Vector3d(0, 0, 1));
        double lenth2 = sqrt(1- pow(lenth, 2));
        double angle = atan2(lenth2,  lenth);
        std::cout<< "the angle " <<angle <<std::endl;
        int k_number2 = 0;
        if ((abs(angle - 0)<0.18) ||(abs(angle - 3.1415)<0.18))
        {
            std::vector<keymap>::iterator itl2 = map.dynamatic_grid.begin();
            while (itl2 != map.dynamatic_grid.end()) {
                keymap::iterator itdy = itl2->begin();
                double height;
                uint8_t  flag = 0;
                while( (itdy != itl2->end())&&(flag == 0) ){
                    height = -(itdy->second.x * this->resolution[0] * pae_f3[0] + itdy->second.y * this->resolution[1] * pae_f3[1] +  pae_f3[3]) / pae_f3[2];
                    if (abs(height - itdy->second.minheight ) < 1) {
                        flag = 1;
                        result[k_number2] = 1;
                    }
                    itdy++;
                }
                itl2++;
                ++k_number2;
            }
        }
    }

    return  result;

}
pointcloud_common::Feature_map pointcloud_common::occupied_grid3d::GetFeatuerMap(){
    std::unordered_map<int, pointfeature>::iterator it;
    it = this->result_map.begin();
    cluster_map.clear();
    while(it != result_map.end())
    {
        grid_feature feature_item;
        Eigen::Vector3f position = GetPositionByindex(it->first);
        std::vector<std::map<float, int> > result_temp = Cluster1D(it->second.totalpoints, this->cluster_value);
        if( result_temp.size() <= 2)
        {
            for(int i = 0; i < result_temp.size(); i++)
            {
                if (result_temp[i].size() <= 1)
                {
                    feature_item.x = position[0] / resolution[0];
                    feature_item.y = position[1] / resolution[1];
                    feature_item.maxheight = result_temp[i].begin()->first + 0.1;
                    feature_item.minheight = result_temp[i].begin()->first;
                    feature_item.total_points = result_temp[i].size();
                    feature_item.class_index = 0;
                }
                else
                {
                    feature_item.x = position[0] / resolution[0];
                    feature_item.y = position[1] / resolution[1];
                    feature_item.maxheight = result_temp[i].rbegin()->first;
                    feature_item.minheight = result_temp[i].begin()->first;
                    feature_item.total_points = result_temp[i].size();
                    feature_item.class_index = 0;
                }
                float z = floor(feature_item.minheight / resolution[2]) ;
                it->second.cluster_feature.push_back(feature_item.minheight);
                cluster_map.insert(std::pair<KEY, grid_feature>(KEY(feature_item.x, feature_item.y, z), feature_item));
            }
        }
        else
        {
            std::vector<float> min, max;
            grid_feature feature_item;
            for(int j = 0;j < result_temp.size()-1; j++)
            {
                min.push_back(result_temp[j].begin()->first);
                max.push_back(result_temp[j].rbegin()->first);
                feature_item.total_points += result_temp[j].size();
            }
            feature_item.x = position[0] / resolution[0];
            feature_item.y = position[1] / resolution[1];
            feature_item.maxheight = max.back() ;
            feature_item.minheight = min.front();
            feature_item.class_index = 0;
            float z = floor(feature_item.minheight / resolution[2]) ;
            it->second.cluster_feature.push_back(feature_item.minheight);
            cluster_map.insert(std::pair<KEY, grid_feature>(KEY(feature_item.x, feature_item.y, z), feature_item));
            feature_item.x = position[0] / resolution[0];
            feature_item.y = position[1] / resolution[1];
            feature_item.maxheight = result_temp.back().rbegin()->first;
            feature_item.minheight = result_temp.back().begin()->first;
            feature_item.total_points = result_temp.back().size();
            feature_item.class_index = 1;
            z = floor(feature_item.minheight / resolution[2]) ;
            it->second.cluster_feature.push_back(feature_item.minheight);
            cluster_map.insert(std::pair<KEY, grid_feature>(KEY(feature_item.x, feature_item.y, z), feature_item));
        }
        it++;
    }
    std::vector<std::vector<int> > result_cluster = ClusterRegionExpand::grid_grouping(Eigen::Vector3i(3,3,3),3,cluster_map,
                                                                                       Eigen::Vector3f(400, 200,1000), result_map, resolution);
    Feature_map result_map = this->FlyingObjectFilter(result_cluster, cluster_map);

    return result_map;
}




std::vector<std::map<float, int> > pointcloud_common::occupied_grid3d::Cluster1D(std::map<float, int> items, float diff_max) {
    std::vector<std::map<float, int> > result;
    std::map<float, int> map_item;
    std::map<float, int>::iterator it = items.begin();
    map_item.insert(std::pair<float, int>(it->first, it->second));
    it++;
    while(it != items.end())
    {
        if(abs(map_item.rbegin()->first - it->first) < diff_max)
        {
            map_item.insert(std::pair<float, int>(it->first, it->second));
        }
        else
        {
            result.push_back(map_item);
            map_item.clear();
            map_item.insert(std::pair<float, int>(it->first, it->second));
        }
        it++;
    }
    result.push_back(map_item);
    return result;
}
std::vector<std::vector<int> > pointcloud_common::ClusterRegionExpand::grid_grouping ( Eigen::Vector3i num, int minpts,  keymap& group_num,
                                                                                       Eigen::Vector3f mapSize, std::unordered_map<int, pointfeature >& result_map,
                                                                                       Eigen::Vector3f resolution)
{
    int clusterid = 2;
    std::unordered_map<KEY, grid_feature, HashFunc, EqualKey>::iterator itg = group_num.begin();
    std::vector<std::vector<int> >result;
    int cluster_total;
    while(itg != group_num.end())
    {
        std::vector<int> item;
        if(itg->second.cluster_index == (uint32_t)1 )
        {
            cluster_total = dbscan_expand( num,  minpts,  itg->first,  clusterid, group_num, mapSize, result_map, resolution);
            if( cluster_total > 0)
            {
                item.push_back(clusterid);
                item.push_back(cluster_total);
                result.push_back(item);
                clusterid += 1;
            }
        }
        itg++;
    }
    std::cout << "clusterid  "<<clusterid << std::endl;
    return result;
}
int pointcloud_common::ClusterRegionExpand::dbscan_distinguish_( Eigen::Vector3i num, KEY pointnum, std::vector< KEY> & densty,
                                                                 keymap& group_num, Eigen::Vector3f mapSize,
                                                                 std::unordered_map<int, pointfeature >& result_map, Eigen::Vector3f resolution)
{
    int min_x = 0, max_x = 0, min_y = 0, max_y = 0, area_sum = 0;
    min_x = std::max(float(0), pointnum.x - num[0]);
    max_x = std::min(float(mapSize[0] ), pointnum.x + num[0]);
    min_y = std::max(float(-mapSize[1]/2), pointnum.y - num[1]);
    max_y = std::min(float(mapSize[1]/2), pointnum.y + num[1]);
    keymap::iterator it3 = group_num.find( pointnum );
    float min_z = it3->second.minheight - 1;
    float max_z = it3->second.maxheight + 1;
    const KEY key = pointnum;
    for (int i = min_x; i< max_x ; i++)
    {
        for (int j = min_y; j< max_y ; j++)
        {
            int index = float(i) * mapSize[1] + float(j);
            std::unordered_map<int, pointfeature>::iterator it2 = result_map.find(index);
            if( it2 != result_map.end())
            {
                for(int k = 0; k < it2->second.cluster_feature.size(); k++)
                {
                    int index_height = floor(it2->second.cluster_feature[k] /  resolution[2]);
                    keymap::iterator it = group_num.find( KEY(float(i), float(j), float(index_height)) );
                    if(it != group_num.end())
                    {
                        if(std::min(it->second.maxheight, max_z) > std::max(it->second.minheight, min_z))
                        {
                            area_sum += it->second.total_points ;
                            if(it->second.cluster_index <= 1)
                            {
                                densty.push_back(KEY(i, j, index_height));
                            }
                        }
                    }
                }
            }
        }
    }
    return  area_sum;
}
int pointcloud_common::ClusterRegionExpand::dbscan_distinguish( Eigen::Vector3i num, KEY pointnum, std::vector< KEY> & densty,
                                                                keymap& group_num, Eigen::Vector3f mapSize, std::unordered_map<int, pointfeature >& result_map,
                                                                Eigen::Vector3f resolution)
{
    int min_x = 0, max_x = 0, min_y = 0, max_y = 0, area_sum = 0, min_z, max_z;
    min_x = pointnum.x - num[0];
    max_x = pointnum.x + num[0];
    min_y = pointnum.y - num[1];
    max_y = pointnum.y + num[1];
    const KEY key = pointnum;
    std::unordered_map<int, pointfeature >::iterator it2 = result_map.find( (int)((int16_t)pointnum.x << 16) | (uint16_t) pointnum.y);
    float hight_t = it2->second.totalpoints.rbegin()->first;
    for (int i = min_x; i< max_x ; i++)
    {
        for (int j = min_y; j< max_y ; j++)
        {
                std::unordered_map<int, pointfeature >::iterator it = result_map.find( (int)(i << 16) | (uint16_t) j);
                if( it != result_map.end())
                {
                    float height = (int16_t)(it->second.totalpoints.rbegin()->first / resolution[2]);
                    KEY key(i, j, height);
                    keymap::iterator it3 = group_num.find( key);
                    if(it3 != group_num.end())
                    {
                        area_sum += it->second.totalpoints.size();
                        if(((it->second.totalpoints.rbegin()->first - hight_t )<= 0.1) &&(it3->second.cluster_index <= 1))
                        {
                            densty.push_back(KEY(i, j, height));
                        }
                    }
                }
        }
    }
    return  area_sum;
}
int pointcloud_common::ClusterRegionExpand::dbscan_expand( Eigen::Vector3i num, int minpts, KEY pointnum, int clusterId,
                                                           keymap& group_num, Eigen::Vector3f mapSize, std::unordered_map<int, pointfeature >& result_map,
                                                           Eigen::Vector3f resolution)
{
    std::vector< KEY>  densty, queryresults;
    int efi = 0;
    std::unordered_map<KEY, grid_feature, HashFunc,EqualKey>::iterator it = group_num.find( pointnum );
    if (dbscan_distinguish(num, pointnum, densty,group_num, mapSize, result_map, resolution)<= minpts)
    {

        it->second.cluster_index = 0;
        return 0;
    }
    else
    {
        it->second.cluster_index  = clusterId;
        for (int i=0; i< densty.size(); i++)
        {
            it = group_num.find(densty[i]);
            it->second.cluster_index = clusterId;
            efi ++;
        }
        while (densty.size()>0)
        {
            if(dbscan_distinguish(num, densty[0], queryresults,group_num, mapSize, result_map, resolution)> 0)
            {

                for (int i=0; i< queryresults.size(); i++)
                {
                    it = group_num.find(queryresults[i]);
                    if (it->second.cluster_index == 1)
                    {
                        densty.push_back(queryresults[i]);
                        it->second.cluster_index = clusterId;
                        efi ++ ;
                    }
                    else if(it->second.cluster_index == 0)
                    {
                        densty.push_back(queryresults[i]);
                        it->second.cluster_index = clusterId;
                        efi ++;
                    }
                }
            }
            std::vector< KEY >().swap(queryresults);
            densty.erase(densty.begin());
        }
    }
    return efi;
}

pointcloud_common::Bbox::ODvector_type pointcloud_common::Bbox::getboundingbox( std::vector<keymap> object,Eigen::Vector3f resolution)
{
    ODvector_type boxes;
    for(int i = 0; i < object.size(); i++)
    {
        keymap::iterator it = object[i].begin();
        float minx = it->first.x * resolution[0];
        float miny = it->first.y * resolution[1];
        float maxx = it->first.x * resolution[0];
        float maxy = it->first.y * resolution[1];
        float minz = it->second.minheight;
        float maxz = it->second.maxheight;
        int total  = 0;
        while(it != object[i].end())
        {
            minx = std::min(minx, it->first.x * resolution[0]);
            miny = std::min(miny, it->first.y * resolution[1]);
            maxx = std::max(maxx, it->first.x * resolution[0]);
            maxy = std::max(maxy, it->first.y * resolution[1]);
            maxz = std::max(maxz, it->second.maxheight);
            minz = std::min(minz, it->second.minheight);
            total += it->second.total_points;
            ++it;
        }
        Bbox::BoxFeature item;
        item.size[0] = maxx - minx;
        item.size[1] = maxy - miny;
        item.size[2] = maxz - minz;
        item.center[0] = (maxx + minx) / 2.0;
        item.center[1] = (maxy + miny) / 2.0;
        item.center[2] = (maxz + minz) / 2.0;
        item.total_points = total;
        boxes.push_back(item);
    }
    return boxes;

}