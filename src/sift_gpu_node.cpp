#include <fstream>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <GL/gl.h>

#include "sift_gpu/sift_gpu_node.hpp"

SIFTGPUNode::SIFTGPUNode(int argc, char** argv)
{
    if (readSIFT((ros::package::getPath("sift_gpu") + "/config/map.txt").c_str()) != 0)
    {
        ROS_ERROR("SIFT MAP NOT FOUND!");
        exit(EXIT_FAILURE);
    }
    
    sift.ParseParam(argc, argv);
    if ( sift.CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED )
    {
        ROS_ERROR("SiftGPU is not supported!");
        exit(EXIT_FAILURE);
    }

    matcher = SiftMatchGPU(256);
    if(matcher.VerifyContextGL() == 0)
    {
        ROS_ERROR("SiftMatchGPU is not supported!");
        exit(EXIT_FAILURE);
    }
    matcher.SetDescriptors(1, train_kps.size(), &train_des[0]);

    ros::NodeHandle nh;
    sift_match_server = nh.advertiseService("/sift_gpu/sift_match", &SIFTGPUNode::siftMatchHandler, this);
}

SIFTGPUNode::~SIFTGPUNode()
{
}

int SIFTGPUNode::readSIFT(const char* filename)
{
    std::fstream file;
    file.open(filename, std::ios::in);
    if (file.is_open() != true) return -1;
    int num;
    std::string tp;
    std::getline(file, tp);
    std::sscanf(tp.c_str(), "%d %d", &width, &height);
    std::getline(file, tp);
    std::sscanf(tp.c_str(), "%d", &num);
    train_kps.resize(num);
    train_des.resize(128 * num);
    for (int i = 0; i < num; i++)
    {
        std::getline(file, tp);
        std::sscanf(tp.c_str(), "%f %f %f %f", &train_kps[i].x, &train_kps[i].y, &train_kps[i].s, &train_kps[i].o);
        
        std::getline(file, tp);
        std::vector<std::string> s;
        boost::split(s, tp, boost::is_any_of(" "));
        for (int j = 0; j < 128; j++)
        {
            train_des[128 * i + j] = std::atof(s[j].c_str());
        }
    }
    file.close();
    return 0;
}

bool SIFTGPUNode::siftMatchHandler(sift_gpu::SIFTMatchRequest& req, sift_gpu::SIFTMatchResponse& res)
{
    res.width = width;
    res.height = height;

    timer.restart();
    sift.RunSIFT(req.query_img.width, req.query_img.height, req.query_img.data.data(), GL_RGB, GL_UNSIGNED_BYTE);
    res.extract_cost_ms = timer.elapsed() * 1000;
    res.nfeature = sift.GetFeatureNum();

    std::vector<SiftGPU::SiftKeypoint> query_kps(sift.GetFeatureNum());
    std::vector<float> query_des(128 * sift.GetFeatureNum());
    sift.GetFeatureVector(&query_kps[0], &query_des[0]);

    matcher.SetDescriptors(0, query_kps.size(), &query_des[0]);

    int match_buff[256][2];
    timer.restart();
    int nmatch = matcher.GetSiftMatch(256, match_buff, 0.7f, 0.6f);
    res.match_cost_ms = timer.elapsed() * 1000;
    res.nmatch = nmatch;
    res.query_kps.resize(2 * nmatch);
    res.train_kps.resize(2 * nmatch);
    for (int i = 0; i < nmatch; i++)
    {
        SiftGPU::SiftKeypoint query_kp = query_kps[match_buff[i][0]];
        SiftGPU::SiftKeypoint train_kp = train_kps[match_buff[i][1]];
        res.query_kps[2 * i] = query_kp.x;
        res.query_kps[2 * i + 1] = query_kp.y;
        res.train_kps[2 * i] = train_kp.x;
        res.train_kps[2 * i + 1] = train_kp.y;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sift_gpu_node");

    SIFTGPUNode sift_gpu_node(argc, argv);
    ros::spin();

    return 0;
}