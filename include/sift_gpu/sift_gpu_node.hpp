#include <mutex>
#include <ros/ros.h>
#include <SiftGPU.h>
#include <boost/timer.hpp>

#include "sift_gpu/SIFTMatch.h"

class SIFTGPUNode
{
private:
    int width, height;
    SiftGPU sift;
    SiftMatchGPU matcher;
    boost::timer timer;
    std::mutex sift_lock;
    std::vector<SiftGPU::SiftKeypoint> train_kps;
    std::vector<float> train_des;
    ros::ServiceServer sift_match_server;

    int readSIFT(const char* filename);
    bool siftMatchHandler(sift_gpu::SIFTMatchRequest& req, sift_gpu::SIFTMatchResponse& res);
public:
    SIFTGPUNode(int argc, char** argv);
    ~SIFTGPUNode();
};
