#ifndef SEARCH_H
#define SEARCH_H

//#include </usr/local/cuda-6.5/include/CL/cl.h>
#include <CL/cl.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <fstream>

#define MAX_LOG_SIZE (0x100000)
#define MATCHES_BUFFER_SIZE 10000
#define VERBOSE false

class Search
{
public:
    Search();
    ~Search();
    void setImage(cv::Mat img);
    void buildProgram(const char* clPath, cl_int win, cl_double p);
    void runProgram(cv::Mat img);
    void* readOutput();
    unsigned int* readMatchesOutput(unsigned int numMatches);
    int readMatchesIndexOutput();
    cv::Mat getInputImage();
private:
    cv::Mat image;
    size_t imageWidth;
    size_t imageHeight;
    size_t imageSize;
    const char* clPath;
    cl_int win;
    cl_double p;
    cl_int matchesIndex;

    FILE* programHandle;
    char *programBuffer;
    size_t programSize;
    cl_program program;
    cl_context context;
    cl_uint deviceCount;
    cl_device_id* devices;
    cl_int err;
    cl_kernel kernel;
    cl_mem clImage;
    cl_mem clResult;
    cl_mem clMatchIndex;
    cl_mem clMatch;
    cl_command_queue queue;
};


#endif // SEARCH_H
