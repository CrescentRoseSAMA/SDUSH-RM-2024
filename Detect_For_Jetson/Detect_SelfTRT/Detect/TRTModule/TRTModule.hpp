//
// Created by xinyang on 2021/4/8.
//

#ifndef _ONNXTRTMODULE_HPP_
#define _ONNXTRTMODULE_HPP_

#include <opencv2/core.hpp>
#include "/home/ruby/Tensorrt/trt/include/NvInfer.h"

const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
struct alignas(4) bbox_t
{
    cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]
    float confidence;
    int color_id; // 0: blue, 1: red, 2: gray
    int tag_id;   // 0: guard, 1-5: number, 6: base

    bool operator==(const bbox_t &) const = default;
    bool operator!=(const bbox_t &) const = default;
};
/*
 * 四点模型
 */
class TRTModule
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.6f;

public:
    explicit TRTModule(const std::string &onnx_file);

    ~TRTModule();

    TRTModule();

    TRTModule(const TRTModule &) = delete;

    TRTModule operator=(const TRTModule &) = delete;

    std::vector<bbox_t> operator()(const cv::Mat &src) const;

private:
    void build_engine_from_onnx(const std::string &onnx_file);

    void build_engine_from_cache(const std::string &cache_file);

    void cache_engine(const std::string &cache_file);

    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    mutable void *device_buffer[2];
    float *output_buffer;
    /*
     *  对于一个buffer，由bbox_t结构定义推断，应该有
     *  4点(8)+ 7种装甲板置信度 + 4种颜色置信度 + 1种置信度
     *  8 + 7 + 4 + 1 = 20
     *  可以知道一个buffer是有20个成员的
     *  再通过TRTModule.cpp推断得出buffer的结构
     *  0-7 四点的x,y
     *  8 是装甲板的置信度
     *  9-12 四种颜色的置信度(红蓝灰紫)
     *  13-19 七种装甲板的置信度
     */
    cudaStream_t stream;
    int input_idx = 0, output_idx = 1;
    size_t input_sz, output_sz;
};
#endif /* _ONNXTRTMODULE_HPP_ */
