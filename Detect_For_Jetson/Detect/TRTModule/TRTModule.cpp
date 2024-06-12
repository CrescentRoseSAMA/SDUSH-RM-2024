#include <eigen3/Eigen/Dense>
#include <fstream>
#include <filesystem>
#include <logger.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <fmt/format.h>
#include <fmt/color.h>
#include "../Utils/Utils.hpp"
#include <vector>
#include <algorithm>
#include <opencv4/opencv2/core/eigen.hpp>
#include "TRTModule.hpp"
using namespace nvinfer1;
using namespace sample;
using namespace std;
#define TRT_ASSERT(expr)                                                      \
    do                                                                        \
    {                                                                         \
        if (!(expr))                                                          \
        {                                                                     \
            fmt::print(fmt::fg(fmt::color::red), "assert fail: '" #expr "'"); \
            exit(-1);                                                         \
        }                                                                     \
    } while (0)

static inline size_t get_dims_size(const Dims &dims)
{
    size_t sz = 1;
    for (int i = 0; i < dims.nbDims; i++)
        sz *= dims.d[i];
    return sz;
}

/*下面大概使用了形参包展开以及右值引用的方式，有时间看看相关知识*/
template <class F, class T, class... Ts>
T reduce(F &&func, T x, Ts... xs)
{
    if constexpr (sizeof...(Ts) > 0)
    {
        return func(x, reduce(std::forward<F>(func), xs...));
    }
    else
    {
        return x;
    }
}

template <class T, class... Ts>
T reduce_max(T x, Ts... xs)
{
    return reduce([](auto &&a, auto &&b)
                  { return std::max(a, b); },
                  x, xs...);
}

template <class T, class... Ts>
T reduce_min(T x, Ts... xs)
{
    return reduce([](auto &&a, auto &&b)
                  { return std::min(a, b); },
                  x, xs...);
}

/*
 *  检测两个bounding box是否重叠，方法就是算交并比
 */
static inline bool is_overlap(const float pts1[8], const float pts2[8])
{
    cv::Rect2f bbox1, bbox2;
    bbox1.x = reduce_min(pts1[0], pts1[2], pts1[4], pts1[6]);
    bbox1.y = reduce_min(pts1[1], pts1[3], pts1[5], pts1[7]);
    bbox1.width = reduce_max(pts1[0], pts1[2], pts1[4], pts1[6]) - bbox1.x;
    bbox1.height = reduce_max(pts1[1], pts1[3], pts1[5], pts1[7]) - bbox1.y;
    bbox2.x = reduce_min(pts2[0], pts2[2], pts2[4], pts2[6]);
    bbox2.y = reduce_min(pts2[1], pts2[3], pts2[5], pts2[7]);
    bbox2.width = reduce_max(pts2[0], pts2[2], pts2[4], pts2[6]) - bbox2.x;
    bbox2.height = reduce_max(pts2[1], pts2[3], pts2[5], pts2[7]) - bbox2.y;
    return (bbox1 & bbox2).area() > 0;
}

/*
 * 交爷自己写的argmax函数，找到一段序列中最大值的下标并返回
 */
static inline int argmax(const float *ptr, int len)
{
    int max_arg = 0;
    for (int i = 1; i < len; i++)
    {
        if (ptr[i] > ptr[max_arg])
            max_arg = i;
    }
    return max_arg;
}

constexpr float inv_sigmoid(float x)
{
    return -std::log(1 / x - 1);
}

constexpr float sigmoid(float x)
{
    return 1 / (1 + std::exp(-x));
}
TRTModule::TRTModule()
{
}
TRTModule::TRTModule(const std::string &onnx_file)
{
    /*读入onnx模型*/
    std::filesystem::path onnx_file_path(onnx_file);
    auto cache_file_path = onnx_file_path;
    cache_file_path.replace_extension("cache"); // 路径扩展名从onnx换成cache
    /*判断cache路径下由于cache文件，有就用cache进行构建，否则使用onnx构建*/
    if (std::filesystem::exists(cache_file_path))
    {
        build_engine_from_cache(cache_file_path.c_str());
    }
    else
    {
        build_engine_from_onnx(onnx_file_path.c_str());
        cache_engine(cache_file_path.c_str());
    }

    TRT_ASSERT((context = engine->createExecutionContext()) != nullptr);
    /*获取输入输出的index，用于获取输入输出的维度*/
    /*
     *  input_Dims = 1 × 384 × 640 × 3
     *  output_Dims = 1 × 128 × 20
     *  不难看出，输出的第二维128对应的是topk，因此topk可以认为是限定一次识别最多可以识别多少目标
     *  而最后一维是
     */
    TRT_ASSERT((input_idx = engine->getBindingIndex("input")) == 0);
    TRT_ASSERT((output_idx = engine->getBindingIndex("output-topk")) == 1);
    auto input_dims = engine->getBindingDimensions(input_idx);
    auto output_dims = engine->getBindingDimensions(output_idx);
    /*获取输入输出的数据量大小*/
    input_sz = get_dims_size(input_dims);
    output_sz = get_dims_size(output_dims);
    /*从cuda中申请推理输入输出需要的设备内存，用于后续在cuda上的推理*/
    TRT_ASSERT(cudaMalloc(&device_buffer[input_idx], input_sz * sizeof(float)) == 0);
    TRT_ASSERT(cudaMalloc(&device_buffer[output_idx], output_sz * sizeof(float)) == 0);
    /*创建一个cuda流，用于管理GPU调度，推理时使用*/
    TRT_ASSERT(cudaStreamCreate(&stream) == 0);
    /*申请一块内存用于存储输出*/
    output_buffer = new float[output_sz];
    TRT_ASSERT(output_buffer != nullptr);
}

TRTModule::~TRTModule()
{
    delete[] output_buffer;
    cudaStreamDestroy(stream);
    cudaFree(device_buffer[output_idx]);
    cudaFree(device_buffer[input_idx]);
}

void TRTModule::build_engine_from_onnx(const std::string &onnx_file)
{
    /*以下是使用tensorrt构建onnx网络的经典流程，基本上是通用的*/
    std::cout << "[INFO]: build engine from onnx" << std::endl;
    auto builder = createInferBuilder(sample::gLogger);
    TRT_ASSERT(builder != nullptr);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);
    TRT_ASSERT(network != nullptr);
    auto parser = nvonnxparser::createParser(*network, sample::gLogger);
    TRT_ASSERT(parser != nullptr);
    parser->parseFromFile(onnx_file.c_str(), static_cast<int>(ILogger::Severity::kINFO));
    auto yolov5_output = network->getOutput(0);
    /*下面应该是一些后处理网络?*/
    /*具体目标为将输出的置信度部分提取，并提取最大的topk个数作为输出*/
    auto slice_layer = network->addSlice(*yolov5_output, Dims3{0, 0, 8}, Dims3{1, 25200, 1}, Dims3{1, 1, 1}); // 从输出可以看到输出的原定维度为1 × 15120 × 20
    auto yolov5_conf = slice_layer->getOutput(0);
    auto shuffle_layer = network->addShuffle(*yolov5_conf);
    shuffle_layer->setReshapeDimensions(Dims2{1, 25200});
    yolov5_conf = shuffle_layer->getOutput(0);
    auto topk_layer = network->addTopK(*yolov5_conf, TopKOperation::kMAX, TOPK_NUM, 1 << 1);
    auto topk_idx = topk_layer->getOutput(1);
    auto gather_layer = network->addGather(*yolov5_output, *topk_idx, 1);
    gather_layer->setNbElementWiseDims(1);
    auto yolov5_output_topk = gather_layer->getOutput(0);

    /*绑定输入输出，防止被优化掉*/
    yolov5_output_topk->setName("output-topk");
    network->getInput(0)->setName("input");
    network->markOutput(*yolov5_output_topk);
    network->unmarkOutput(*yolov5_output);
    auto config = builder->createBuilderConfig();
    config->setFlag(nvinfer1::BuilderFlag::kFP16); // 设置精度计算
    /*
    if (builder->platformHasFastFp16())
    {
        std::cout << "[INFO]: platform support fp16, enable fp16" << std::endl;
        config->setFlag(BuilderFlag::kFP16);
    }
    else
    {
        std::cout << "[INFO]: platform do not support fp16, enable fp32" << std::endl;
    }
    */
    size_t free, total;
    cuMemGetInfo(&free, &total);
    std::cout << "[INFO]: total gpu mem: " << (total >> 20) << "MB, free gpu mem: " << (free >> 20) << "MB" << std::endl;
    std::cout << "[INFO]: max workspace size will use all of free gpu mem" << std::endl;
    config->setMaxWorkspaceSize(free);
    TRT_ASSERT((engine = builder->buildEngineWithConfig(*network, *config)) != nullptr);
    config->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();
}

void TRTModule::build_engine_from_cache(const std::string &cache_file)
{
    std::cout << "[INFO]: build engine from cache" << std::endl;
    std::ifstream ifs(cache_file, std::ios::binary);
    ifs.seekg(0, std::ios::end);
    size_t sz = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    auto buffer = std::make_unique<char[]>(sz);
    ifs.read(buffer.get(), sz);
    auto runtime = createInferRuntime(gLogger);
    TRT_ASSERT(runtime != nullptr);
    TRT_ASSERT((engine = runtime->deserializeCudaEngine(buffer.get(), sz)) != nullptr);
    runtime->destroy();
}

void TRTModule::cache_engine(const std::string &cache_file)
{
    auto engine_buffer = engine->serialize();
    TRT_ASSERT(engine_buffer != nullptr);
    std::ofstream ofs(cache_file, std::ios::binary);
    ofs.write(static_cast<const char *>(engine_buffer->data()), engine_buffer->size());
    engine_buffer->destroy();
}

/*
 *  中文为自己加的注释，英文为交爷给的注释
 */

std::vector<bbox_t> TRTModule::operator()(const cv::Mat &src) const
{
    Timer T;
    // pre-process [bgr2rgb & resize]
    cv::Mat x;
    float fx = (float)src.cols / 640.f, fy = (float)src.rows / 384.f; // 求出resize前的比例，用于之后将四点从标准输入上的尺寸转换到输入上的尺寸
    /*图像进行转换成RGB，然后Resize*/
    cv::cvtColor(src, x, cv::COLOR_BGR2RGB);
    if (src.cols != 640 || src.rows != 384)
    {
        cv::resize(x, x, {640, 384});
    }
    x.convertTo(x, CV_32F);
    // run model
    cudaMemcpyAsync(device_buffer[input_idx], x.data, input_sz * sizeof(float), cudaMemcpyHostToDevice, stream);
    /*
    cudaMemcpyAsync(device_buffer[input_idx], channels[0].data, input_sz * sizeof(float) / 3, cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(device_buffer[input_idx] + channels[0].elemSize(), channels[1].data, input_sz * sizeof(float) / 3, cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(device_buffer[input_idx] + channels[1].elemSize(), channels[2].data, input_sz * sizeof(float) / 3, cudaMemcpyHostToDevice, stream);
    */
    this->context->setOptimizationProfileAsync(0, stream);
    this->context->setTensorAddress("input", device_buffer[input_idx]);
    this->context->setTensorAddress("output-topk", device_buffer[output_idx]);
    bool success = this->context->enqueueV3(stream);
    cudaMemcpyAsync(output_buffer, device_buffer[output_idx], output_sz * sizeof(float), cudaMemcpyDeviceToHost,
                    stream);
    cudaStreamSynchronize(stream);

    // post-process [nms]
    std::vector<bbox_t> rst;
    rst.reserve(TOPK_NUM);
    std::vector<uint8_t> removed(TOPK_NUM);
    auto input_dims = engine->getBindingDimensions(input_idx);
    auto output_dims = engine->getBindingDimensions(output_idx);

    for (int i = 0; i < TOPK_NUM; i++)
    {
        auto *box_buffer = output_buffer + i * 20; // 20->23

        /*第8位推断为是<装甲板>的置信度(未经过sigmoid归一化)，而keep_thres为置信阈值，通过sigmoid的反函数来求出未经过sigmoid的置信度*/
        /*两者相比较，筛掉置信度低于置信阈值的样本*/
        if (box_buffer[8] < inv_sigmoid(KEEP_THRES))
            break;
        /*判断*/
        if (removed[i]) // 只处理没被romove的样本
            continue;
        /*向rst中填入一个空成员*/
        rst.emplace_back();
        /*取出最后一个成员，也就是最后一个空成员*/
        auto &box = rst.back();
        /*将本轮循环样本的前8位数据传给box的pts，即像素四点，每点(x,y),四点为 4 * 2 = 8 个数据*/
        memcpy(&box.pts, box_buffer, 8 * sizeof(float));
        /*将四点按开头算的比例映射到原图像上*/
        for (auto &pt : box.pts)
            pt.x *= fx, pt.y *= fy;
        /*读取buffer不同的数据段通过argmax来获取该装甲板的信息*/
        box.confidence = sigmoid(box_buffer[8]);
        box.color_id = argmax(box_buffer + 9, 4);
        box.tag_id = argmax(box_buffer + 13, 7);
        /*通过计算IOU来进行非极大值抑制*/
        /*
         *  可以注意到，由于我们检测装甲板的任务很简单，目标少，因此直接采用有交集就筛掉的原则，不再用传统NMS那一套
         *  比如先对置信度排序，然后遍历该种类的所有box，计算IOU，筛掉IOU大于阈值或者置信度小于阈值的box
         */
        for (int j = i + 1; j < TOPK_NUM; j++) // 遍历每一个样本
        {
            auto *box2_buffer = output_buffer + j * 20;
            if (box2_buffer[8] < inv_sigmoid(KEEP_THRES)) // 过掉置信度小的样本
                break;
            if (removed[j])
                continue;
            if (is_overlap(box_buffer, box2_buffer)) // 如果有交集，直接将其remove
                removed[j] = true;
        }
    }

    return rst;
}
