#ifndef _TRTFRAME_HPP_
#define _TRTFRAME_HPP_
#include <opencv4/opencv2/core.hpp>
#include <vector>
#include <map>
#include "/home/ruby/Tensorrt/trt/include/NvInfer.h"

/*
 *  @brief 定义box的类型
 *
 *  @note   xyxyxyxy: [x1, y1, x2, y2, x3, y3, x4, y4] 逆时针四点
 *          xyhw_center: [x, y, w, h, center_x, center_y]  中心点坐标和宽高
 *          xyhw_topl: [x, y, w, h, top_left_x, top_left_y]    左上角坐标和宽高
 *          xyxy: [x1, y1, x2, y2] 左上角坐标和右下角坐标
 *
 */
enum box_type
{
    xyxyxyxy = 1,
    xyhw_center,
    xyhw_topl,
    xyxy
};

/*
 *  @brief 定义推理参数
 *
 *  @note  该结构体用于定义推理参数，包括预处理、后处理等参数
 *         topk: 是否使用topk输出
 *         topk_num: topk输出的最大数量
 *         cvt_code: 图像预处理时的转换代码
 *         input_size: 推理输入尺寸
 *         normalize: 是否对图像进行归一化
 *         hwc2chw: 是否将图像从HWC转为CHW
 *         type: 推理输出的box类型
 *         conf_pos: 推理输出中置信度的位置
 *         box_pos: 推理输出中box信息的起始位置
 *         conf_thre: 置信度阈值
 *         iou_thre: IOU阈值
 *         has_sigmoid: 网络的输出是否经过sigmoid激活
 *         classes_info: 类别信息
 */

struct InferParam
{
    /*infer param*/
    bool topk;
    int topk_num;

    /*preprocess param*/
    int cvt_code;
    cv::Size input_size;
    bool normalize;
    bool hwc2chw;

    /*postprocess param*/
    box_type type;
    int conf_pos;
    int box_pos;
    float conf_thre;
    float iou_thre;
    bool has_sigmoid;

    /*class info*/
    struct class_info
    {
        std::vector<std::string> classes_names;
        int classes_offset;
        int classes_num;
    };
    std::vector<class_info> classes_info;
};

/*
 *  @brief  3维向量类型
 *
 *  @note  3维向量类型，用于表示输出的tensor的维度
 */
struct Dim3d
{
    int dim1;
    int dim2;
    int dim3;
    Dim3d(int dim1_, int dim2_, int dim3_)
    {
        dim1 = dim1_;
        dim2 = dim2_;
        dim3 = dim3_;
    };
    Dim3d(nvinfer1::Dims &dim)
    {
        dim1 = dim.d[0];
        dim2 = dim.d[1];
        dim3 = dim.d[2];
    }
    Dim3d()
    {
        dim1 = -1;
        dim2 = -1;
        dim3 = -1;
    }
};

struct BoxInfo
{
    std::vector<std::pair<int, std::string>> classes;
    float confidence;
    std::vector<float> box;
    BoxInfo()
    {
        classes.clear();
        confidence = 0.0f;
        box.clear();
    }
    BoxInfo(float conf, float *box_data, box_type type) : confidence(conf), box(box_data, box_data + (type == xyxyxyxy ? 8 : 4))
    {
    }
};

/*
 *  @brief 基于TensorRT的onnx推理框架类
 *
 *  @note  该类主要用于对TensorRT推理框架的封装，包括创建推理引擎、序列化、保存、推理、NMS等功能
 */
class TRTFrame
{
private:
    Dim3d outputDims;
    const std::string input_name = "input";
    const std::string output_name = "output-topk";
    size_t inputsz;
    size_t outputsz;
    nvinfer1::IHostMemory *serialized_engine;
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    cudaStream_t stream;
    mutable void *device_buffer[2];
    float *host_buffer;
    const InferParam param;
    float fx, fy;

public:
    TRTFrame();
    explicit TRTFrame(const std::string &onnx_file, const InferParam &param_);
    ~TRTFrame();
    void Create_Engine_From_Onnx(const std::string &onnx_file);
    void Create_Engine_From_Serialization(const std::string &onnx_file);
    void Create_Serialized_Engine(const std::string &onnx_file);
    void Save_Serialized_Engine(const std::string &des);
    void Save_Serialized_Engine(nvinfer1::IHostMemory *serialized_engine_, const std::string &des);
    void Infer(void *input_tensor);
    float IOU_xyxyxyxy(float yxyxyxy1[8], float xyxyxyxy2[8]);
    float IOU_xywh_center(float xyhw1[4], float xyhw2[4]);
    float IOU_xywh_topl(float xyhw1[4], float xyhw2[4]);
    float IOU_xyxy(float xyxy1[4], float xyxy2[4]);
    float IOU(float *pts1, float *pts2, box_type type);
    void NMS(std::vector<float> &output_tensor, std::vector<std::vector<float>> &res_tensor);
    void NMS(std::vector<std::vector<float>> &res_tensor);
    void Preprocess(cv::Mat &src, cv::Mat &blob);
    void Postprocess(std::vector<std::vector<float>> &res_tensor, std::vector<BoxInfo> &box_infos);
    void Show_xyxyxyxy(cv::Mat &img, std::vector<BoxInfo> &box_infos);
    void Run(cv::Mat &src, std::vector<BoxInfo> &boxinfos);
    // preprocess;postprocess;
};

#endif