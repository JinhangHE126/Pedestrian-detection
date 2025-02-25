# 行人检测
## 主要模块
Detect.py - 行人识别，使用yolo v8模型。我电脑性能不够，所以没有训练很多轮。训练的onnx文件可用但效果不好。
可自行替换为自己训练的模型。

Track.py - 行人跟踪，使用KCF算法。特征使用的是HOG特征，目前并未实现CN特征。
之后可将HOG特征与CN特征进行特征融合以获得更好的效果。这个模块里也使用了卡尔曼滤波辅助跟踪过程，但效果不是很好。
除此之外，这个模块有自适应尺度的功能，在检测时会使用不同尺寸的检测框进行检测，这样可以随着目标尺寸的变化而变化。

Kalman.py - 卡尔曼滤波的实现

TrackingProgram.py - 总体的跟踪算法。先使用行人识别检测所有行人，获取检测框面积最大的目标（这个可以修改）。
再将检测框信息输入KCF算法，用KCF算法进行跟踪。由于是长时跟踪，因此使用响应值的最大值和APCE两个指标作为置信度的指标。
统计过去n帧的响应值最大值与apce（n默认30），然后求平均。若当前帧的最大响应值与apce均超过平均值*a
（其中a是一个系数，由response_thres和apce_thres提供），则认为跟踪结果正确。否则认为跟踪失败。
跟踪失败时会使用Detect模块重新检测所有行人，然后与Track模块中的信息进行比较，再次获取目标准确位置。

## 依赖
模块需要python、numpy和cv2

我用的版本分别是:
 - python 3.9.17
 - numpy 1.25.2
 - opencv 4.6.0

（同anaconda的yolo环境）

## 参数
TrackingProgram:
 - video_path 视频路径。这个可修改代码调整为摄像头输入
 - model_path 行人识别模型onnx文件路径
 - model_input_size 模型输入尺寸。默认416（表示输入416*416大小的图片）

Detect:
 - model_path 同上
 - Input_size 同上
 - colors 检测框颜色（不需要管）
 - class_desc 每个分类的名字（不需要管）
 - conf_thres 置信度阈值（低于该置信度的检测框会被过滤掉。默认0.3）

Track:
 - roi_scale_weight KCF运行时会使用不同尺寸的检测框进行比对。这个是指
检测框尺寸变化时是否需要增益或抑制（为1表示无增益或抑制。默认0.8）
 - min_scale 检测框尺寸变小时最小的比例（默认0.95）
 - max_scale 检测框尺寸变大时最大的比例（默认1.05）
 - scale_step 检测框尺寸每次变化的步长（默认0.05）
 - response_thres 最大响应值阈值系数（见TrackingProgram的描述。默认0.5）
 - apce_thres apce阈值系数（见TrackingProgram的描述。默认0.5）
 - keep_hist_cnt 记录过去多少帧（见TrackingProgram的描述。默认30）
 - learning_rate KCF算法每次跟踪后会使用新的参数更新原参数。这个就是更新的程度（默认0.012）

（其他参数可使用默认参数，如需修改请自己看代码）# Pedestrian-detection
