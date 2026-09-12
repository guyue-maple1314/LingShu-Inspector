# 波束形成后的单通道音频帧（1.9）：供 Python 侧声纹模型推理
std_msgs/Header header
float64 beam_azimuth      # 波束指向方位角（度）
float64 beam_elevation    # 波束指向俯仰角（度）
uint32 sample_rate        # 采样率 Hz
float32[] samples         # 波束形成后的单通道波形
bool valid                # 帧是否有效；false 时不得用于推理
