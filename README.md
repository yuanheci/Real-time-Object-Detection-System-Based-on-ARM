开发平台为ARMv7架构的NXP.IMX6ULL

目标识别功能，移植了前向推理框架[ncnn](https://github.com/Tencent/ncnn)

和轻量级目标检测算法FastestDet

> **FastestDet是针对计算资源紧缺的ARM平台设计的，突出单核效能**，因为在实际业务场景中，不会把所有CPU资源都给推理框架做模型推理的，假如说你想在例如树莓派, RK3399, RK3568去跑实时目标检测，那么FastestDet是比较好的选择，或者移动端上不想占用太多cpu资源，也可以去用单核并设置cpu sleep去推理FastestDet，在低功耗的条件下运行算法。

然而由于IMX6ULL资源有限，即使使用该算法，依然无法流畅地运行实时推理（存在一定延迟）。



#### 文件说明

+ FastestDet_NCNN-master为算法实现部分 [FastestDet]([dog-qiuqiu/FastestDet: :zap: A newly designed ultra lightweight anchor free target detection algorithm， weight only 250K parameters， reduces the time consumption by 10% compared with yolo-fastest, and the post-processing is simpler (github.com)](https://github.com/dog-qiuqiu/FastestDet))

- asyncnoti.c 为驱动代码
- v4l2_camera.c 实现了ov2640摄像头流畅捕捉画面，当按下按键时进行ncnn推理完成目标识别。
- video_v4l2_camera.c 实现了实时目标检测功能（存在一定延时）。



![new](C:\Users\14464\Desktop\up\README.assets\new.png)



![Fast_res_image](C:\Users\14464\Desktop\up\README.assets\Fast_res_image.png)

![kalvin_tool_20240728150235](C:\Users\14464\Desktop\up\README.assets\kalvin_tool_20240728150235.gif)