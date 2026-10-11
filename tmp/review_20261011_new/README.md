# 2026-10-11 新六次急弯视频与遥测解析索引

本目录为离线分析产物，不属于固件。未修改原始视频、bin和业务代码。

## 阅读顺序

先看各组视频时序图，再看控制曲线和点云；视频时间与板端时间无精确同步标记，不能按相同秒数直接对应。曲线绿色区域是正式采集的主机接收窗口；不代表整段视频。

|组别|视频时序图|控制曲线|前向点云|
|---|---|---|---|
|左1 195850|[时序图](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/195850_sheet_0.jpg)|[曲线](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/195850_trace.png)|[点云](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/195850_cloud.png)|
|左2 改善样本 200137|[时序图](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/200137_sheet_0.jpg)|[曲线](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/200137_trace.png)|[点云](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/200137_cloud.png)|
|左3 未完成协商 200513|[时序图](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/200513_sheet_0.jpg)|[曲线](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/200513_trace.png)|无，空图不作为证据|
|右1 201004|[时序图](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201004_sheet_0.jpg)|[曲线](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201004_trace.png)|[点云](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201004_cloud.png)|
|右2 无导出点云 201147|[时序图](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201147_sheet_0.jpg)|[曲线](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201147_trace.png)|无，空图不作为证据|
|右3 201936|[时序图](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201936_sheet_0.jpg)|[曲线](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201936_trace.png)|[点云](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/201936_cloud.png)|

## 原始视频

- [右弯_v2_1011_201004_11986015000000_出弯时撞出赛道边界mp4.mp4](D:/OneDrive/Desktop/robocup/调试视频/右弯_v2_1011_201004_11986015000000_出弯时撞出赛道边界mp4.mp4)
- [右弯_v2_1011_201147_12088640000000_几乎快出弯成功了.mp4](D:/OneDrive/Desktop/robocup/调试视频/右弯_v2_1011_201147_12088640000000_几乎快出弯成功了.mp4)
- [右弯_v2_1011_201936_12557859000000_弯道过急,不好摆.mp4](D:/OneDrive/Desktop/robocup/调试视频/右弯_v2_1011_201936_12557859000000_弯道过急,不好摆.mp4)
- [左弯-v2_1011_195850_11311812000000_摆弯略慢,贴近赛道内侧,但是没刮蹭赛道内侧边界,出弯时刮蹭外侧边界.mp4](D:/OneDrive/Desktop/robocup/调试视频/左弯-v2_1011_195850_11311812000000_摆弯略慢,贴近赛道内侧,但是没刮蹭赛道内侧边界,出弯时刮蹭外侧边界.mp4)
- [左弯_v2_1011_200137_11478265000000_这次比上次离赛道内侧远了一些,情况改善.mp4](D:/OneDrive/Desktop/robocup/调试视频/左弯_v2_1011_200137_11478265000000_这次比上次离赛道内侧远了一些,情况改善.mp4)
- [左弯_遥测数据可能有损_v2_1011_200513_11694265000000_同样入弯时贴近赛道内侧,出弯时刮蹭赛道外侧.mp4](D:/OneDrive/Desktop/robocup/调试视频/左弯_遥测数据可能有损_v2_1011_200513_11694265000000_同样入弯时贴近赛道内侧,出弯时刮蹭赛道外侧.mp4)
- [赛道布局.mp4](D:/OneDrive/Desktop/robocup/调试视频/赛道布局.mp4)

[赛道布局时序图](D:/OneDrive/Desktop/robocup/tmp/review_20261011_new/layout_sheet_0.jpg)

## 方法与限制

- `video.py`：测试视频每0.4秒取一帧，布局视频每2秒；源文件时长、尺寸与SHA256在 `video_metadata.json`。
- `analyze.py`：复用现有解码器读原始bin，按session/控制序号/配置版本关联附加诊断；结果为各组 `_joined.json` 与 `summary.json`。
- `probe.py`：在本目录编译当前生产 `path_track.c`，检查保存点云的左右重复关联；结果为 `probe_results.json`。
- 曲线缺口可能是附加诊断缺失或几何无效，不等于串口没有收到雷达数据。
- 点云绘图的黑横线表示雷达前100 mm与后205 mm处的前后轴示意，半宽130 mm；不是整车扫掠包络。
- 点云目标到原点的虚线仅表示目标方向，不是已规划的可行行驶轨迹。
- 32条完整快照中有4条重复，只有28个不同控制序号；200137主要保存了后段，不能用这些点云复现它的入弯回摆。
- C探针使用毫米整数化点云和记录宽度，未完整重建宽度学习与避让历史。重复样本和拒绝原因可复核，目标微小差异不能误报为板端浮点错误。

## 完整结论与计划

[六次急弯视频遥测复核与修改方案](D:/OneDrive/Desktop/robocup/robocup-新建文件夹队/robocup-新建文件夹/md文档/M10P六次急弯视频遥测复核与修改方案_20261011.md)
