# M10P 到货自检

## 环境与运行

已配置独立 conda 环境 `m10p`，Python 3.10.20、pyserial 3.5、Tk 8.6。
安装时国内镜像 SSL 连接失败，最终克隆本机 ld14p 环境并从默认源补齐两个运行库；原环境未修改。绘图使用 Tk，无需 matplotlib。

在 Anaconda Prompt 或已配置 conda 的终端执行：

```powershell
conda activate m10p
cd D:\OneDrive\Desktop\robocup\m10p
python check_m10p.py --list-ports
python check_m10p.py --port COM11 --seconds 15 --plot --range 2
```

当前检测到的设备为 `COM11 / USB-Enhanced-SERIAL CH9102`，重新插拔后以实际端口为准。
仅有一个串口时可以省略 `--port`；多个串口时必须指定。
关闭点云窗口或 Ctrl+C 可提前结束并保存报告，报告会标记测试不完整。
窗口绿色点为普通测距，黄色点为高反点；半径由 `--range` 指定，单位米。

无窗口测试并保存原始数据：

```powershell
python check_m10p.py --port COM11 --seconds 30 --raw reports/capture.bin
```

默认 JSON 报告保存到脚本旁 `reports/日期时间.json`，可用 `--output` 指定。
测试只接收数据，不发送电机或配置命令。结束时释放串口；雷达继续运行。

## 检查内容与判定

- 512000 bps、8N1；解析分片、连续多帧及噪声后的重新同步。
- 检查 A5 5A 帧头、双字节长度、FA FB 帧尾、角度范围、非零转速计数。
- 按长度提取测距区，剔除 FFFF，再按 `起始角度 + 15 / 点数 × 索引` 分配角度。
- 高位为高反标志；清除高位得到毫米距离。0 距离单独统计，不绘制。
- 统计收帧率、点速率、转速、24 个角度区域覆盖、角度连续性及接收间隔。

`PASS`（退出码 0）：满足本程序基本自检条件。
`WARN`（退出码 1）：有异常或测试不充分，具体原因见 `issues`。
`FAIL`（退出码 2）：未收到合法帧或串口/文件访问失败。

默认经验阈值：至少采集 5 秒；平均转速 10–14 Hz；覆盖全部 24 个区域且至少跨零点两次；角度跳变不超过 1%；无结构错误；接收间隔不超过 0.5 秒；帧率与转速推算值相差不超过 15%；非零距离点至少占 10%。这些阈值用于到货初筛，不是厂家验收标准。接收间隔也受电脑调度和绘图影响。

`discarded_bytes` 包含打开串口时收到的半帧；`buffered_bytes` 是结束时尚未完整的一帧，二者不直接等于丢包。
协议没有校验和，不能发现所有字节错误。PASS 也不代表已完成测距精度标定：请在已知距离摆放平整目标，结合点云检查距离和位置响应。

## 与资料的差异

参考本目录厂家 Python 示例、数据输出格式和用户手册。
文档常规帧为 160 字节 / 70 点，本机实际收到 **156、158、160、162 字节**（68、69、70、71 点），因此按长度字段解析，保留 10 字节时间区和 2 字节帧尾。
接收长度限制为 22–512 的偶数，避免损坏长度导致无界等待。原始厂家示例保留不变。

## 本次实机结果

2026-09-27，COM11，带实时点云采集 15 秒：**PASS**。
共 4311 帧，约 19921 点/秒，平均 720.02 rpm（12 Hz），覆盖全部 24 个角度区域，结构异常帧 0、角度跳变 0。
最终报告为 `reports/hardware_check_verified.json`，对应原始数据为同名 `.bin`。
早期 `hardware_check.json` 和 `plot_check.json` 的 WARN 来自初版将 162 字节帧误判为异常；已检查原始数据并修正，以 verified 报告为最终结果。
软件解析测试 5 项通过；测距精度仍需摆放已知距离目标核验。

## 复现环境与软件测试

```powershell
conda env create -f environment.yml
conda activate m10p
python -m unittest discover -s . -p test_check_m10p.py -v
```

若下载慢，可在创建环境时通过 `--override-channels -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main` 指定国内 conda 镜像；pip 可用 `-i https://pypi.tuna.tsinghua.edu.cn/simple`。无需修改全局源配置。

## 排障

- 串口打不开：关闭厂家上位机、串口助手或其他正在使用 COM11 的程序。
- 没有数据：检查实际端口、供电、共地、TX/RX 和转接板。使用配套接线；串口电平按手册为 3.3V，勿用 RS232 电平直连。
- 持续异常帧或丢帧：检查 USB 线、转接板和供电，再用无窗口模式对比。
- 大量零距离：检查雷达周围是否有可测目标，目标是否太近、太远或被遮挡。
