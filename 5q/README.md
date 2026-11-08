# 5q —— 循迹版固件（急弯优化版）

STC8H8K64U + TB6612 + 4×TCRT5000 四路红外循迹固件。

本目录是在小组可运行版（`4.源码`）基础上做的**急弯优化版**，用于解决"曲率太大转不过来、探头跑出黑线就永久停车"的问题。基线版本保持原样未改动，本版只改了 `Config/config.h` 与 `App/line.c` 两个文件。

- 编译结果：**code 5893 B / data 106 B / xdata 0 / 0 Error 0 Warning**
- 系统时钟：**11.0592 MHz**（`config.h` 中 `SYSCLK_HZ` 写死，STC-ISP 必须选同一个频率）
- 已编译固件：`car.hex`（由 `Objects/car.hex` 复制）

---

## 1. 目录结构

```
5q/
├─ App/
│  ├─ car.c/.h          主流程状态机（标定、起跑、停车）
│  ├─ line.c/.h         循迹核心：读四路 → 算误差 → 转向量 → PWM   ★本版改动
│  └─ display.c/.h      OLED 界面
├─ Config/
│  ├─ config.h          全部可调参数都在这里                        ★本版改动
│  ├─ board.h           引脚定义
│  └─ STC8H.H           寄存器定义
├─ Drivers/             motor / oled / timebase / ultrasonic
├─ main.c
├─ STARTUP.A51
├─ car.uvproj           Keil 工程（器件、时钟、头文件路径已配好）
├─ car.hex              可直接烧录的固件
└─ 优化说明与调参手册.md   ★ 为什么这么改 + 怎么调参，上车前先看这个
```

`Objects/`、`Listings/` 是编译中间产物，已在 `.gitignore` 中排除。

---

## 2. 编译与烧录

1. 双击 `car.uvproj`（Keil µVision + C51），按 `F7` 编译，无错误。
2. STC-ISP 选 **STC8H8K64U**，IRC 频率选 **11.0592 MHz**（选错会表现为"上电就冲、OLED 乱跳"，因为所有定时都按 2.17 倍速跑）。
3. 烧 `car.hex`，或直接用 `Objects/car.hex`。

命令行自检（可选）：

```bash
"D:/AppGallery/Keil/UV4/UV4.exe" -b car.uvproj -j0 -o build.log
```

---

## 3. 标定与起跑（按键 P3.3）

1. 上电 → `HOLD P3.3 TO TEST` / `WAIT FOR CAL`，**3 秒内不按**即进入标定。
2. `ALL 4 ON WHITE`：四路都压白底，按住 P3.3 保存。
3. `1 LEFT OUT` … `4 RIGHT OUT`：四路分别逐个压黑线中心并保存。
4. `CENTER CAR ON LINE`：车体骑在线上（黑线在两内探头之间），保存。
5. 检查屏幕四路 `W/B` 值，要求 **B − W > 120**（`MIN_CAL_SPAN`），否则报 `LOW ADC SPAN`。
6. `CAL DONE` 后按住 P3.3 → 延时 2 秒起跑。

> 标定值只存在 RAM，**断电即丢**。换场地、换电池、拆装探头后都要重标。探头整体前移/后移不需要改代码，重标一次即可（`SetCenterReference()` 会自动重算 `center_q`）。

---

## 4. 本版改了什么（6 处）

| # | 位置 | 参数 | 原值 | 现值 | 作用 |
|---|---|---|---|---|---|
| 1 | `config.h` | `LINE_GAIN_LOW / HIGH` | 单一 `KP=3` | **3 / 8**（拐点 80，中间线性过渡） | 变增益：\ |e\|≤80 与原版逐位相同，大偏差增益近 3 倍 |
| 2 | `config.h` | `MAX_TRIM` | `18`（因误差上限 200 永不生效） | **14** | 变成真正生效的满舵：`26/0`，内侧停转 |
| 3 | `config.h` | `LINE_CURVE_SLOW` | 无 | **4**（下限 12） | 弯道主动降速，换来转弯余量 |
| 4 | `config.h` | `LINE_PID_KD` | `0` | **3** | 微分预判，补相位滞后 |
| 5 | `config.h` | `MOTOR_STEP_MS` | `20` | **10** | 电机斜坡响应快一倍 |
| 6 | `line.c` | 失线救援 | 1 s 超时、单向扫 | **1.6 s + 后半程反向扫 + 枢轴 22%**，确认帧 3→2 | 冲出弯后还能救回来，不再直接永久停车 |

效果：最小转弯半径约从 **1.4×轮距** 降到 **0.5×轮距**（内侧轮停转，绕另一侧轮子转）。

**误差符号（与旧版相反，别搞混）**

```
error > 0  → 黑线在车身右侧 → 左轮快、右轮慢 → 向右转
error < 0  → 黑线在车身左侧 → 左轮慢、右轮快 → 向左转
```

想完全回到原版手感：`LINE_GAIN_HIGH=3`、`MAX_TRIM=18`、`LINE_CURVE_SLOW=0`、`LINE_PID_KD=0`。

---

## 5. 上车调参顺序

详见 **[优化说明与调参手册.md](优化说明与调参手册.md)**，要点：**每一步只动一个参数**。

1. 先跑直道，确认没变坏（`|e|≤80` 段与原版完全相同）。
2. 跑急弯：还冲出去 → `LINE_GAIN_HIGH` +1；弯出口冲出去 → `LINE_PID_KD` +1；车头已斜还在加速 → `LINE_CURVE_SLOW` +1。
3. 过弯后画龙 → `LINE_PID_KD` −1 或 `LINE_GAIN_HIGH` −1。
4. 最后再提速度：`LINE_LEFT_BASE_PWM / LINE_RIGHT_BASE_PWM`（现 20/18）每档 +2，每档重跑急弯。

调参到极限仍然冲出去，就该动机械了：**探头前伸 20~30 mm** 的效果比调任何参数都大。
