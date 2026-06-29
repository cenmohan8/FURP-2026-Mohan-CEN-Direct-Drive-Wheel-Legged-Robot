[EXPERIMENT_LOG (1).md](https://github.com/user-attachments/files/29156450/EXPERIMENT_LOG.1.md)# Weekly Progress Log

> Update this file **every week**. Add a new entry at the top for each week.
> This is the first thing we check during review. Keep it honest and specific — it also feeds your attendance record (Rule 1).

**How to use:** copy the *Week template* block below for each new week. Newest week goes at the top.

---

## Week template — copy me

### Week N — YYYY-MM-DD

**Attended this week's meeting:** Yes / No (if No, did you email leave? Yes / No)

**Progress this week**
- _What did you actually do / finish?_

**Challenges & blockers**
- _What got in the way? What are you stuck on?_

**Next steps**
- _What will you do next week?_

**Hours spent (optional):** _e.g. 6h_

**Links (optional):** _commits, notebooks, docs, datasets..._

---

<!-- =================  YOUR ENTRIES BELOW  ================= -->

### Week 1 — 2026-06-15

**Attended this week's meeting:** Yes

[Up# 实验日志：USD → MuJoCo 模型转换（HopperTrex 轮腿机器人）

## 目标

将 SolidWorks/USD 导出的四足轮腿机器人 `HopperTrex(2).usd` 转换为可在 MuJoCo 中仿真的
`frog.xml` 模型，保证各零件（车身、左右大腿、左右小腿、左右轮）的相对位姿与原始
USD 设计一致。

机器人运动链结构：

```
chassis_base (车身, freejoint)
├── thigh_left  → calf_left  → wheel      (左腿)
└── thigh_right → calf_right → wheel_01   (右腿)
```

---

## 核心原理

- **USD** 存储的是每个 prim 的**世界变换矩阵**（local-to-world）。
- **MuJoCo** 要求每个 body 给出相对**父 body** 的位姿（`pos` + `quat`）。
- 转换公式（关键）：
  ```
  子相对父的变换 = (父世界矩阵)^-1 × (子世界矩阵)
  ```
  用矩阵乘法自动处理平移与旋转，再从结果矩阵中提取 translation 和 rotation quaternion。

---

## 遇到的核心问题：右轮（wheel_01）始终对不上

车身、大腿、小腿这条链很快就对齐了，但**右轮无论如何调整都无法回到正确位置**。
表现包括：左右轮重合、左右轮接到了相反的关节、中心对称（点对称）而非轴对称、
右轮飞到车身侧面等。

最终定位到这是**两个独立 bug 叠加**，因此极难排查。

### Bug 1：同名 prim 覆盖（最隐蔽的根因）

最初脚本使用 `stage.TraverseAll()` 遍历，并以零件**名字**作为字典 key 存储世界矩阵：

```python
for prim in stage.TraverseAll():
    name = prim.GetName()
    if name in targets:
        world_mat[name] = ...   # ← 同名 prim 会互相覆盖！
```

USD 文件中存在多份同名 prim（例如 `/Flattened_Prototype_1/calf_left` 与
`/HopperTrex/HopperTrex/calf_left`）。遍历时后者覆盖前者，导致部分零件读到了**错误副本**的坐标。

**最致命的症状**：`wheel` 与 `wheel_01` 读出了完全相同的世界坐标
`(0.1221, -0.0725, 0.1566)`，于是右轮永远算出和左轮一样的结果，
任何镜像 / 四元数操作都无济于事。

### Bug 2：父节点路径选错

改用精确路径后，一度将**根节点** `/HopperTrex/HopperTrex`（pos ≈ `(0, 0, 0.087)`）
误当作 `chassis_base`。真正的车身是其子节点：

```
/HopperTrex/HopperTrex/chassis_base   pos = (0.2901, 0.1089, 0.0746)
```

父节点（坐标原点）选错，所有相对变换基准全错，导致出现
“轮子位置对了，但大腿/小腿整个往旁边偏移”的现象。

---

## 走过的弯路（按症状打补丁，均无效）

| 尝试                                       | 做法                                   | 结果                                        |
|-------------------------------------------|---------------------------------------|----------------------------------------------|
| 四元数符号归一化 `norm_quat`（强制 w≥0）    | 想消除 ExtractRotationQuat 的符号二义性 | 在接近 180° 的关节上把轮子掀到错误一侧，两轮叠合 |
| 反复猜镜像轴（X / Y / Z）                   | 假设左右轮关于某世界轴镜像              | 时而中心对称、时而左右接反，始终对不上           |
| 硬编码右轮相对坐标 | 直接写死第一版读出的数值 | 数值本身就来自被污染的 prim，依旧错误    |
| 用 chassis 的 Y 坐标做对称反推              | 镜像左轮世界坐标再反推局部坐标           | 位置接近但仍偏，且引入新的姿态错误               |


---

## 解决问题的关键一步：完整 dump prim 层级

停止猜测，改为把每个 prim 的**完整路径 + 类型 + 局部变换 + 父级 + 世界坐标**全部打印：

```python
for prim in stage.TraverseAll():
    if prim.GetTypeName() == "Xform":
        xf = UsdGeom.Xformable(prim)
        wt = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = wt.ExtractTranslation()
        print(f"{prim.GetPath()}  pos=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})")
```

输出立刻暴露两件事：

1. `wheel` 与 `wheel_01` 都**直接挂在 `/HopperTrex/HopperTrex` 根下**，
   各自拥有真实且不同的局部位姿（右轮局部 pos = `(0.1221, -0.0725, 0.0695)`）。
   之前读到的相同坐标是同名 prim 污染所致。
2. 真正的 `chassis_base` 是 `/HopperTrex/HopperTrex/chassis_base`，
   不是根节点 `/HopperTrex/HopperTrex`。

> **结论**：右轮对不上**从来不是数学（镜像 / 四元数）问题，而是数据读取问题——读错了 prim。**

---

## 最终修复

只需两处改动：

1. **用 `GetPrimAtPath` + 完整路径字典精确取 prim**，彻底杜绝同名覆盖：

```python
prim_map = {
    "chassis_base": "/HopperTrex/HopperTrex/chassis_base",
    "thigh_left":   "/HopperTrex/HopperTrex/thigh_left",
    "thigh_right":  "/HopperTrex/HopperTrex/thigh_right",
    "calf_left":    "/HopperTrex/HopperTrex/calf_left",
    "calf_right":   "/HopperTrex/HopperTrex/calf_right",
    "wheel":        "/HopperTrex/HopperTrex/wheel",
    "wheel_01":     "/HopperTrex/HopperTrex/wheel_01",
}

world_mat = {}
for key, path in prim_map.items():
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    world_mat[key] = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
```

2. **修正 `chassis_base` 为真正的子节点路径**（见上）。

修复后，右轮直接通过

```python
wr_p, wr_q = relative("wheel_01", "calf_right")
```

自然算出正确相对位姿，**无需任何镜像或硬编码 hack**。

轮子 hinge 转轴统一设为 `axis="0 1 0"`（绕世界/局部 Y 轴旋转）。

---

## 经验总结

1. **几何怎么调都不对时，第一时间 dump 完整的 prim 路径与层级，不要反复猜变换参数。**
2. USD 中**同名 prim 普遍存在**（尤其有 Prototype / Flatten 副本时）。
   遍历时务必用**完整路径**作 key，或直接 `GetPrimAtPath`，绝不能只用 `GetName()`。
3. 确认每个 body 的**父节点**是否选对——父错则全错。
4. 谨慎使用四元数符号归一化：在接近 180° 的关节附近会引入不连续，导致姿态翻转。
5. 区分症状与病根：位置 / 姿态错误往往源于**输入数据读错**，而非变换数学本身。

---

## 最终验证

左右轮、左右关节、车身完全对称对齐，模型在 MuJoCo 中正常加载并保持正确装配姿态。

---
### Week 2 — 2026-06-22

**Attended this week's meeting:** Yes

直驱轮腿机器人 LQR 平衡控制 — 实验日志

---

1. 项目概述

本项目目标是在 MuJoCo 仿真环境中实现一台直驱轮腿机器人（Wheel-Legged Robot）的 LQR 平衡控制。机器人结构为：一个车身（chassis）+ 左右两条腿（大腿→小腿→轮子），共 6 个执行器（4 个腿关节 + 2 个轮子）。第一阶段目标为静态站立平衡。

1.1 控制架构

部件	数量	控制方法	任务	
大腿（thigh）	2	PD	锁在平衡姿态	
膝盖（knee）	2	PD	锁在平衡姿态	
轮子（wheel）	2	LQR	维持车身平衡	

说明: PD 锁腿是阻抗控制（Impedance Control）的最简形式（刚度 kp + 阻尼 kd），LQR 负责核心的不稳定平衡问题。两者各司其职，不存在"退化成 PID"的问题。

---

2. 环境搭建

2.1 开发环境

组件	版本/配置	
MuJoCo	3.x	
Visual Studio	2026 (x64)	
GLFW	3.4 (lib-vc2022)	
Python	3.11（用于模型分析）	
关键库	mujoco, scipy, numpy	

2.2 VS 项目配置

项目属性（Debug / x64）：

1. C/C++ → 常规 → 附加包含目录:
   
```
   C:/path/to/mujoco/include
   C:/path/to/glfw-3.4.bin.WIN64/include
   ```

2. 链接器 → 常规 → 附加库目录:
   
```
   C:/path/to/mujoco/lib
   C:/path/to/glfw-3.4.bin.WIN64/lib-vc2022
   ```

3. 链接器 → 输入 → 附加依赖项:
   
```
   mujoco.lib
   glfw3.lib
   opengl32.lib
   ```

4. 调试 → 环境:
   
```
   PATH=C:\/path/to/mujoco/bin;%PATH%
   ```

2.3 运行时 DLL 放置

将以下文件复制到 `x64/Debug/`（exe 同级目录）：
- `mujoco.dll`
- `glfw3.dll`

---

3. 模型加载与调试

3.1 基础仿真验证

最小验证代码（不含渲染，纯物理步进）：

```cpp
#include <mujoco/mujoco.h>
#include <cstdio>

int main() {
    char error[1000] = "";
    mjModel* m = mj_loadXML("C:/path/to/robot.xml", nullptr, error, 1000);
    if (!m) { printf("加载失败: %s\n", error); return 1; }
    
    mjData* d = mj_makeData(m);
    for (int i = 0; i < 100; i++) {
        mj_step(m, d);
    }
    printf("仿真成功！跑了100步，时间: %f 秒\n", d->time);
    
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
```

常见问题: `0xc0000135`（找不到 `mujoco.dll`）→ 将 DLL 复制到 exe 目录即可。

3.2 带窗口渲染

使用 MuJoCo 官方 `sample/basic.cc` 模板，替换主循环并加入控制逻辑。关键结构：

```cpp
// 主循环：实时仿真 + 60fps 渲染
while (!glfwWindowShouldClose(window)) {
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0) {
        // === 控制逻辑在此处插入 ===
        mj_step(m, d);
    }
    // 渲染...
}
```

---

4. 模型装配修复（核心难题）

4.1 问题溯源

原始模型从 USD 格式转换而来，存在多个系统性错误：

问题	现象	根因	
轮子横转	轮子像飞盘一样平着打转	关节轴 `axis="0 1 0"` 错误，应为 `axis="1 0 0"`	
轮子偏心转	轮子绕非圆心点旋转	网格几何中心 `(0.046, -0.020, 0.049)` 不在原点，未做偏移补偿	
关节平面错误	大腿/膝盖关节向外拐	USD 旋转数据基于错误的对称轴假设	
左右不对称	右腿整体偏移	`wheel_01` 是 `wheel` 的未镜像副本（同名 prim 覆盖）	
腿身断开	大腿悬在车身下方	富华模型髋关节 Z 高度为 0，丢失了 +0.1745m 的偏移	

4.2 关键认知：坐标系与对称性

通过 USD 结构侦察，确定：
- 左右对称面为 XZ 平面（左右靠 Z 轴区分），而非之前假设的 XY 平面
- 所有网格是"各自归零"导出的，丢失了装配体中的相对位置关系
- 拼装信息只能来自 USD 的位姿数据，但位姿数据因对称轴错误而损坏

4.3 最终修复方案

采用"富华参考模型 + 高度补正"的混合策略：

1. 基础骨架：采用富华 `robot.xml` 的完整结构（含正确的关节定义、质量、惯量、碰撞体）
2. 高度补正：将髋关节 Z 从 `0` 改为 `+0.1745`（来自坏 USD 中保留的正确位置数据），使腿与车身连接
3. 轮子轴修正：`axis="0 1 0"` → `axis="1 0 0"`
4. 保留真实参数：使用模型自带的物理参数（见 5.1 节）

4.4 碰撞体说明

模型使用 `<geom class="collision">` 定义简化碰撞体（胶囊/圆柱），与视觉网格分离：
- 大腿/小腿：`capsule`（胶囊体）
- 轮子：`cylinder`（圆柱体），半径 `0.10`

视觉穿模（网格重叠）不影响物理仿真，因为物理引擎只使用碰撞体。

---

5. LQR 控制器设计

5.1 真实物理参数（来自模型）

部件	质量 (kg)	转动惯量 (kg.m^2)	
车身 (chassis)	5.87	0.126 / 0.144 / 0.085	
大腿 (thigh)	2.02	0.030 / 0.028 / 0.003	
小腿 (calf)	1.40	0.042 / 0.039 / 0.004	
轮子 (wheel)	2.02	0.020 / 0.010 / 0.010	

5.2 状态量定义

```
x = [theta, theta_dot, x_pos, x_vel]

  theta      = 车身俯仰角 (pitch)          —— 从四元数提取
  theta_dot  = 俯仰角速度 (绕 Y 轴)         —— qvel[4]
  x_pos      = 前进位置 (X 坐标)            —— qpos[0]
  x_vel      = 前进速度 (X 方向)            —— qvel[0]
```

5.3 执行器编号

```
ctrl[0] = thigh_left_01   (左大腿)
ctrl[1] = thigh_right_01  (右大腿)
ctrl[2] = knee_left       (左膝)
ctrl[3] = knee_right      (右膝)
ctrl[4] = wheel_left      (左轮)
ctrl[5] = wheel_right     (右轮)
```

5.4 平衡点确定

关键认知：倒立摆不存在"松手能站住"的稳定姿态 —— 任何姿态松手必倒。平衡点是数学上的理想点，LQR 的作用就是主动维持这个不稳定的点。

通过扫描不同大腿角度，找到重心最接近轮轴上方的姿态：

参数	数值	说明	
大腿角度	-0.19 rad	左大腿	
膝盖角度	-1.0 rad	左膝盖	
右腿镜像	+0.19 / +1.0	符号相反	
车身高度	0.176 m	落地稳定后高度	
重心偏移	+0.0167 m	相对轮轴，< 2cm	

5.5 数值线性化

使用 MuJoCo 有限差分法在平衡点求 A、B 矩阵：

```python
# 在平衡点施加微小扰动，观测状态变化
for i in range(4):
    A[:,i] = (perturb(i, +eps) - perturb(i, -eps)) / (2*eps)

# 扰动轮子力矩
B = (step(+eps) - step(-eps)) / (2*eps)

# 离散→连续时间
A_c = (A - I) / dt
B_c = B / dt
```

重要：线性化时必须使用正确的轮子施力方向（左负右正），否则 B 矩阵方向错误。

5.6 轮子方向修正

通过三种力矩组合测试，确定正确方向：

组合	前进 dx	偏航 yaw	结论	
左+1 右+1	-0.004	+18.5	打转！	
左+1 右-1	-0.096	0.0	直线后退	
左-1 右+1	+0.163	0.0	直线前进	

结论：左右轮必须反号施力（左负右正），才能获得纯前进运动。

5.7 LQR 求解

连续时间 A、B 矩阵：

```
A = [[  0.000,   1.000,   0.000,   0.000],
     [  2.047,   0.000,   0.000,   0.000],
     [  0.000,   0.000,   0.000,   1.000],
     [ 12.973,   0.000,   0.000,   0.000]]

B = [[ 0.000],
     [-0.001],
     [ 0.000],
     [-0.827]]
```

权重矩阵：

```python
Q = diag([100.0, 1.0, 1.0, 1.0])   # 最重视俯仰角
R = [[1.0]]
```

求解 Riccati 方程得 K 矩阵：

```
K = [-32.4257, -9.7109, -1.0000, -4.5067]
      pitch     rate     pos      vel
```

验证：pitch 增益最大（-32.4），对应 PID 的 P 项；rate 增益次之（-9.7），对应 D 项。符号和量级均合理。

---

6. C++ 控制代码

6.1 核心控制循环

```cpp
// 读状态
double pitch      = get_pitch();      // 俯仰角
double pitch_rate = d->qvel[4];       // 俯仰角速度
double pos        = d->qpos[0];       // X 位置
double vel        = d->qvel[0];       // X 速度

// LQR: 计算轮子力矩
double wheel_cmd = -(K[0]*pitch + K[1]*pitch_rate 
                    + K[2]*pos + K[3]*vel);

// PD: 锁住腿在平衡姿态
d->ctrl[0] = leg_kp*(THIGH  - d->qpos[7])  - leg_kd*d->qvel[6];   // 左大腿
d->ctrl[1] = leg_kp*(-THIGH - d->qpos[10]) - leg_kd*d->qvel[9];  // 右大腿
d->ctrl[2] = leg_kp*(KNEE   - d->qpos[8])  - leg_kd*d->qvel[7];  // 左膝
d->ctrl[3] = leg_kp*(-KNEE  - d->qpos[11]) - leg_kd*d->qvel[10]; // 右膝

// 施加: 左右轮子反号（关键！）
d->ctrl[4] = -wheel_cmd;   // 左轮
d->ctrl[5] = +wheel_cmd;   // 右轮
```

6.2 初始化到平衡姿态

```cpp
// 在 mj_makeData 后、仿真循环前执行
d->qpos[2]  = 0.176;                    // 车身高度
d->qpos[3]  = 1; d->qpos[4] = 0;       // 四元数 = 竖直
d->qpos[5]  = 0; d->qpos[6] = 0;
d->qpos[7]  = -0.19;                    // 左大腿
d->qpos[8]  = -1.0;                     // 左膝
d->qpos[10] =  0.19;                    // 右大腿（镜像）
d->qpos[11] =  1.0;                     // 右膝（镜像）
mj_forward(m, d);
```

6.3 辅助函数

```cpp
// 从四元数提取俯仰角（绕 Y 轴）
double get_pitch() {
    double w = d->qpos[3], x = d->qpos[4];
    double y = d->qpos[5], z = d->qpos[6];
    double s = 2.0 * (w*y - z*x);
    if (s > 1) s = 1; if (s < -1) s = -1;
    return asin(s);
}
```

---

7. 实验结果

7.1 成功实现

- 机器人在 MuJoCo 仿真中成功实现 LQR 平衡站立
- 车身保持水平，轮子前后小幅滚动以维持平衡
- 腿关节被 PD 锁定在平衡姿态，不参与主动平衡

7.2 已知现象

现象	原因	优先级	
缓慢漂移	K[2]（位置增益）仅 -1.0，对原点约束弱	低，不影响平衡	
视觉穿模	视觉网格重叠，碰撞体正常	无需处理	

7.3 调试经验

1. 先确认物理再谈控制：轮子轴方向不对、关节中心偏了，再好的 K 矩阵也救不了
2. 用诊断脚本代替猜测：三种力矩组合测试比反复改代码盲调高效 10 倍
3. 倒立摆没有稳定姿态：不要试图找"松手不倒"的姿态，直接构造数学平衡点让 LQR 去稳
4. 镜像件的符号关系必须通过实验确认：左右轮同号 = 打转，反号 = 前进

---

8. 下一步计划

阶段	目标	方法	
8.1	前进后退控制	给位置/速度参考信号，让机器人能按指令移动	
8.2	位置控制加强	增大 Q 矩阵中位置项权重，减少漂移	
8.3	阻抗控制	将腿 PD 升级为可调刚度/阻尼的阻抗控制	
8.4	实机迁移	将仿真中的控制律迁移到真实机器人上测试	

---

9. 文件目录结构

```
frog_model/
  frog_lqr_v2.xml          # 最终使用的 MuJoCo 模型
  meshes/
    chassis_base.stl
    thigh_left.stl
    thigh_right.stl
    calf_left.stl
    calf_right.stl
    wheel.stl
  src/
    main.cpp               # C++ 控制代码
  scripts/
    probe_model.py         # 模型侦察脚本
    find_balance.py        # 平衡姿态搜索
    compute_K.py           # LQR 数值线性化
    test_wheels.py         # 轮子方向诊断
  experiment_log.md        # 本文件
```

---

附录：关键参数速查

```
平衡姿态:
  左大腿 = -0.19 rad
  左膝盖 = -1.0 rad
  右大腿 = +0.19 rad (镜像)
  右膝盖 = +1.0 rad (镜像)
  车身高度 = 0.176 m

LQR 增益:
  K = [-32.4257, -9.7109, -1.0000, -4.5067]

PD 参数:
  kp = 300.0
  kd = 15.0

执行器编号:
  0-1: 左右大腿
  2-3: 左右膝盖
  4-5: 左右轮子（注意反号！）

状态量地址:
  pitch      → qpos[3:7] 四元数提取
  pitch_rate → qvel[4]
  x_pos      → qpos[0]
  x_vel      → qvel[0]
```
