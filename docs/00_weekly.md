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

