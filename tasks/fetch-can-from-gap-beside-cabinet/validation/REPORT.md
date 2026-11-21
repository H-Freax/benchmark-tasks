# 窄缝取罐：CPU 可行性扫描

完整扫描 3456 个参数组合：通过 1522（44.04%），失败 1934。每组测试两个相差 180° 的朝下夹爪 yaw，共 6912 个方向结果。

环境：macOS-26.6-arm64-arm-64bit，Python 3.12.14，NumPy 2.5.5，SciPy 1.18.1，python-fcl 0.7.0.11。耗时 636.4 秒。YAM 源码提交 `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`。未运行 Isaac Sim、Arena 运行时、GPU 或接触动力学。

## 尺寸

| 项目 | 实测值（mm） |
|---|---:|
| 手掌最低面到指尖 L | 83.343734 |
| 指尖低于 TCP | 12.842500 |
| 手掌最低面高于 TCP | 70.501234 |
| 手掌：开合方向 / 横向最大截面 | 101.999999 / 64.999999 |
| 手腕 link5：参考姿态两个方向最大截面 | 56.999996 / 68.953449 |
| 净口 67 mm 时手指外缘总宽 | 114.484186 |
| 入缝净口 71 mm 时手指外缘总宽 | 118.484186 |
| 手指完整横向包络 | 67.999220 |
| 罐直径 / 缩放后实际高度 | 67.000000 / 100.752791 |

手腕宽度随 joint6 变化：扫描抓取姿态的横向投影跨度范围 56.999996–73.052055 mm，开合方向 56.999996–73.052070 mm。参考最大截面用凸网格边与水平平面的交点计算；w 采用完整投影包络，属于保守选择。

框架常数：`PAD_TIP_PAST_TCP=0.013` m，`TCP_OFFSET=[0.0, 0.0, 0.134]` m，`FINGER_OPEN=-0.04695` m，`FINGER_CLOSED=0.0` m。框架默认控制频率是 10.0 Hz，本扫描按要求使用 20 Hz。

## 每个 δ 的最小 c

同一个 c 下优先列最小墙距最大的通过组合，再以关节余量排序。x、y 单位 m；其余长度单位 mm。

| δ | c | x | y | θ° | h | d | w | 最小墙距 | 关节余量° |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5 | 4 | 0.20 | 0.22 | 0 | 65 | 40.753 | 71.999 | 7.585619 | 2.798952 |
| 15 | 4 | 0.20 | -0.22 | 0 | 65 | 50.753 | 71.999 | 3.957334 | 0.520096 |
| 25 | 4 | 0.16 | 0.16 | 60 | 35 | 90.753 | 77.052 | 4.509224 | 5.000420 |
| 40 | 4 | 0.16 | 0.16 | 60 | 35 | 105.753 | 77.052 | 4.503250 | 1.612153 |

## 推荐的三个组合

按最小 c 档内的关节余量和 IK 余量选择不同抓取位置；推荐仅针对本次路径的数值鲁棒性。

| 序号 | x,y（m） | θ° | δ / h / c / w（mm） | 最小墙距 mm | 关节余量° | 位置 / 姿态 IK 余量 |
|---:|---|---:|---|---:|---:|---|
| 1 | 0.16, 0.10 | 60 | 25 / 35 / 4 / 77.052 | 4.500141 | 5.983861 | 0.417669 mm / 0.088087° |
| 2 | 0.16, -0.10 | 60 | 25 / 35 / 4 / 77.052 | 4.500127 | 5.983861 | 0.417669 mm / 0.088087° |
| 3 | 0.24, 0.22 | 60 | 25 / 35 / 4 / 77.052 | 4.501276 | 5.951625 | 0.416337 mm / 0.088360° |

## 失败统计

以下分母均为 3456 个组合；原因允许重叠。采用每行选择的 yaw，不隐藏另一 yaw 的尝试，后者另有完整统计和 CSV。

| 项目 | 数量 | 占全组合 |
|---|---:|---:|
| 路径 IK 超差 | 416 | 12.04% |
| 路点上移 10 mm 不可达 | 992 | 28.70% |
| 墙距不足 3 mm | 1450 | 41.96% |
| 墙面接触或穿透 | 234 | 6.77% |
| 关节实际越界 | 0 | 0.00% |
| 关节在限位边界饱和 | 1007 | 29.14% |
| TCP / 指尖高度不足 | 0 | 0.00% |

| 连杆 | 墙距不足 3 mm 的组合数 | 实际接触组合数 |
|---|---:|---:|
| left:base | 192 | 117 |
| left:link2 | 48 | 48 |
| left:tip_left | 603 | 0 |
| left:tip_right | 603 | 0 |
| right:base | 192 | 117 |
| right:link2 | 48 | 48 |
| right:tip_left | 603 | 0 |
| right:tip_right | 603 | 0 |

限位饱和涉及：{"joint4": 992, "joint6": 39}。IK 实现将每次迭代夹到 URDF 限位内，所以‘越界为零’不能替代 IK 误差检查。

| 失败原因组合（互斥，每个失败行计一次） | 数量 |
|---|---:|
| IK;waypoint_plus_10mm;joint_limit_saturation | 176 |
| IK;waypoint_plus_10mm;joint_limit_saturation;wall_clearance | 240 |
| joint_limit_saturation;wall_clearance | 2 |
| wall_clearance | 940 |
| waypoint_plus_10mm;joint_limit_saturation | 308 |
| waypoint_plus_10mm;joint_limit_saturation;wall_clearance | 268 |

两个 yaw 的全部 6,912 行尝试（包含未被选中的失败）：

| 项目 | 数量 |
|---|---:|
| total_rows | 6912 |
| passed | 2196 |
| failed | 4716 |
| IK | 2352 |
| plus10 | 3216 |
| wall | 4036 |
| joint_limit_violation | 0 |
| joint_limit_saturation | 3348 |

## 数值核验

完整检查了 169,344.0 次 `ArmPlan.action7`、8,640.0 次 +10 mm 路点 IK。315 个 URDF 凸碰撞体纳入检查。每条路径 98 帧；四个 c 共用同一条完整 IK 路径，墙距分别计算。

网格、唯一性、两个 yaw、每帧调用、每个路点 +10 mm、通过阈值、源码哈希与独立距离检查：{"grid_complete": true, "unique_rows": true, "variants_complete": true, "every_frame_called": true, "every_waypoint_plus10": true, "passing_thresholds_consistent": true, "selected_pass_iff_some_variant_passes": true, "script_hash_matches": true, "independent_distance_checks": true}。

| 代表组合 x,y; θ,δ,h,c | CSV 墙距 mm | 独立凸 QP mm | 差值 mm |
|---|---:|---:|---:|
| 0.2,0.22; 0,5,65,4 | 7.585619285 | 7.585525269 | 9.40164476e-05 |
| 0.2,-0.22; 0,15,65,4 | 3.957334029 | 3.957247200 | 8.68297877e-05 |
| 0.16,0.16; 60,25,35,4 | 4.509223991 | 4.509223994 | 3.29682982e-09 |
| 0.16,0.16; 60,40,35,4 | 4.503250447 | 4.503250487 | 3.97318411e-08 |
| 0.16,-0.22; 0.0,5.0,35.0,4.0 | 0.000000000 | 0.000000000 | 0 |
| 0.16,-0.22; 0.0,25.0,65.0,4.0 | 0.000000000 | 0.000000000 | 0 |
| 0.16,-0.22; 0.0,15.0,65.0,4.0 | 0.000000000 | 0.000000000 | 0 |
| 0.16,-0.22; 60.0,5.0,35.0,4.0 | 1.993008988 | 1.993008656 | 3.32801065e-07 |

## 全部问题与适用范围

d/L 分类与按 TCP 实际手掌入缝阈值不一致的组合共有 1152 个。完整网格检查不依赖该分类，故不会省略手掌碰撞；但缝宽仍严格按要求的 d/L 分类计算。

1. First full run stopped before scanning: sandbox denied writing into the attached worktree. Retried in /private/tmp; outputs copied afterward.
2. Initial dependency install was blocked by sandbox cache writes, and initial public asset download by sandbox DNS. Retried with reviewed network access and a temporary cache; both succeeded.
3. Migrated branch .venv uses Python 3.9.6 and lacks numpy/yam_arena. Scan used bundled Python 3.12.14 and temporary CPU dependencies.
4. Local Arena source pyproject version is 0.3.0, commit c7edf3a9d5d8276565208f72422f8f8a0806d6cf; the scan imports no Isaac or Arena runtime.
5. Can asset is the 6.2 public mirror; no Isaac Sim 6.1 / Arena 0.3.1 simulator validation was performed.
6. usd-core emitted ARCH_CACHE_LINE_SIZE != Arch_ObtainCacheLineSize() during measurement; USD mesh dimensions and SHA match the previously recorded asset.
7. d is measured from TCP, so palm entry starts at 70.501234 mm, not at L=83.343734 mm. Requested d/L classification retained; complete hull collision checks prevent omitted palm geometry.
8. At an exact downward pose, wrist link5 begins 134 mm above TCP, beyond the largest scan d=105.752791 mm. The requested d>L width rule therefore conservatively includes a wrist that is above the wall top.
9. The original task scaled the can to 66 mm; this scan uses 67 mm with height 100.752791 mm.
10. Joint-limit saturation is counted separately from violations because the unmodified framework solver clips every iterate.
11. All passes are CPU kinematics/geometry only: no contact simulation or proof of stable can extraction.
12. Failed local IK seeds/paths do not establish global infeasibility.

θ 为镜像有符号定义；两只手各自在自己一侧工作。起点为已经朝下的悬停姿态，不包含 READY 到悬停的过渡。终点按罐底高于墙顶 30 mm 检查；没有把 TCP 高于墙顶误当作罐子已经提出。

只检查指定机器人—墙体条件；不认证自碰撞、台面碰撞或 URDF 中没有的相机碰撞体。未运行 GPU、Dream-1、媒体上传或 PR 操作。本目录没有新增正式任务评分或正式任务测试。
