# SimToolReal 风格训练 observation 与资产对照

本页保留从 Interprior 迁入的历史训练几何约定，仅针对 SimToolReal 风格训练。
所有训练代码路径均指来源仓库 Interprior；本工具不导入或运行这些模块。

## 虚拟掌心调试

两版 Wuji URDF 新增 `right_palm_link → palm_center` 固定 frame，局部位置
`(0.004, -0.0023529, 0.0434161)` m，姿态与手掌相同。
X 经用户调节为 +4 mm，Y/Z 保留掌体 visual mesh 包围盒中心，作为两版同一手掌的几何参考；
它不是标定的接触面中心，也不是惯性中心。Viser 的 Virtual palm tuning 面板
可分别调整两版局部 xyz，紫色点实时更新，显示可复制的 URDF origin XML。
调节过程只预览，点击各模型的确认按钮会写入对应 URDF；训练 observation 配置不会自动切换。

Kuka 的历史 palm offset `(0,-0.02,0.16)` m 换算到 `left_hand_C_MC` 下为
`(0.01931854,-0.00517630,0.065)` m。Sharpa 掌体网格范围为
x `[-0.02460,0.02528]`、y `[-0.04365,0.04149]`、z `[0.00003,0.09932]` m，
因此该点靠近掌体 +X 一侧，并在腕到指根方向约 65 mm 处；这里只依据网格范围描述位置，
不声称它是精确掌面标定点。

这里的原始 SimToolReal 指仓库 `legacy/isaacgymenvs` 中的 Kuka + 左 Sharpa。
当前 `cfg/simulation/simtoolreal.yaml` 已迁移为 xArm7 + Wuji，名称相同但机器人不同。

| 项目 | 原始 SimToolReal | 现有侧装 Wuji | 现有直装相机 Wuji |
|---|---|---|---|
| URDF（相对仓库根目录） | `assets/urdf/kuka_sharpa_description/iiwa14_left_sharpa_adjusted_restricted.urdf` | `xarm7_wuji_90.urdf` | `xarm7_wuji_direct_camera.urdf` |
| 关节 | Kuka 7 + Sharpa 22 = 29 | xArm7 7 + Wuji 20 = 27 | 同左 |
| observation palm 位置 | `iiwa14_link_7` 原点 + 旋转后的 `(0,-0.02,0.16)` m | `link7` 原点，offset 为零 | 同左 |
| observation palm 姿态 | `iiwa14_link_7` 姿态 | `link7` 姿态 | 同左 |
| 实体手掌 link | `left_hand_C_MC` | `right_palm_link` | `right_palm_link` |
| 指尖来源与顺序 | `left_{index,middle,ring,thumb,pinky}_DP` | `right_finger1..5_link4`（thumb,index,middle,ring,pinky） | 同左 |
| 指尖局部偏移 | 各 DP + `(0.02,0.002,0)` m | 各 link4 + 独立 pad-center 标定偏移；若导入 tip_link 则偏移为零 | 同左 |

Wuji 网格位于仓库根目录的 `meshes/`；Kuka/Sharpa 网格与其 URDF 同目录。
以下其他物体资产和运行时行为描述的是来源仓库 Interprior，不是本仓库的依赖。
物体资产还包括 `assets/urdf/dextoolbench`、`assets/urdf/oxl` 和基础 cube/rect_block/table URDF。
handle-head 程序化物体由 `envs/simulation/scene/objects.py` 生成，当前场景运行时创建 `/tmp/simtoolreal_assets_*` 临时目录。

## Actor observation 的字段对应（Python 半开区间）

| 字段 | Sharpa 基础 140D | Wuji 基础 134D |
|---|---|---|
| joint_pos | 0:29 | 0:27 |
| joint_vel | 29:58 | 27:54 |
| prev_action_targets | 58:87 | 54:81 |
| palm_pos | 87:90 | 81:84 |
| palm_rot | 90:94 | 84:88 |
| object_rot | 94:98 | 88:92 |
| fingertip_pos_rel_palm | 98:113 | 92:107 |
| keypoints_rel_palm | 113:125 | 107:119 |
| keypoints_rel_goal | 125:137 | 119:131 |
| object_scales | 137:140 | 131:134 |

Wuji 启用 surface-distance 特征后追加 `hand_keypoint_to_surface_vec[134:174]`。
维度取决于实际配置中的字段列表，不能只根据任务名判断。
`palm_vel`、`object_vel` 等在默认 critic state 中，未放入上表 actor。

`joint_pos = 2*(q-lower)/(upper-lower)-1`；速度为关节速度，prev targets 为关节目标值。
历史代码按 Isaac Gym DOF 顺序读 29 维；当前代码将 Isaac Lab 顺序显式重排为
`joint1..7`，然后 finger1..5 各 joint1..4。不同机器人的手关节不能逐索引直接复制。
指尖若要按语义对齐，Sharpa 的 index/middle/ring/thumb/pinky 应对应 Wuji 的 2/3/4/1/5。

两套 policy 的 quaternion 都是 **xyzw**；Isaac Lab 内部和页面控件使用 wxyz，需转换。
`fingertip_pos_rel_palm = tip_world - palm_world`，`keypoints_rel_palm = keypoint_world - palm_world`，
没有乘 palm 逆旋转，因而是世界轴方向的相对向量。
四个物体 keypoint 的符号顺序为 `+++ / ++- / --+ / ---`，经尺寸缩放、物体旋转和平移生成；
`keypoints_rel_goal` 为当前与目标同序 keypoint 相减。
当前多环境代码从世界坐标减去 env origin 得到环境内坐标。

Sharpa 的 palm 位置含偏移，但 palm_rot 和 critic palm_vel 仍直接取 Kuka 末端刚体。
Wuji 的 `right_palm_link` 是 URDF 实体手掌，和 observation 使用的 `link7` 不重合。
页面保留这两种定义并显示差异。

## 页面说明与代码来源

三套机器人仅为展示沿 y 轴分开，显示位移不计入 observation。
黄色物体姿态与尺寸共享，便于比较相同物体下的几何；这不代表三者训练初始物体位置相同。
Sharpa 29 个关节独立控制，初始采用历史配置 startArmHigher=False 的姿态。
本页展示无噪声 FK 几何，不运行物理仿真，也不复现训练中的噪声、延迟、dropout 或 clipping。

- 原始配置：`legacy/isaacgymenvs/cfg/task/SimToolReal.yaml`
- 原始映射：`legacy/isaacgymenvs/tasks/simtoolreal/env.py` 的初始化、`palm_center_pos` 和 `populate_obs_and_states_buffers`
- 当前配置：`envs/simulation/env_cfg/base_env_cfg.py`、`cfg/simulation/simtoolreal.yaml`
- 当前关节重排：`envs/simulation/utils/reset.py`
- 当前 observation：`envs/simulation/utils/observation.py`
