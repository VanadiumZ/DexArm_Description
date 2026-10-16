# xArm7 末端执行器 URDF 模型

本仓库提供基于 **UFACTORY xArm7** 的多种末端执行器（Wuji 灵巧手、XHand、xArm 平行夹爪）URDF 模型，以及对应的可视化/碰撞网格。模型可用于 ROS、Gazebo、PyBullet、Isaac Sim 等支持 URDF 的工具链。

## 虚拟掌心与 observation 工具使用指南

工具可以加载机器人 URDF、调整关节和虚拟掌心、摆放测试物体，并实时显示
observation 几何分量和相对向量。调整完成后，可在 Viser 页面点击确认按钮，
将掌心定义直接写回正在查看的 URDF。

工具使用独立 Python CPU 环境，不需要 Interprior、Isaac Sim/Lab、CUDA 或 ROS。
服务器负责 URDF/FK 计算，浏览器负责三维显示；不运行物理仿真或训练。

**历史 observation 对照专门针对 SimToolReal 风格训练设计。**
Wuji/Sharpa 保留迁移时的掌心定义、指尖偏移和几何计算约定；XHand/未知型号提供
明确列出的几何观测，不默认兼容已有策略的完整 observation。
详细历史字段见 [SimToolReal observation 说明](docs/asset_observation_comparison.md)。

### 1. 安装与快速开始

以下命令在仓库根目录执行，使用 Python 3.11：

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt

# 加载 XHand，进入单模型完整调节界面
python -m tools.urdf_observation_viewer \
  --urdf xarm7_xhand_direct.urdf \
  --host 127.0.0.1 --port 8080
```

浏览器打开 `http://localhost:8080`。停止工具时，在启动终端按 `Ctrl+C`。
修改工具代码后需停止旧进程，再重新运行。

如果已经安装好仓库的 `.venv`，无需重新创建，直接启动：

```bash
.venv/bin/python -m tools.urdf_observation_viewer \
  --urdf xarm7_xhand_direct.urdf --host 127.0.0.1 --port 8080
```

使用 uv 安装或更新依赖：

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

已有 uv 环境只执行第二行即可；uv 创建的环境默认可能没有 pip。
其他环境细节见 [环境配置文档](docs/environment.md)。

### 2. 选择运行模式和模型

| 模式 | 启动方式 | 用途 |
| --- | --- | --- |
| 单模型模式 | 指定 `--urdf` | Wuji、Sharpa、XHand 或新 URDF；关节、掌心 xyz/rpy、物体与观测、确认保存 |
| 历史三模型对照 | 不指定 `--urdf` | 同时比较两套 Wuji 与 Kuka + 左 Sharpa 的 SimToolReal 风格几何 |

#### 单模型启动示例

```bash
# XHand 直装
.venv/bin/python -m tools.urdf_observation_viewer --urdf xarm7_xhand_direct.urdf

# XHand 直装带相机
.venv/bin/python -m tools.urdf_observation_viewer --urdf xarm7_xhand_direct_camera.urdf

# Wuji 侧装，也可换成其他 xarm7_wuji_*.urdf
.venv/bin/python -m tools.urdf_observation_viewer --urdf xarm7_wuji_90.urdf

# Kuka + 左 Sharpa
.venv/bin/python -m tools.urdf_observation_viewer \
  --urdf assets/urdf/kuka_sharpa_description/iiwa14_left_sharpa_adjusted_restricted.urdf

# 通用模式：夹爪，并指定 TCP 为掌心参考 link
.venv/bin/python -m tools.urdf_observation_viewer \
  --urdf xarm7_gripper.urdf --palm-link link_tcp

# 外部 URDF；路径与 link 名需替换成实际值
.venv/bin/python -m tools.urdf_observation_viewer \
  --urdf /path/to/robot.urdf --palm-link your_palm_link
```

URDF 必须能找到其引用的网格文件，不能只复制 URDF 而遗漏 mesh。
默认资产路径按工具所在仓库解析；显式传入的相对路径按启动时的工作目录解析。

#### 历史三模型对照

```bash
.venv/bin/python -m tools.urdf_observation_viewer
```

默认加载：

| 界面名称 | 模型 |
| --- | --- |
| old | `xarm7_wuji_90.urdf`，侧装 Wuji |
| new | `xarm7_wuji_direct_camera.urdf`，直装相机 Wuji |
| SimToolReal | `assets/urdf/kuka_sharpa_description/iiwa14_left_sharpa_adjusted_restricted.urdf` |

两套 Wuji 的手臂分别调节，20 个手指关节及测试物体姿态共享；模型沿 Y 轴分开仅供显示。
可以查看两套 Wuji 的几何差异、历史 Sharpa 偏移、物体 keypoint、相对向量和 JSON 快照。
此模式的 Wuji 掌心控件调节 xyz，姿态沿用 URDF；各自的确认按钮写回 old/new 文件。
要调节掌心 rpy、使用更细的调节步长或保存 Sharpa 掌心，请通过 `--urdf` 进入单模型模式。

`--old-urdf`、`--new-urdf`、`--simtoolreal-urdf` 用于替换对应对照资产，仍要求原有
Wuji/Sharpa 关节和 link 结构；不能用这些参数直接替换成 XHand。

### 3. 手型识别与自动关节读取

识别依据是 URDF 内部关节和 link 名称，不依赖文件名。

| 手型 | 控制顺序 | 默认掌心参考 link | 本仓库自由度 |
| --- | --- | --- | --- |
| 右 Wuji | xArm7，然后 finger1～5 各 4 个关节 | `right_palm_link` | 27 |
| 左 Sharpa | Kuka，然后历史 Sharpa 固定关节顺序 | `left_hand_C_MC` | 29 |
| 右 XHand | xArm7，然后 thumb/index/mid/ring/pinky | `right_hand_link` | 19 |
| 未知型号 | 自动读取独立关节，保持 URDF 顺序 | 根 link，需自行选择掌心参考 | 由 URDF 决定 |

已知手型使用固定映射，额外的可动关节按 URDF 顺序追加。改名、关节不完整或其他左右手版本
若不符合现有签名，会进入通用模式。通用模式支持 `revolute`、`continuous`、`prismatic`；
不提供 `floating`/`planar` 多自由度关节控制。`mimic` 关节随源关节更新，不单独控制。

关节控件读取 URDF 限位：旋转单位 rad，移动单位 m；没有限位时不设置输入上下界。
查看器中的关节配置是调试姿态，不会通过掌心确认按钮保存到 URDF。

### 4. 单模型界面：从定位掌心到保存

#### 显示模型与选择参考 link

用 `显示 visual` / `显示 collision` 切换外观与碰撞网格，在“关节控制”中摆好机器人姿态。
进入 **Virtual palm tuning · 手掌局部坐标**，检查 `掌心参考 link`：

- 已知手型默认使用上表中的手掌 link；也可用 `--palm-link` 或页面下拉框更改。
- 未知模型默认是根 link，应先改成实际手掌或希望附着掌心的刚体。
- xyz/rpy 表示虚拟掌心相对于该 link 的局部变换，不是世界坐标。
- 不允许把 `palm_center` 或其子节点选作父节点，避免形成关节环。

初值优先读取 URDF 已有的 `palm_center`。没有该 frame 时，Sharpa 在存在 Kuka 腕部
link 的情况下使用历史偏移；XHand 和未知模型从所选 link 原点开始，需自行确定掌心位置。
初值不等于已标定的接触面中心。

#### 大球定位和精调

| 控件 | 选项／单位 | 默认值 |
| --- | --- | --- |
| 掌心球显示 | 大球半径 25 mm / 小球半径 4 mm / 隐藏 | 大球 |
| 位置调节步长 | 1 / 0.1 / 0.01 mm | 0.1 mm |
| 姿态调节步长 | 0.01 / 0.001 / 0.0001 rad | 0.001 rad |
| `xyz (m)` | 局部位置，直接输入米值 | 来自已加载掌心或型号初值 |
| `rpy (rad)` | 局部 roll、pitch、yaw | 来自已加载掌心或型号初值 |

先用大球定位，再切换小球和细步长进行精调。输入框保留 7 位小数，也可以直接键入数值。
例如沿参考 link 的 X 轴增加 0.1 mm，就是将 x 增加 `0.0001` m。
显示球的球心才是掌心参考点；球半径只影响显示，不改变 observation，也不会写成碰撞几何。

切换参考 link 会恢复已加载／最近确认的掌心相对于新 link 的变换，
不会保留尚未确认的局部输入。未存在掌心定义时，使用该型号的初始化规则。

#### 确认并覆写当前 URDF

1. 检查面板中的保存目标路径，以及当前参考 link、xyz/rpy。
2. 点击 **确认掌心并写入 URDF**。
3. 查看面板“已保存到 …”及保存参数；失败时同一区域显示原因。

按钮直接写入启动时 `--urdf` 指定的文件，包括外部路径对应的文件：

- 没有 `palm_center` 时，添加该 link 及 `palm_center_joint` 固定关节。
- 已有 `palm_center` 时，更新其固定关节的 parent 和 origin；保留已有关节名称。
- 重复确认更新同一掌心定义，不重复添加节点；保留其他模型定义、注释和网格路径。
- 只保存掌心定义，不保存物体姿态、关节调试角度、球体显示选项或 observation 配置。

调整数值本身只预览，点击确认才写入。**恢复已加载／确认的掌心** 会恢复启动时的定义，
或本次会话最近确认的参数；该按钮本身不写文件。确认后继续调整，需要再次确认才会保存。
重启工具会读取已写入的掌心定义。

保存后的 XML 形如：

```xml
<link name="palm_center"/>
<joint name="palm_center_joint" type="fixed">
  <parent link="right_hand_link"/>
  <child link="palm_center"/>
  <origin xyz="0.01 0.02 0.03" rpy="0 0 0"/>
</joint>
```

上述数值只是格式示例，不是 XHand 推荐标定值。保存 URDF 后，训练端仍需显式使用
该 frame；不会自动改变策略输入定义。接入方法见下文“Wuji 虚拟掌心与 RL training”。

### 5. 测试物体、掌心参考与实时 observation

进入 **Object / observation · 物体与观测**：

1. 拖动黄色物体的平移／旋转控件，改变物体位姿。
2. 修改 `物体尺寸 xyz (m)` 和 `Keypoint scale`。
3. 修改关节或掌心 xyz/rpy，观察数值、指尖点和物体 keypoint 的实时变化。
4. 用 `显示指尖 / keypoint 相对向量` 控制连线显示。
5. 用 `重置物体姿态与尺寸` 恢复初始物体：位置 `(0.45, 0, 0.56)` m、无旋转、
   尺寸 `(0.04, 0.04, 0.04)` m、keypoint scale `1.5`。

当前测试物体是可调尺寸的长方体，不提供任意物体 mesh 导入入口。
物体 keypoint 使用四个固定角点符号 `+++ / ++- / --+ / ---`，由半尺寸乘 keypoint scale
后按物体姿态变换。scale 为 1 时在长方体角点上；默认 1.5 会在几何体之外。

`Observation 掌心参考` 决定观测使用的点和朝向：

| 选项 | 位置／朝向来源 |
| --- | --- |
| `virtual` | 当前调节后的虚拟掌心 |
| `semantic` | 页面所选参考 link 的原点和朝向 |
| `legacy`（Wuji） | 历史 `link7` 原点和朝向 |
| `legacy`（Sharpa） | Kuka 腕部旋转后的 `(0, -0.02, 0.16)` m 偏移点，朝向沿用腕部 |

XHand/未知型号不提供 legacy。紫色球是调节中的虚拟掌心，绿色标记及坐标轴是当前
observation 参考；选择 virtual 时两者重合。即使 observation 选择 semantic/legacy，
确认保存的仍是 xyz/rpy 控件中的虚拟掌心定义。

#### 指尖来源

| 手型 | 观测指尖定义 | 顺序 |
| --- | --- | --- |
| Wuji | `right_finger*_link4` 加历史标定偏移 | thumb/index/middle/ring/pinky |
| Sharpa | 对应 DP link 加 `(0.02, 0.002, 0)` m 局部偏移 | index/middle/ring/thumb/pinky |
| XHand | URDF tip frame 原点，未标定接触点 | thumb/index/middle/ring/pinky |
| 未知型号 | 用户填写的 link 原点 | 输入顺序 |

未知模型在 `指尖 link（逗号分隔，可为空）` 中输入实际 link 名称。
留空时不计算指尖分量，但仍显示掌心和物体观测；不会自动把所有末端 link 当作指尖。

#### 观测字段、维度和坐标约定

单模型面板提供以下几何向量，`N` 为指尖数量：

| 字段 | 维度 | 五指时的区间（左闭右开） | 定义 |
| --- | --- | --- | --- |
| `palm_pos` | 3 | `[0:3]` | 当前观测掌心在机器人基坐标系中的位置 |
| `palm_rot_xyzw` | 4 | `[3:7]` | 当前观测掌心朝向，quaternion 为 xyzw |
| `fingertip_pos_rel_palm` | `3N` | `[7:22]` | 各指尖位置减去掌心位置 |
| `keypoints_rel_palm` | 12 | `[22:34]` | 四个物体 keypoint 位置减去掌心位置 |

合计 `19 + 3N` 维：五指为 **34D**，不选择指尖时为 **19D**。
页面另列 `object_rot_xyzw`，它不拼入上述向量。动态区间以面板／快照中的
`geometry_slices` 为准。

位置使用 m，角度使用 rad。相对向量是在机器人基坐标轴下相减，
**没有乘掌心逆旋转**；旋转掌心朝向本身不会把相对向量改成掌心局部坐标。
物体控件内部使用 wxyz，导出的 quaternion 字段使用 xyzw。

该向量对应原工具的 critical geometry block，**不是完整 134D/140D actor observation**。
单模型模式没有仿真速度、前一动作或目标状态，不会填造这些字段；XHand 的 34D 几何
也不意味着能直接输入 Wuji/Sharpa 的旧策略。

### 6. JSON 快照与文件保存的区别

点击 **Print snapshot JSON**，快照打印到启动工具的终端，不会自动下载或写成 JSON 文件。
单模型快照包含：

| 内容 | 字段 |
| --- | --- |
| 模型与参考 | `hand_type`、`anchor`、`palm_parent`、坐标／定义说明 |
| 关节配置 | `joint_names`、`joint_positions_raw`；数值未归一化 |
| 掌心参数 | `palm_local_xyz`、`palm_local_rpy`、虚拟掌心位置／朝向 |
| 物体状态 | `object_position`、`object_rot_xyzw`、`object_size`、`keypoint_scale` |
| 指尖／keypoint | `tip_links`、`tip_offsets_local`、`fingertip_positions`、`object_keypoints` |
| 观测 | `geometry_fields`、`geometry_slices`、`geometry_vector` |

需要留存整个运行日志，可在启动命令末尾加 `2>&1 | tee viewer.log`。
该日志同时包含服务输出和点击时打印的 JSON，不是单个纯 JSON 文档。

**确认掌心并写入 URDF** 保存模型定义；**Print snapshot JSON** 记录当前调试状态。
两个按钮独立，不会互相触发。原三模型模式也支持快照，但使用 old/new/SimToolReal 对照结构，
并非上述单模型 schema。

### 7. 远程访问和命令行参数

服务器上直接提供页面：

```bash
.venv/bin/python -m tools.urdf_observation_viewer \
  --urdf xarm7_xhand_direct.urdf --host 0.0.0.0 --port 8080
```

本机浏览器访问 `http://服务器IP:8080`。如果使用 SSH 端口转发，服务器端使用
`--host 127.0.0.1 --port 8080`，本机另开终端执行：

```bash
ssh -L 8080:127.0.0.1:8080 user@server
```

然后打开本机 `http://localhost:8080`；按实际账号和服务器地址替换 `user@server`。

| 参数 | 默认值／作用 |
| --- | --- |
| `--urdf PATH` | 指定后进入单模型模式；不指定则进入历史三模型模式 |
| `--palm-link NAME` | 覆盖单模型默认掌心参考 link；必须与 `--urdf` 同用 |
| `--old-urdf PATH` | 对照模式 old Wuji；默认 `xarm7_wuji_90.urdf` |
| `--new-urdf PATH` | 对照模式 new Wuji；默认 `xarm7_wuji_direct_camera.urdf` |
| `--simtoolreal-urdf PATH` | 对照模式 Kuka + Sharpa；默认上文资产路径 |
| `--host ADDRESS` | 默认 `0.0.0.0`；仅本机访问可用 `127.0.0.1` |
| `--port NUMBER` | 默认 `8080`；占用时改成 `8081` 等空闲端口 |
| `--help` | 查看命令行帮助 |

指定 `--urdf` 时，不使用三个历史对照资产参数；单模型模式只加载所选模型。

### 8. 常见问题

| 现象 | 处理方法 |
| --- | --- |
| 页面仍是旧界面／没有新按钮 | 停止旧 Python 进程，用当前仓库 `.venv` 重新启动，再刷新浏览器 |
| 进入了三模型界面，想查看 XHand | 添加 `--urdf xarm7_xhand_direct.urdf` |
| 掌心在 mesh 内看不清 | 使用默认 25 mm 大球，或暂时关闭 visual；精调时切换小球 |
| XHand 掌心初始位置不合适 | 无已保存 frame 时从 `right_hand_link` 原点开始，需调节并确认保存 |
| 未知手型没有指尖向量 | 在指尖输入框填写实际 link 名称；空输入不会猜测指尖 |
| 页面显示物体尺寸／指尖错误 | 尺寸各分量必须为正且有限，指尖名称必须存在；修正后恢复观测和快照导出 |
| 点击保存失败 | 看面板错误；检查文件／目录写权限、父 link、重名节点、是否形成环，以及已有掌心是否为 fixed joint |
| 修改了 xyz，重启却没有变化 | 确认是否点击保存成功，且重启时加载的是面板显示的同一个文件 |
| 找不到 mesh | 保留 URDF 对应目录结构和网格文件，检查相对路径和大小写 |
| 远程页面打不开 | 检查启动输出中的端口、host 和转发设置；也可通过 SSH 转发访问 |
| 34D 向量不能用于旧 policy | 它只是几何分量，完整训练 observation 还涉及关节归一化、速度、动作历史及任务字段 |

### 9. 测试与工具代码入口

```bash
# 普通 venv
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q tests

# 如果是 uv 环境，安装测试依赖改用：
uv pip install --python .venv/bin/python -r requirements-dev.txt
```

| 文件 | 职责 |
| --- | --- |
| [tools/urdf_observation_viewer.py](tools/urdf_observation_viewer.py) | CLI、历史三模型对照、共享 URDF 渲染和几何计算 |
| [tools/palm_inspector.py](tools/palm_inspector.py) | 单模型识别、关节控制、掌心精调、物体与观测交互 |
| [tools/urdf_palm.py](tools/urdf_palm.py) | 确认后的掌心 frame 写入、更新和结构检查 |
| [requirements.txt](requirements.txt) | 独立工具依赖 |
| [requirements-dev.txt](requirements-dev.txt) | 测试依赖 |

当前 22 项测试覆盖已知／未知模型、FK 和历史几何一致性、物体和掌心变换、掌心写入与重复保存。
另已通过实际 Viser WebSocket 事件验证 XHand 的控件更新、快照、大／小球切换、精调、
确认写入和恢复；这些检查未替代浏览器画面的人工检查或动力学仿真验证。

## ⚠️ 使用前须知

- `direct` 构型的 **u start pose 必须设置为**：

  ```text
  [0 0 0 0 pi pi/2 0]
  ```

  这里的 `u` 指机械臂的 7 个关节起始配置；角度使用弧度，`pi` 和 `pi/2` 请在加载配置时转换为数值。
- 所有 URDF 模型均可正常使用。使用相机模型时请确认 `meshes/camera/d405.stl` 路径可访问。
- URDF 中的 `visual` 用于显示，`collision` 用于碰撞检测；不要随意删除碰撞网格，否则会影响规划和仿真结果。

## 文件结构

```text
.
├── README.md
├── requirements.txt                 # 查看器独立运行依赖
├── requirements-dev.txt             # 测试依赖
├── tools/
│   ├── urdf_observation_viewer.py    # CLI、历史对照与共享渲染
│   ├── palm_inspector.py            # 单模型掌心与 observation 交互
│   └── urdf_palm.py                 # 确认写入 URDF
├── docs/                            # 环境与历史 observation 说明
├── tests/                           # 几何检查
├── assets/urdf/kuka_sharpa_description/  # Kuka + 左 Sharpa URDF 与网格
├── xarm7_gripper.urdf                 # xArm 原厂平行夹爪
├── xarm7_wuji_direct.urdf             # xArm7 + Wuji 灵巧手，直装
├── xarm7_wuji_direct_camera.urdf      # 直装 Wuji + 相机支架 + 双 D405
├── xarm7_wuji_90.urdf                 # xArm7 + Wuji 灵巧手，90° 适配
├── xarm7_wuji_90_camera.urdf          # 90° 适配 Wuji + 相机支架 + 双 D405
├── xarm7_xhand_direct.urdf            # xArm7 + XHand，直装
├── xarm7_xhand_direct_camera.urdf     # 直装 XHand + 双 D405
└── meshes/
    ├── xarm7_1305/                    # xArm7 本体网格
    │   ├── visual/                    # DAE/STL，可视化网格
    │   └── collision/                 # OBJ，碰撞网格
    ├── wuji/                          # Wuji 五指灵巧手网格
    ├── xhand/                          # XHand 网格及凸包碰撞网格（OBJ）
    ├── gripper/                        # xArm 平行夹爪网格
    ├── adapter/                        # 末端适配器、相机支架
    └── camera/                         # Intel RealSense D405 外形网格
```

xArm 模型的网格路径以仓库根目录为基准；Kuka + Sharpa 网格路径以其 URDF 所在目录为基准。部分 STL 来源以毫米建模，URDF 中已通过 `scale="0.001 0.001 0.001"` 或对应的位姿变换换算到米制；导入其他软件时请保持这一单位约定。

## URDF 内部组成

每个模型都由以下几类元素组成：

- `link`：刚体部件，例如 xArm7 的 `link1`~`link7`、手掌和各指节。
- `joint`：连接父子 `link` 的关节；机械臂和手指的运动关节通常是 `revolute`，适配器、指尖和相机使用 `fixed`。
- `visual`：用于渲染的外观网格，通常来自 `meshes/*/visual` 或对应的 STL/DAE 文件。
- `collision`：用于碰撞检测的简化网格，通常来自 OBJ、碰撞 STL 或凸包网格。
- `inertial`：质量、质心和惯量参数，供动力学仿真使用。
- `origin` / `rpy`：描述部件之间的平移和旋转；`rpy` 按 roll–pitch–yaw（弧度）解释。

## URDF 构型说明

| 文件 | 末端执行器/附件 | 安装方式与特点 | 状态 |
| --- | --- | --- | --- |
| `xarm7_gripper.urdf` | xArm 原厂平行夹爪 | 直接安装在 `link7`，包含左右手指、内外指节和 `link_tcp` | ✅ 可用 |
| `xarm7_wuji_direct.urdf` | Wuji 五指灵巧手 | 通过 `wuji_xarm_adapter_1` 直装，末端坐标由 `joint_eef` 固定连接 | ✅ 可用；需设置 direct 起始 pose |
| `xarm7_wuji_direct_camera.urdf` | Wuji + 两个 D405 | 直装版本上增加 `camera_stand_link`、适配器和两个固定 D405 相机 | ✅ 可用；需设置 direct 起始 pose |
| `xarm7_wuji_90.urdf` | Wuji 五指灵巧手 | 适配器相对 `link7` 旋转约 90°（`rpy` 中可见 `-1.5708`），改变手的安装朝向 | ✅ 可用 |
| `xarm7_wuji_90_camera.urdf` | Wuji + 两个 D405 | 90° 适配版本；相机支架位于两段适配器之间，两个相机固定在正交安装面 | ✅ 可用 |
| `xarm7_xhand_direct.urdf` | XHand 五指灵巧手 | 通过 `xhand_adapter` 直装，含 `link_eef`、`link_tcp` 和拇指/食指/中指/无名指/小指关节链 | ✅ 可用；需设置 direct 起始 pose |
| `xarm7_xhand_direct_camera.urdf` | XHand + 两个 D405 | 在 XHand 直装模型上增加 `d405_camera_1_link`、`d405_camera_2_link` 固定相机 | ✅ 可用；需设置 direct 起始 pose |

### 名称中的关键词

- `xarm7`：UFACTORY xArm7 七自由度机械臂本体（`link_base` 至 `link7`）。
- `wuji`：Wuji 五指灵巧手；每根手指通常由多个 `revolute` 关节和一个固定指尖组成。
- `xhand`：XHand 五指灵巧手；关节命名以 `right_hand_*` 开头，并带有独立的末端/工具坐标链接。
- `gripper`：xArm 原厂平行夹爪，使用 `drive_joint` 等联动关节控制夹持开合。
- `direct`：末端适配器按直装姿态连接到 `link7`，**必须使用指定的 u start pose**。
- `90`：Wuji 适配器旋转约 90° 的安装版本；它不是机械臂关节 90° 限位。
- `camera`：在末端增加两个固定的 Intel RealSense D405 外形模型及安装支架；相机没有额外的运动自由度。

## 起始位姿配置

仅名称包含 `direct` 的构型需要统一设置机械臂 `u` 的起始位姿：

```python
u_start = [0, 0, 0, 0, pi, pi / 2, 0]
# 若接口只接受浮点数：
u_start = [0.0, 0.0, 0.0, 0.0, 3.141592653589793,
           1.5707963267948966, 0.0]
```

适用文件：

1. `xarm7_wuji_direct.urdf`
2. `xarm7_wuji_direct_camera.urdf`
3. `xarm7_xhand_direct.urdf`
4. `xarm7_xhand_direct_camera.urdf`

`xarm7_wuji_90*.urdf` 和 `xarm7_gripper.urdf` 不属于 direct 构型；如上层控制器有额外的安全初始姿态要求，请以控制器配置为准。

## Wuji self-collision 过滤 U 组

四个 Wuji 构型在后续创建 Isaac Scene 时，统一通过环境侧的 pair-wise collision filter 处理；原始 URDF 文件保持不修改。针对手掌与手指基部的结构性重叠，所有构型都需要过滤以下 4 对 link：

```text
U_wuji_self_collision = {
  (right_finger2_link2, right_palm_link),
  (right_finger3_link2, right_palm_link),
  (right_finger4_link2, right_palm_link),
  (right_finger5_link2, right_palm_link),
}
```

这些 link2 与手掌在正常装配时存在几何重叠/过近，属于需要忽略的相邻结构碰撞。`finger1_link2` 不在过滤组内，应保留正常碰撞检测。

关于 `link7` 相关过滤，请按构型处理：

- **直连构型**（`xarm7_wuji_direct*.urdf`）：`joint_eef` 将手掌放在 `link7` 法兰前方约 30 mm，默认不需要过滤 `link7 ↔ right_palm_link`。如果沿用已有直连场景配置，应保留 `link7 ↔ right_finger{2,3,4,5}_link2` 这组过滤；同时加入上面列出的 `right_finger{2,3,4,5}_link2 ↔ right_palm_link` 过滤。
- **90° 转接构型**（`xarm7_wuji_90*.urdf`）：腕部、90° 转接件和手掌的相对位置不同，`link7 ↔ right_palm_link` 可能出现结构性接触。只有在 Isaac Scene 的实际碰撞几何确认存在重叠时，才将该 pair 加入 90° 构型的 U 组。

因此，`link7 ↔ right_palm_link` 不是全局错误，也不是所有构型都必须过滤的固定规则；应根据构型和实际加载的碰撞几何单独决定。其余 self-collision 保持开启。

## Wuji 虚拟掌心与 RL training

四个 `xarm7_wuji_*.urdf` 均定义同名虚拟坐标系 `palm_center`；
XHand 和 gripper 暂不添加。该 frame 通过固定关节挂在 `right_palm_link` 下：

```xml
<link name="palm_center"/>
<joint name="palm_center_joint" type="fixed">
  <parent link="right_palm_link"/>
  <child link="palm_center"/>
  <origin xyz="0.004 -0.0023529 0.0434161" rpy="0 0 0"/>
</joint>
```

位置单位为米，即手掌局部坐标 `(+4.0000, -2.3529, +43.4161) mm`。
X 是交互调试后的设定，Y/Z 来自掌体网格包围盒中心；它是 observation 参考点，
不是已标定的接触面中心，也不是质心。`rpy=0` 表示坐标轴与手掌同向，
并不表示与 `link7` 同向。四种安装链各自决定它相对机械臂末端的变换。

该 link 不包含 visual、collision 或 inertial，也不增加可动自由度；
可视化时显示的球体只是调试标记，不应作为碰撞几何写入训练资产。

### 训练环境需要接入的内容

仅更新 URDF 不会自动切换策略 observation，也不需要改 PPO/SAPG 等优化算法。
环境初始化时明确指定参考刚体与目标 frame，例如：

```yaml
# 示例配置字段，由训练环境自行实现读取，并非 URDF/Isaac 的内置开关。
palm_reference_body: link7
palm_frame: palm_center
```

1. 从选中的原始 URDF 沿固定关节链求出 `T_link7_palm_center`，
   包含全部适配器、相机支架和手掌安装变换。在初始化时计算并缓存；
   不要将 `(0.004, -0.0023529, 0.0434161)` 直接当作 link7 下的偏移。
   不同构型分别求解，不跨构型复用末端变换。缺少 frame 或链上包含可动关节时应明确报错。
2. 导入开启 `merge_fixed_joints=True`（Isaac Sim/Lab）或
   `collapse_fixed_joints=True`（Isaac Gym）时，`palm_center` 可能不再是独立刚体。
   可继续保留合并，每步从实际保留的 `link7` 刚体位姿重建虚拟掌心；
   如果导入器保留的是其他参考刚体，需按实际 body/frame 重新求固定变换。
   不要直接假设 `find_bodies("palm_center")` 能查到。
3. 用虚拟掌心统一生成 `palm_pos`、`palm_rot`、
   `fingertip_pos_rel_palm` 和 `keypoints_rel_palm`，并检查 reward、
   critic state、成功条件中使用 palm 的位置，避免混用 link7 与掌心语义。
4. 多环境中，位置减去 env origin；坐标轴方向仍按环境的约定处理。
   保持字段顺序和 quaternion 约定一致，例如 Isaac Lab 内部 wxyz，
   如策略约定 xyzw，则在拼 observation 时转换。
5. 若使用 palm 速度，计算偏移点线速度；下面公式要求输入为参考 link
   **原点**的线速度。如果仿真器返回质心速度，应先转换至参考 link 原点，
   或直接使用质心到虚拟掌心的位移，不能混用两种位置与速度。

令 `T_A_B` 将 B 中的坐标变换到 A，训练每步的几何关系为：

```python
# 初始化：从完整 URDF 固定链取得，按构型缓存。
T_link7_palm = urdf.get_transform("palm_center", "link7")  # yourdfpy 示例
r = T_link7_palm[:3, 3]
R_offset = T_link7_palm[:3, :3]

# 每步：以下是单环境矩阵记法；批量训练使用对应张量运算。
offset_world = R_world_link7 @ r
p_world_palm = p_world_link7 + offset_world
R_world_palm = R_world_link7 @ R_offset

palm_pos = p_world_palm - env_origin
palm_rot = matrix_to_policy_quaternion(R_world_palm)  # 按策略约定实现
fingertip_pos_rel_palm = fingertips_world - p_world_palm
keypoints_rel_palm = object_keypoints_world - p_world_palm

v_world_palm = v_world_link7_origin + cross(omega_world_link7, offset_world)
omega_world_palm = omega_world_link7
```

上面的相对位置保持现有世界轴向量差约定，没有乘掌心逆旋转。
如果另行选择掌心局部坐标表达，则还需乘 `R_world_palm.T`，
这属于另一种 observation 定义，应显式区分。
掌心速度示例也使用世界轴；需要局部速度时应同样转换。

### 验证、checkpoint 与部署

- 先对每种构型跑小规模 FK 检查：在零位和一个非零关节位姿下，
  检查 `T_world_link7 @ T_link7_palm_center` 与原始 URDF 的 palm FK 一致；
  frame 添加前后都应保持 27 个可动关节。
- 在实际仿真导入后检查保留的 body/frame、掌心显示位置、quaternion 顺序，
  并做一次 observation shape/finiteness 和短步运行检查。
  这里的 URDF/FK 检查不能代替 Isaac 导入与动力学验证；
  frame-only link 若未被导入器保留，应使用上述变换重建方法。
- 旧策略若使用 link7 原点/朝向，新 palm 定义虽可保持 observation 维度不变，
  数值与含义仍发生变化。不要直接将旧 checkpoint 当作兼容策略；
  旧模型回放需保留旧 observation 约定，新训练或明确的微调使用新定义。
- 部署端、数据采集和训练端必须读取相同版本的 URDF，使用相同掌心 frame、
  位置/姿态/速度计算和关节顺序。将资产构型、掌心定义及 observation 版本记录在实验配置中。

## 在仿真器中加载

1. 将仓库根目录加入仿真器的资源搜索路径，确保 `meshes/...` 相对路径能够解析。
2. 选择与硬件末端执行器一致的 URDF 文件。
3. 对 `direct` 文件先设置上面的 `u_start`，再进行 IK、轨迹规划或动力学仿真。
4. 相机仅作为固定几何体建模；真实的 D405 内参、深度噪声和坐标系需在仿真器/驱动中另行配置。

## 维护建议

- 修改适配器或相机安装位姿后，同时检查 `visual`、`collision` 和对应 `joint` 的 `origin`。
- 新增网格时，保持目录分类和相对路径风格，并确认文件名大小写与 URDF 完全一致（Linux 区分大小写）。
- 修改相机安装位姿后，请重新标定两个 D405 的外参，并更新对应 URDF 中的固定关节位姿。
