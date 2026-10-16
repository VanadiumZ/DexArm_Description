# 虚拟掌心查看器环境配置

查看器使用普通 CPU Python 环境，不需要 Interprior、Isaac Sim/Lab、CUDA 或 ROS。
使用 Python 3.11；直接依赖版本固定在 `requirements.txt`，测试依赖在 `requirements-dev.txt`。
这些版本沿用迁移时的查看器环境，不代表训练环境的依赖要求。

## 安装

在仓库根目录运行：

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m tools.urdf_observation_viewer --host 127.0.0.1 --port 8080
```

也可使用 uv：

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements.txt
.venv/bin/python -m tools.urdf_observation_viewer --host 127.0.0.1 --port 8080
```

浏览器打开 `http://localhost:8080`。服务器无需桌面或 GPU，渲染在浏览器中进行。
在远程服务器使用时，可在本机运行 `ssh -L 8080:127.0.0.1:8080 user@server`，
然后访问同一地址。需要直接通过服务器地址访问时，使用 `--host 0.0.0.0`。
端口被占用时通过 `--port 8081` 改用其他端口。

## 默认资产和参数

默认同时加载以下三套模型及 visual/collision mesh：

- `xarm7_wuji_90.urdf`：侧装 Wuji（界面 old）。
- `xarm7_wuji_direct_camera.urdf`：直装 Wuji + 相机（界面 new）。
- `assets/urdf/kuka_sharpa_description/iiwa14_left_sharpa_adjusted_restricted.urdf`：历史 Kuka + 左 Sharpa。

默认路径相对脚本定位的仓库根目录解析。可使用 `--old-urdf`、`--new-urdf`、
`--simtoolreal-urdf` 指定替代文件；相对参数路径以当前工作目录为基准。
上述三个对照参数仅用于原历史对照模式；Wuji 参数要求相同的 27 个关节名称以及 `link7`、`right_palm_link`、`palm_center` 等 frame，
不能在这些对照参数中直接换成 XHand 或 gripper；请改用下方 `--urdf` 单模型模式。Kuka 参数要求原来的 29 自由度和 Sharpa link 命名。
其他安装构型的初始姿态与历史 observation 对照结果需按实际模型解释。

`Virtual palm tuning · 手掌局部坐标` 调节掌心局部 xyz，实时显示可复制的 URDF origin XML；
调整过程只预览；点击“确认掌心并写入 URDF”后直接更新当前文件。
三模型对照界面分别点击 old/new 确认按钮。`Print snapshot JSON` 只将当前几何快照打印到启动终端。

## 单模型模式

```bash
.venv/bin/python -m tools.urdf_observation_viewer --urdf xarm7_xhand_direct.urdf
.venv/bin/python -m tools.urdf_observation_viewer --urdf xarm7_gripper.urdf --palm-link link_tcp
```

Wuji、Sharpa、XHand 按关节与 link 签名识别，使用 `tools/palm_inspector.py` 中的固定映射。
未知模型从 URDF 自动读取独立关节、类型和限位；mimic 关节由 URDF 库随源关节更新。
旋转单位 rad，移动单位 m；无限位关节使用不设上下界的数值控件。
默认掌心 link 和识别范围见 README。可用 `--palm-link` 覆盖默认值，也可在页面选择。
切换参考 link 会重新读取已有 palm_center 变换，或恢复该型号的初始偏移。
掌心位置和姿态分别通过 xyz / rpy 调整；页面提供完整 fixed joint/link XML。
确认按钮保存当前 parent link、xyz/rpy，并在面板显示成功或失败；重复保存更新同一掌心定义。
已有非固定掌心关节、重复定义或会形成环的参考 link 会拒绝保存。
文件使用完整写入后替换的方式更新，保留其他模型定义、注释和网格路径。
恢复按钮使用最近确认的掌心参数。XHand 和未知模型使用明确列出的几何字段，不宣称匹配历史策略输入。

单模型模式也可拖动黄色物体、修改尺寸/姿态和 keypoint scale；
observation 几何数值与指尖/keypoint 相对向量随关节、物体和掌心调整实时更新。
使用 `Observation 掌心参考` 切换 virtual/semantic，以及 Wuji/Sharpa 的 legacy 定义。
使用 `Print snapshot JSON` 输出当前快照；字段定义和详细操作见 README。

## 检查

```bash
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q tests
.venv/bin/python -m tools.urdf_observation_viewer --help
```

若使用 uv 创建的环境（默认没有 pip），安装测试依赖使用：

```bash
uv pip install --python .venv/bin/python -r requirements-dev.txt
```

测试覆盖掌心固定 frame、安装变换、历史 Sharpa 偏移和 observation 几何关系。
启动查看器会实际加载三套模型的网格；缺少文件时请检查是否完整克隆了资产目录，
不要只复制 URDF。该工具只做 FK 与几何显示，不运行动力学训练。

## 本次迁移验证

在 Linux / Python 3.11.14 的独立 `.venv` 中安装上述依赖后，5 项几何测试全部通过；
三套模型的 218 处网格引用均存在，查看器完成三套模型初始化，HTTP 页面请求返回 200。
未进行浏览器人工交互检查或动力学仿真。

单模型功能扩展后，全部 15 项测试通过，覆盖仓库内 7 个已知手型构型、通用夹爪、
未知模型的 prismatic/continuous/mimic 关节与固定 frame XML 导出。
原三模型对照、单模型 Wuji/Sharpa/XHand 和通用夹爪均完成初始化及 HTTP 200 检查；
尚未进行浏览器人工交互验证。

物体/observation 扩展后共 19 项测试通过，其中验证物体平移与旋转、掌心局部偏移、
未知模型可选指尖、无效输入，以及单模型 Wuji/Sharpa 与旧几何计算的一致性。

另通过真实 Viser WebSocket 控件事件验证 XHand：拖动物体、修改掌心 xyz、物体尺寸、
关节角度均触发实时 observation 更新，点击 JSON 导出后检查快照中的值与输入一致。
该检查覆盖交互数据流，未替代浏览器画面的人工检查。

确认写入功能加入后共 22 项测试通过，验证新增/更新掌心、保留其他模型元素和注释、
拒绝环及无效参数。另通过真实 Viser WebSocket 按钮事件在临时 XHand URDF 副本上验证
写入、重复确认不重复添加、恢复最近确认值及 JSON 导出；测试未修改仓库机器人资产。
