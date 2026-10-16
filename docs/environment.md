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
Wuji 参数要求相同的 27 个关节名称以及 `link7`、`right_palm_link`、`palm_center` 等 frame，
不支持直接换成 XHand 或 gripper。Kuka 参数要求原来的 29 自由度和 Sharpa link 命名。
其他安装构型的初始姿态与历史 observation 对照结果需按实际模型解释。

`Virtual palm tuning · 手掌局部坐标` 调节掌心局部 xyz，实时显示可复制的 URDF origin XML；
不会自动写入 URDF。`Print snapshot JSON` 将当前几何快照打印到启动终端。

## 检查

```bash
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q tests/test_urdf_observation_viewer.py
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
