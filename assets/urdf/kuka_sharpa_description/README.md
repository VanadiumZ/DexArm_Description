# Kuka + 左 Sharpa 历史资产

从 Interprior 的 `assets/urdf/kuka_sharpa_description/` 完整复制，
来源工具迁移基线提交为 `c5e968c39d7c7cbf16b3b97eb48e9ffcadae5b3e`。
保留原 URDF、网格文件及相对路径，用于 SimToolReal 风格训练的历史几何对照。
Interprior 内供历史训练使用的原资产保留，本查看器不依赖该仓库。

入口为 `iiwa14_left_sharpa_adjusted_restricted.urdf`，包含 7 个 Kuka 关节和
22 个左 Sharpa 关节。网格位于 `new_iiwa14_meshes/`、`left_sharpa_meshes/`
及 `sharpa_mount.stl`。导入时保持此目录结构。
