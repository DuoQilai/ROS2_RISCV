# auto_explorer

基于实时 SLAM 地图的边界探索与地图保存，使用 C++17。Nav2 执行目标导航，节点处理目标选择、取消、场内覆盖率和保存。

构建使用仓库统一入口，双端环境与运行步骤见[第23章教案](../../teaching_docs_k3_com260_kit/ch23_SLAM与导航综合实训.md)。

`config/project_museum.sdf` 将原 Museum 水平放大 1.5 倍；原模型与实验场景不变。`scripts/prepare_museum.py` 从原碰撞网格的 0.18 m 高度截面生成 0.05 m 栅格，并加入 0.25 m 方形安全边界（包含 0.22 m 机器人半径），取出生点连通区域。白色区域共 691.7325 m²，用于固定覆盖率分母；墙外、障碍物内部和不连通区域不计入。该掩码不参与探索选点。

修改场景后可从仓库运行 `python3 src_k3_com260_kit/auto_explorer/scripts/prepare_museum.py` 重新生成场景与掩码（需要 Pillow）。正常实验直接使用随包安装的文件。
