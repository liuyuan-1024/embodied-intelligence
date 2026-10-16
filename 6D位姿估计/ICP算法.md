# ICP（Iterative Closest Point）算法

## 1. ICP 要解决的问题

ICP 用于配准两组描述同一刚体表面的三维点。输入通常包括：

- **source 点云**：待变换的点集 $Q=\{\mathbf q_i\}$；
- **target 点云**：作为参考坐标系的点集 $P=\{\mathbf p_j\}$；
- **初始变换**：source 到 target 的近似刚体变换 $\mathbf T_0$。

算法输出一个从 source 坐标系到 target 坐标系的变换：

$$
\mathbf T_{TS}
=
\begin{bmatrix}
\mathbf R_{TS} & \mathbf t_{TS} \\
\mathbf 0^\mathsf T & 1
\end{bmatrix},
\qquad
\mathbf p_T=\mathbf R_{TS}\mathbf p_S+\mathbf t_{TS}.
$$

其中 $\mathbf R\in SO(3)$ 表示旋转，$\mathbf t\in\mathbb R^3$ 表示平移。合起来是 6 个自由度。

ICP 不直接理解物体类别或图像语义。它根据几何位置建立点对应关系，再求一个能减小几何残差的刚体变换。原始 ICP 及其性质可参考 [Besl 和 McKay（1992）](https://doi.org/10.1109/34.121791)；Open3D 的实现说明见[官方 ICP 教程](https://www.open3d.org/docs/latest/tutorial/pipelines/icp_registration.html)。

## 2. ICP 的直观过程

设当前变换是 $\mathbf T_k$。ICP 反复执行两步：

1. **建立对应关系**：将 source 点按 $\mathbf T_k$ 变换，在 target 中为每个 source 点找最近邻。
2. **更新变换**：在本轮对应关系固定的条件下，求一个新的刚体变换，使对应点残差变小。

之后重新计算最近邻，再更新变换。这个交替过程持续到更新很小或达到迭代上限。

~~~mermaid
flowchart LR
    A["source 点云 Q"] --> B["用当前 T 变换 source"]
    C["target 点云 P"] --> D["最近邻搜索"]
    B --> D
    D --> E["距离阈值与对应点筛选"]
    E --> F["最小化几何残差，求变换更新"]
    F --> G["更新 T"]
    G --> B
    G --> H["收敛条件或迭代上限"]
~~~

ICP 常被称为局部配准方法。最近邻对应关系取决于当前位姿；如果初始位姿偏差较大，算法可能把几何上相近但实际不对应的表面配成一对，之后在错误对应上继续优化。

## 3. 数学目标：对应关系与残差

### 3.1 最近邻对应关系

将 source 点 $\mathbf q_i$ 变换到 target 坐标系：

$$
\mathbf x_i^{(k)}=\mathbf R_k\mathbf q_i+\mathbf t_k.
$$

在 target 中寻找最近邻：

$$
c_i^{(k)}
=
\arg\min_j
\left\|\mathbf x_i^{(k)}-\mathbf p_j\right\|_2.
$$

如果最近邻距离大于阈值 $\tau$，该点通常不作为本轮有效对应点。对应集合可写成：

$$
\mathcal K_k=
\left\{(i,c_i^{(k)}):
\left\|\mathbf x_i^{(k)}-\mathbf p_{c_i^{(k)}}\right\|_2<\tau
\right\}.
$$

距离阈值的单位必须与点云坐标单位一致。YCB-V BOP 点云使用毫米时，$\tau$ 也应以毫米设置。

### 3.2 点到点目标函数

最常见的点到点 ICP 在当前对应集合上最小化：

$$
E_{p2p}(\mathbf R,\mathbf t)
=
\sum_{(i,j)\in\mathcal K_k}
\left\|
\mathbf p_j-(\mathbf R\mathbf q_i+\mathbf t)
\right\|_2^2.
$$

这是一个关于刚体变换的最小二乘问题。若对应关系固定，可用 SVD 得到闭式解；ICP 的非线性和局部性主要来自每轮重新分配最近邻对应关系。

可把完整流程写成：

$$
\mathbf T_k
\xrightarrow{\text{最近邻匹配}}
\mathcal K_k
\xrightarrow{\text{刚体最小二乘}}
\mathbf T_{k+1}.
$$

## 4. 固定对应关系时的 SVD 刚体配准

本节说明点到点 ICP 每次更新的核心计算。给定已经匹配的 source 点 $\mathbf q_i$ 和 target 点 $\mathbf p_i$，求：

$$
\min_{\mathbf R,\mathbf t}
\sum_i
\left\|
\mathbf p_i-(\mathbf R\mathbf q_i+\mathbf t)
\right\|_2^2,
\quad
\mathbf R^\mathsf T\mathbf R=\mathbf I,
\quad
\det(\mathbf R)=1.
$$

### 4.1 平移由质心关系确定

计算两组点的质心：

$$
\bar{\mathbf q}=\frac{1}{N}\sum_i\mathbf q_i,
\qquad
\bar{\mathbf p}=\frac{1}{N}\sum_i\mathbf p_i.
$$

给定旋转 $\mathbf R$ 时，最优平移为：

$$
\mathbf t=\bar{\mathbf p}-\mathbf R\bar{\mathbf q}.
$$

因此可以先把两组点中心化，再求旋转。

### 4.2 协方差矩阵与 SVD

令中心化点为 $\mathbf q_i'=\mathbf q_i-\bar{\mathbf q}$、$\mathbf p_i'=\mathbf p_i-\bar{\mathbf p}$，构造：

$$
\mathbf H=\sum_i\mathbf q_i'{\mathbf p_i'}^\mathsf T.
$$

中心化后，平移项消失，目标函数可写为：

$$
E(\mathbf R)=\text{常数}-2\operatorname{tr}(\mathbf H\mathbf R).
$$

因此，固定对应关系时，求最小平方误差等价于在旋转约束下最大化 $\operatorname{tr}(\mathbf H\mathbf R)$。SVD 将该问题化为三个主轴方向上的对齐问题。

对 $\mathbf H$ 做奇异值分解：

$$
\mathbf H=\mathbf U\mathbf\Sigma\mathbf V^\mathsf T.
$$

在列向量约定、目标为 $\mathbf p_i\approx\mathbf R\mathbf q_i+\mathbf t$ 时：

$$
\mathbf R=
\mathbf V
\begin{bmatrix}
1&0&0\\
0&1&0\\
0&0&\det(\mathbf V\mathbf U^\mathsf T)
\end{bmatrix}
\mathbf U^\mathsf T,
\qquad
\mathbf t=\bar{\mathbf p}-\mathbf R\bar{\mathbf q}.
$$

行列式修正确保结果是纯旋转而非包含镜像反射的正交矩阵。该计算也常称为 Kabsch 刚体配准。

下面的代码函数只解决**固定对应点**下的刚体配准，不包含最近邻查找和 ICP 迭代：

~~~python
import numpy as np

def rigid_transform_svd(source, target):
    """求 source 到 target 的刚体变换；两数组按行保存对应点。"""
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)

    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center

    H = source_zero.T @ target_zero
    U, _, Vt = np.linalg.svd(H)

    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ correction @ U.T
    t = target_center - R @ source_center

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T
~~~

## 5. 两类常见残差

### 5.1 点到点 ICP

对每对对应点计算三维向量差：

$$
\mathbf r_i
=
\mathbf p_i-(\mathbf R\mathbf q_i+\mathbf t),
\qquad
E=\sum_i\|\mathbf r_i\|_2^2.
$$

特点：

- 不要求点云带法线；
- 目标和 source 的采样密度、噪声会直接影响最近邻距离；
- 固定对应关系时可用 SVD 闭式求解；
- 适合作为入门实现和几何初值细化基线。

当前阶段三 Notebook 使用点到点 ICP。

### 5.2 点到平面 ICP

点到平面 ICP 使用 target 对应点处的法线 $\mathbf n_i$，只惩罚沿法线方向的偏差：

$$
r_i
=
\mathbf n_i^\mathsf T
\left(\mathbf R\mathbf q_i+\mathbf t-\mathbf p_i\right),
\qquad
E_{p2l}=\sum_i r_i^2.
$$

若当前 source 点已变换到 target 坐标系，记 $\mathbf x_i=\mathbf R\mathbf q_i+\mathbf t$。对一个很小的位姿更新 $(\delta\mathbf v,\delta\boldsymbol\omega)$，一阶近似为：

$$
\mathbf x_i'
\approx
\mathbf x_i+\delta\mathbf v+
\delta\boldsymbol\omega\times\mathbf x_i.
$$

于是残差近似为：

$$
r_i'
\approx r_i+
\begin{bmatrix}
\mathbf n_i^\mathsf T &
(\mathbf x_i\times\mathbf n_i)^\mathsf T
\end{bmatrix}
\begin{bmatrix}
\delta\mathbf v\\
\delta\boldsymbol\omega
\end{bmatrix}.
$$

每个对应点由此产生一个关于 6 个小位姿增量的线性方程。将这些方程堆叠后，可用加权最小二乘或 Gauss–Newton 方法求增量，再将增量组合到当前位姿上。

点到平面方法在局部几何近似成立时通常收敛较快；它需要可靠法线。平坦表面沿切向的移动对该残差影响很小，因此几何约束不足时会出现某些自由度难以确定的情况。Open3D 对点到点和点到平面目标函数有[公式与接口说明](https://www.open3d.org/docs/latest/tutorial/pipelines/icp_registration.html)。

## 6. ICP 的完整迭代步骤

给定 source $Q$、target $P$、初始 source-to-target 变换 $T_0$，点到点 ICP 可概括为：

1. 设置 $T\leftarrow T_0$。
2. 变换所有 source 点：$x_i=Tq_i$。
3. 为每个 $x_i$ 在 target 中寻找最近邻。
4. 按距离阈值、法线夹角、互为最近邻或残差规则筛除不可靠对应点。
5. 对保留对应点求增量 $\Delta T$，使变换后的 source 更接近对应 target 点。
6. 更新 $T\leftarrow\Delta T T$。
7. 检查停止条件；若未停止，回到第 2 步。

使用第 4 节的固定对应点求解器，点到点 ICP 的核心循环可写成以下伪代码。最近邻函数只表示查询过程，具体实现通常使用 KD-tree 等空间索引：

~~~python
T = T_init
for iteration in range(max_iterations):
    source_current = transform_points(T, source)
    target_ids, distances = nearest_neighbors(target, source_current)
    keep = distances < max_correspondence_distance

    delta_T = rigid_transform_svd(
        source_current[keep],
        target[target_ids[keep]],
    )
    T = delta_T @ T

    if converged(delta_T, previous_error):
        break
~~~

伪代码中的 `delta_T` 把当前 source 点映射到匹配的 target 点，所以它从左侧乘到已有变换 `T` 上。若改成直接对原始 source 点求完整变换，则更新形式相应不同。

常见停止条件包括：

- 连续两轮 RMSE 的相对变化低于阈值；
- 平移增量和旋转增量低于阈值；
- 达到最大迭代次数；
- 有效对应点数量不足。

ICP 的误差通常会逐步下降，但这只说明算法在优化当前几何目标。它不表示已找到全局最优位姿，也不表示对象身份或 BOP 位姿一定正确。Besl–McKay 论文讨论了 ICP 的单调收敛到局部极小值性质；“局部”是应用时需要保留的边界。

## 7. 初始化为什么重要

最近邻关系会随位姿改变，因此目标函数不是一个处处平滑的单一二次函数。位姿初值较差时，最近邻可能落在错误的面、边或相邻物体上，优化就可能收敛到错误位置。

ICP 常与其他方法组成两阶段流程：

$$
\text{全局粗位姿}
\longrightarrow
\text{ICP 局部细化}.
$$

粗位姿可以来自：

- PPF 等 CAD 几何匹配；
- FPFH 等局部特征配合 RANSAC；
- RGB 的 2D–3D 对应关系与 PnP；
- 已训练位姿网络或 FoundationPose 等方法；
- 机器人已知的抓取/装配先验。

初始值还应与 ICP 的对应距离阈值匹配：阈值过小会使正确对应被排除、甚至没有对应点；阈值过大则会引入更多错误最近邻。Open3D 的 [ICP 教程](https://www.open3d.org/docs/latest/tutorial/pipelines/icp_registration.html)把粗位姿和初始变换作为 ICP 输入；[全局配准教程](https://www.open3d.org/docs/latest/tutorial/pipelines/global_registration.html)介绍了基于特征与 RANSAC 的初始对齐流程。

## 8. 常见 ICP 变体

| 变体 | 对应残差/策略 | 主要作用与约束 |
|---|---|---|
| 点到点 ICP | 三维欧氏点距离 | 不需要法线；适合基础配准 |
| 点到平面 ICP | target 法线方向距离 | 局部收敛通常更快；依赖法线质量 |
| Trimmed ICP | 保留误差较小的一部分对应点 | 适合两点云重叠比例未知的情况 |
| 鲁棒 ICP | 对残差使用 Huber、Cauchy、Tukey 等鲁棒损失 | 降低少量大残差/离群点的影响 |
| G-ICP | 用局部协方差建模两组点的邻域几何 | 统一点到点与点到平面的局部统计表达 |
| 多尺度 ICP | 从粗体素到细体素逐级优化 | 先扩大捕获范围，再提高局部精度 |

[Open3D 鲁棒核教程](https://www.open3d.org/docs/latest/tutorial/t_pipelines/t_robust_kernel.html)展示了将点到平面 ICP 与鲁棒损失结合的方式。[Generalized-ICP 论文](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf)以概率框架整合点到点和点到平面形式。[Rusinkiewicz 与 Levoy（2001）](https://graphics.stanford.edu/papers/fasticp/)系统讨论了采样、匹配、拒绝和最小化策略对速度与收敛的影响。

## 9. 典型失败原因与诊断

| 现象 | 常见原因 | 可检查项 |
|---|---|---|
| 没有对应点或 fitness 接近 0 | 初值离目标较远；距离阈值小于当前偏差 | 初值坐标方向、单位、距离阈值 |
| RMSE 下降但叠加明显错位 | 错误表面之间形成低残差局部极小值；重复结构或对称性 | 可视化对应关系、引入全局初值、检查对象对称性 |
| 位姿在平面方向漂移 | 可见表面近似平面，切向约束弱 | 检查几何分布、引入边缘/颜色/多视角信息 |
| 遮挡越强，模型越偏 | 完整模型点与部分观测点的对应比例不平衡 | 选择正确的 source/target 方向、裁剪模型可见部分、使用 trimmed/鲁棒策略 |
| 结果对阈值非常敏感 | 深度噪声、单位不一致或异常点较多 | 检查深度比例、CAD 尺寸、点云单位和掩码边缘 |
| 不同帧的位姿跳变 | 单帧 ICP 初值不稳定或对称姿态切换 | 用上一帧跟踪结果初始化、加入时序约束 |

ICP 的最近邻关系通常是非互惠的：一个 source 点找到一个 target 点，不代表该 target 点反过来会选中原来的 source 点。可通过互为最近邻、距离门限、法线一致性、颜色/特征约束或鲁棒损失减少错误匹配。具体筛选策略会改变被优化的目标，需要和适用场景对应。

## 10. 与当前 BOP/YCB-V Notebook 的对应

阶段三 Notebook：[PPF 粗位姿估计与 ICP 位姿细化](./03-PPF%20粗位姿估计与%20ICP%20位姿细化.ipynb)。

其中数据和坐标关系如下：

| Notebook 变量/数据 | 含义 |
|---|---|
| **observed_cloud_c** | 从深度图和 **mask_visib** 得到的 source 点云，坐标系为相机 C |
| **cad_cloud_o** | 从 CAD 表面采样的 target 点云，坐标系为物体 O |
| **T_gt** | BOP 真值 $T_{CO}$，将 CAD 从 O 变换到 C |
| **T_init** | 对 **T_gt** 加入已知小扰动后的初始物体到相机变换 |
| **T_c_to_o_init** | $T_{init}^{-1}$，ICP 所需的 source C 到 target O 初值 |
| **icp_result.transformation** | Open3D 返回的 source-to-target 变换，即相机 C 到物体 O |
| **T_est** | 对 ICP 输出求逆后得到的物体到相机位姿估计 |

因此代码先计算：

~~~python
T_c_to_o_init = np.linalg.inv(T_init)
~~~

随后将 **observed_cloud_c** 作为 source、**cad_cloud_o** 作为 target 运行 ICP，并将返回的变换求逆：

~~~python
T_est = np.linalg.inv(icp_result.transformation)
~~~

该 Notebook 使用 BOP 真值掩码，并由真值位姿构造初始位姿。它用于研究 ICP 局部优化、参数和初值的关系；不用于报告 BOP 测试集上的完整检测或位姿估计性能。

## 11. 如何解读输出

### transformation

Open3D 返回的矩阵方向是 **source → target**。解释 4×4 矩阵前，应先确认 source 与 target 各自处于哪个坐标系。矩阵方向相反时应求逆；不能仅因变量名中出现“pose”就假设方向。

### fitness

在 Open3D legacy **o3d.pipelines.registration** 实现中，fitness 是有效对应点数除以 source 点数。它反映 source 点中有多少落入最大对应距离阈值，不是位姿误差，也不等于 BOP AR。[Open3D legacy 实现](https://github.com/isl-org/Open3D/blob/main/cpp/open3d/pipelines/registration/Registration.cpp)可查看该计算。不同接口或文档版本可能采用不同说明，使用时应核对当前 API 的定义。

对于当前 Notebook，source 是实例可见深度点云。fitness 偏高表示较多观测点能在阈值内找到 CAD 邻点；它不证明 CAD 的所有表面都已覆盖。

### inlier_rmse

这是有效对应点距离的均方根值，单位与点云坐标相同。低 RMSE 只说明当前被接受的点对应残差较小；如果对应关系错误、只覆盖很小区域或大量点被阈值排除，RMSE 仍可能偏低。应同时查看对应点数量、fitness、可视化和外部位姿误差。

### 位姿误差

阶段三 Notebook 用真值计算旋转角误差和平移误差，用于比较受控扰动前后的偏差。BOP 评测按其定义的误差函数处理可见性和对称性，详见 [BOP 任务说明](https://bop.felk.cvut.cz/tasks/)；Notebook 中的两个误差不替代官方 BOP 指标。

## 12. 建议的学习实验

1. 使用默认扰动运行一次 ICP，记录初始位姿和输出位姿的旋转、平移误差。
2. 增大旋转或平移扰动，记录 ICP 是否仍能降低位姿误差。
3. 固定初始位姿，分别调整最大对应距离和体素尺寸，记录 **fitness**、**inlier_rmse** 和几何叠加。
4. 将距离阈值调得过小，观察有效对应点数量变化；将阈值调得过大，观察错误对应对结果的影响。
5. 将点到点估计替换为点到平面估计：先为 target 点云估计法线，再比较收敛速度和结果。结果依赖法线质量与几何形状。
6. 将 PPF 或特征匹配/RANSAC 产生的粗位姿作为 ICP 初值，形成“粗配准 + 局部细化”的流程。

每次实验只改变一个因素，并记录数据样本、初值、阈值、体素尺寸、迭代次数和结果。这样可以区分算法变化与参数变化造成的影响。

## 13. 参考资料

1. Besl, P. J. and McKay, N. D. [A Method for Registration of 3-D Shapes](https://doi.org/10.1109/34.121791), IEEE TPAMI, 1992.
2. Open3D. [ICP Registration Tutorial](https://www.open3d.org/docs/latest/tutorial/pipelines/icp_registration.html).
3. Chen, Y. and Medioni, G. [Object Modelling by Registration of Multiple Range Images](https://www.users.cs.utah.edu/~tch/publications/pub176.pdf), 1992.
4. Rusinkiewicz, S. and Levoy, M. [Efficient Variants of the ICP Algorithm](https://graphics.stanford.edu/papers/fasticp/), 3DIM, 2001.
5. Segal, A., Haehnel, D. and Thrun, S. [Generalized-ICP](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf), RSS, 2009.
6. Open3D. [Robust Kernels for Point-to-Plane ICP](https://www.open3d.org/docs/latest/tutorial/t_pipelines/t_robust_kernel.html).
