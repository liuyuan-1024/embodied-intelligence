# BOP YCB-V 数据文件说明

本文说明本项目下载的 BOP 格式 YCB-V 数据中，场景、图像、实例标注、CAD 模型、掩码和相机参数之间的对应关系。重点解释 `gt_id` 与 `obj_id`。

## 1. 当前数据目录

本项目的数据根目录为：

```text
.local/data/bop/ycbv/
```

当前本地目录包含 YCB-V 的测试场景子集和物体模型。具体场景号、图像号会因下载包和评测版本而不同；目录编号是数据索引，不代表物体类别。

```text
ycbv/
├── camera_cmu.json              # CMU 相机参数（用于相机模拟/渲染）
├── camera_uw.json               # UW 相机参数（用于相机模拟/渲染）
├── dataset_info.md              # YCB-V 数据集说明、模型处理和相机说明
├── models/                      # 常规纹理 CAD 模型
│   ├── models_info.json
│   ├── obj_000001.ply
│   └── obj_000001.png
├── models_fine/                  # 更高细节的 CAD 模型
│   ├── models_info.json
│   ├── obj_000001.ply
│   └── obj_000001.png
├── models_eval/                  # 为位姿误差评测准备的重采样/简化模型
│   ├── models_info.json
│   └── obj_000001.ply
└── test/
    └── 000048/                   # 一个场景（scene_id）
        ├── scene_camera.json     # 当前场景每帧的相机内参和深度比例
        ├── scene_gt.json         # 当前场景每帧的实例 obj_id 与真值位姿
        ├── scene_gt_info.json    # 每个实例的包围框、像素数和可见比例等统计
        ├── rgb/                  # 彩色图像
        ├── depth/                # 深度图
        ├── mask/                 # 物体轮廓掩码
        └── mask_visib/           # 物体可见部分掩码
```

上面的场景目录编号仅作结构示例。BOP 格式中，`mask`、`mask_visib` 等子目录可能是可选项；本地 YCB-V 测试场景包含 RGB、深度、两类掩码和三种场景 JSON 文件。

## 2. 四个编号：`scene_id`、`image_id`、`gt_id`、`obj_id`

| 名称 | 含义 | 从哪里读取 | 示例用途 |
|---|---|---|---|
| `scene_id` | 场景/序列编号 | 场景文件夹名，例如 `test/000048/` | 定位该场景下的全部帧和标注 |
| `image_id` | 场景内某一帧图像的编号 | RGB、深度文件名，以及 JSON 的键 | `000001.png` 与该场景 JSON 中图像键 `1` 指向同一帧 |
| `gt_id` | 当前图像的 `scene_gt.json` 列表中，某条实例标注的从零开始的序号 | **不是独立 JSON 字段**，由列表位置确定 | 对应掩码文件名中的第二个编号 |
| `obj_id` | 物体模型编号 | 当前图像的 `scene_gt.json` 实例记录中的字段 | `obj_id: 6` 对应 `models/obj_000006.ply` |

### `gt_id` 和 `obj_id` 的对应方式

以某个图像键 `1` 为例，`scene_gt.json` 中的值是一个列表。列表里每个元素描述该帧中的一个物体实例及其真值位姿：

```json
{
  "1": [
    {"obj_id": 2,  "cam_R_m2c": [...], "cam_t_m2c": [...]},
    {"obj_id": 10, "cam_R_m2c": [...], "cam_t_m2c": [...]},
    {"obj_id": 6,  "cam_R_m2c": [...], "cam_t_m2c": [...]}
  ]
}
```

该示例中：

- 第一条记录的 `gt_id = 0`，`obj_id = 2`，模型文件为 `models/obj_000002.ply`。
- 第二条记录的 `gt_id = 1`，`obj_id = 10`，模型文件为 `models/obj_000010.ply`。
- 第三条记录的 `gt_id = 2`，`obj_id = 6`，模型文件为 `models/obj_000006.ply`。

`gt_id` 是列表下标，所以从 `0` 开始，并且在每张图像的列表中重新编号。它不是场景全局编号，也不是物体类别/模型编号。`obj_id` 则标识使用哪一个 CAD 模型。

如果同一张图里出现两个相同型号的物体，它们会有相同的 `obj_id`，但占据列表中的不同位置，因此有不同的 `gt_id`；两条记录也各自有独立的位姿。

掩码文件按 `图像编号_gt编号.png` 命名。例如帧 `000001` 中，列表第 2 条记录（`gt_id = 1`）对应的可见掩码路径为：

```text
test/000048/mask_visib/000001_000001.png
```

此处两个相同格式的 `000001` 含义不同：第一个是 `image_id`，第二个是补齐到六位的 `gt_id`。同一图像里的掩码序号应与 `scene_gt.json` 和 `scene_gt_info.json` 的列表位置对应。

## 3. 场景目录中的文件

每个场景文件夹对应一个 `scene_id`，里面的 JSON 文件以图像编号为键。RGB、深度、相机参数、位姿和实例信息要通过同一个 `image_id` 对齐。

### `rgb/`

彩色相机图像。`rgb/000001.png` 是一帧图像；BOP 的图像格式也可能使用其他扩展名。

### `depth/`

深度图通常是 16 位无符号整数图像，像素保存原始深度值。要转换为毫米，使用同一帧 `scene_camera.json` 中的 `depth_scale`：

$$
z_{\mathrm{mm}} = z_{\mathrm{raw}} \times \mathrm{depth\_scale}.
$$

不能只凭像素数值就假定原始值已经是毫米。

### `scene_camera.json`

每个 `image_id` 对应一组相机信息，常用字段为：

- `cam_K`：当前图像的 $3\times3$ 相机内参矩阵，按行存储。
- `depth_scale`：原始深度转换到毫米时使用的乘数。
- `cam_R_w2c`、`cam_t_w2c`：可选的世界坐标系到相机坐标系的变换。

图像投影和深度反投影应优先使用该帧的 `cam_K`。根目录中的 `camera_uw.json` 和 `camera_cmu.json` 是相机配置文件，主要用于传感器模拟/渲染；`dataset_info.md` 说明了 YCB-V 场景与这两类相机参数的关系。

### `scene_gt.json`

每个图像键对应一个实例记录列表。列表中的每条记录包含：

- `obj_id`：该实例使用的物体模型 ID。
- `cam_R_m2c`：从物体模型坐标系到相机坐标系的旋转矩阵。
- `cam_t_m2c`：从物体模型坐标系到相机坐标系的平移向量，单位为毫米。

对模型坐标系中的点 $\mathbf{p}_m$，真值变换为：

$$
\mathbf{p}_c = \mathbf{R}_{m2c}\mathbf{p}_m + \mathbf{t}_{m2c}.
$$

`scene_gt.json` 的数组位置就是 `gt_id`。它本身通常不作为一项写在对象记录里。

### `scene_gt_info.json`

该文件的图像键和列表顺序与 `scene_gt.json` 对齐。列表中的第 `gt_id` 项描述相同实例的投影/可见性统计，例如：

- `bbox_obj`：物体轮廓的 2D 包围框，格式为 `[x, y, width, height]`。
- `bbox_visib`：可见物体部分的 2D 包围框。
- `px_count_all`：物体轮廓总像素数。
- `px_count_valid`：物体轮廓中具有有效非零深度的像素数。
- `px_count_visib`：物体可见部分的像素数。
- `visib_fract`：可见比例，按 `px_count_visib / px_count_all` 计算。

`scene_gt_info.json` 给出的是标注统计信息，不是位姿算法运行时检测出来的结果。

### `mask/` 与 `mask_visib/`

- `mask/<image_id>_<gt_id>.png`：对应实例的物体轮廓掩码。
- `mask_visib/<image_id>_<gt_id>.png`：对应实例在当前图像中实际可见区域的掩码。

掩码文件名中的 `gt_id` 用来在同一帧有多个实例时区分它们。掩码像素可用于选出该实例对应的 RGB 或深度区域；阶段二使用 `mask_visib` 保留可见表面的深度点。

## 4. CAD 模型目录

### `models/`

常规模型目录，是阶段一和阶段二读取 CAD 几何的默认来源。`models_info.json` 中记录每个 `obj_id` 对应模型的三维包围盒和直径等元信息。

YCB-V 的模型和位姿平移单位为毫米。BOP 格式将模型坐标系原点放在模型三维包围盒中心；该数据集对原始 YCB 模型做了相应转换，并同步转换了真值位姿。

### `obj_XXXXXX.ply` 和同名 `.png`

- `.ply` 保存三角网格几何，包括顶点、面、法线和纹理坐标等。
- 同名 `.png` 在 PLY 引用纹理图像时，保存模型表面纹理（texture atlas）。例如 `obj_000006.ply` 的头部声明了 `TextureFile obj_000006.png`。

该 PNG 是 CAD 表面纹理，不是场景 RGB 图像，也不是该物体的分割掩码。只读取点坐标或做几何投影时，通常只需要 PLY；需要显示纹理外观时才需要 PNG。

### `models_fine/`

包含更高细节版本的模型。本地 `dataset_info.md` 说明该目录基于原始数据中的 `textured.obj`；例如 `obj_000006.ply` 有约 26 万个顶点，而 `models/obj_000006.ply` 有约 8 千个顶点。初学时使用 `models/` 更省内存，使用 `models_fine/` 可在需要更精细几何或纹理时切换。

### `models_eval/`

包含为位姿误差计算而重采样和简化的评测模型，通常不作为阶段一、阶段二的默认可视化模型。它的作用是让评测中对模型表面进行距离/误差计算时使用统一的采样表示。

### `models_info.json`

每个模型文件夹都有一份该文件，按 `obj_id` 存储模型元信息，例如三维包围盒范围、各轴尺寸和模型直径。它不存图像中的实例列表，也不提供某一帧的位姿。

## 5. 根目录的其他文件

- `dataset_info.md`：YCB-V 的数据来源、模型转换、数据划分和相机说明。本地副本提到 `models` 基于 `textured_simple.obj`，`models_fine` 基于 `textured.obj`。
- `camera_uw.json`、`camera_cmu.json`：数据集相机配置。实际某帧的内参从该帧的 `scene_camera.json` 读取。
- `test_targets_bop19.json`、`test_targets_bop24.json`：若对应版本下载包包含，则列出相应 BOP 评测使用的测试目标；它们是评测目标清单，不替代 `scene_gt.json` 中的逐实例位姿标注。当前本地目录若没有这些文件，不影响读取场景本身的 JSON 标注。

## 6. 阶段一和阶段二如何使用这些文件

### 阶段一：用真值位姿投影 CAD

1. 从 `test/<scene_id>/scene_gt.json` 选择一个 `image_id` 和其中一个列表位置，即 `gt_id`。
2. 从这条记录读取 `obj_id`，打开 `models/obj_<obj_id 补齐为六位>.ply`。
3. 从同一条记录读取 `cam_R_m2c`、`cam_t_m2c`，把 CAD 点从物体坐标系变换到相机坐标系。
4. 从 `scene_camera.json` 同一 `image_id` 读取 `cam_K`，把相机坐标点投影到 RGB 图像。
5. 将投影与原图比较，核对真值几何链路。

### 阶段二：把可见深度与 CAD 放到同一坐标系

1. 读取同一 `image_id` 的 `depth` 图和 `scene_camera.json` 中的 `cam_K`、`depth_scale`。
2. 用该帧对应的 `mask_visib/<image_id>_<gt_id>.png` 筛出所选实例可见区域的深度像素。
3. 将深度像素反投影到相机坐标系。
4. 将相同实例的 CAD 点用 `scene_gt.json` 中的真值位姿变换到相机坐标系。
5. 对比可见深度点与完整 CAD 点云。

两个阶段都使用真值标注，目的分别是核对几何投影链路和比较点云；它们没有运行物体检测、位姿估计或模型训练。

## 7. 常见混淆

| 容易混淆的概念 | 正确区分 |
|---|---|
| `gt_id` 和 `obj_id` | `gt_id` 是当前图像实例列表的序号；`obj_id` 是 CAD 模型编号。 |
| `gt_id` 和“真值位姿” | `gt_id` 只用于找到实例记录；位姿数值在同一条记录的 `cam_R_m2c`、`cam_t_m2c` 中。 |
| 场景编号和图像编号 | `scene_id` 是文件夹；`image_id` 是该场景中的帧。相同 `image_id` 可在不同场景出现。 |
| 一条实例记录和一种模型 | 一条记录描述一个图像实例；多个实例可以使用同一个 `obj_id`。 |
| `models/obj_000006.png` 和场景图像 | 前者是 CAD 纹理图；场景彩色图在 `test/<scene_id>/rgb/`。 |
| `mask` 和 `mask_visib` | 前者表示物体轮廓；后者表示被遮挡后仍可见的部分。 |
| `visib_fract` 和深度有效比例 | `visib_fract` 衡量可见轮廓占物体总轮廓的比例；它不等同于深度有效像素比例。 |
| `models/` 和 `models_eval/` | `models/` 是常规 CAD 几何；`models_eval/` 用于标准化的位姿误差计算。 |

## 8. 官方格式参考

- [BOP Toolkit：BOP 数据集格式](https://github.com/thodan/bop_toolkit/blob/master/docs/bop_datasets_format.md)：场景目录、图像编号、掩码命名、相机参数、真值位姿、可见性统计、模型和坐标单位。
- [BOP 官方数据集页面](https://bop.felk.cvut.cz/datasets/)：BOP 数据集划分、下载说明和基准任务。
- 本地数据集说明：[dataset_info.md](../.local/data/bop/ycbv/dataset_info.md)。
