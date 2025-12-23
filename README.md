# MobilityGeneration (NYC) — OD → 公共交通出行链（SUMO）

这个仓库整理为“最小可运行”版本：给定 OD（fromEdge/toEdge + depart），在 NYC SUMO 路网上自动规划公共交通（walk + public transport + walk），输出**显式出行链**并做可用性验证。

## 你能得到什么

- NYC 路网：`sumo_network/newyork.net.xml`（**不随仓库上传**，见下文说明）
- 公交/地铁站点（已映射到路网）：`sumo_network/bus_stops_mapped.add.xml`、`sumo_network/subway_stops_mapped.add.xml`
- 公交/地铁 PT 车辆与停靠（可用于 intermodal routing）：`sumo_network/bus_routes_all_complete.rou.xml`、`sumo_network/subway_routes_complete.rou.xml`
- OD → 出行链生成与验证工具：`tools/od_plan_and_validate.py` 等
- OD/出行链样例：`examples/od_planner/sample_nyc_bus/od.json`、`examples/od_planner/sample_nyc_bus/explicit_chains.json`

不需要的历史脚本、日志、以及中间产物已移动到 `backup/`，并在 `.gitignore` 中排除（不会被 git 追踪）。

## 大文件与 Git LFS

该仓库包含 `.net.xml`、时刻表 JSON、PT `.rou.xml` 等大文件。请使用 Git LFS：

1) 安装并初始化：
- `git lfs install`

2) 跟踪大文件模式（本仓库已提供 `.gitattributes`；等价于运行 `git lfs track "<pattern>"` 并提交 `.gitattributes`）
- 参考 Git LFS 文档：`git lfs track "<pattern>"`，并 `git add .gitattributes` 后提交。

3) 如果这些大文件在你本地已经存在于普通 git 中，需要重新归一化以写入 LFS（Git LFS FAQ 建议对已存在文件执行 renormalize 后再提交）：
- `git add --renormalize .`

> 注意：GitHub LFS 有配额限制；如果你账号配额不足，建议把超大原始数据放到 Release/外部存储，只在仓库里保留脚本与下载说明。

### 关于 `sumo_network/newyork.net.xml`

该文件约 4.6GB，超过 GitHub LFS 单文件 2GB 限制，因此**不会上传到 GitHub**（已在 `.gitignore` 排除）。

使用本仓库前，请自行把 `newyork.net.xml` 放到 `sumo_network/` 目录（文件名必须一致），或改 `tools/pt_scenarios.json` / 命令行参数指向你的网络文件路径。

## 环境依赖

- 已安装 SUMO（需要 `sumo`、`duarouter`，以及 Python 的 `traci`/`sumolib`）
- Python 3.9+（建议）

## 全流程（从站点映射到出行链）

### 1)（可选）重新做“站点→路网”映射（强制 bus 可通行 + 方向一致）

本仓库当前的 `sumo_network/bus_stops_mapped.add.xml` 已是映射结果；如需重做：

```bash
python3 map_stops_to_network_optimized.py \
  -s sumo_network/bus_stops_mapped.add.xml \
  -n sumo_network/newyork.net.xml \
  -o sumo_network/bus_stops_mapped.add.xml \
  --require-vclass bus \
  --timetable Data/timetables/merged_bus_timetable.json \
  --stop-prefix bus_stop_ \
  --max-radius 4000
```

要点：
- `--require-vclass bus`：只把 busStop 放到允许 `bus` 的 lane 上（解决 “vehicle is not allowed on destination edge” 的常见根因）
- `--timetable ...`：利用站序推断方向，在候选 lane 里优先选与“指向下一站”一致的 lane（减少 “busStop not downstream”）

### 2) 生成/更新公交 PT 路由文件（路线 edges 填充 + 剔除无效路线）

```bash
python3 tools/build_nyc_bus_routes.py \
  --depart-begin 25200 --depart-end 32400 \
  --max-vehicles-per-line 10 \
  --max-vehicles-total 8000 \
  --out sumo_network/bus_routes_all_complete.rou.xml \
  --tmp sumo_output/bus_routes_all_tmp.rou.xml \
  --routing-threads 1
```

说明：
- `--depart-begin/--depart-end` 用于只生成一个时间窗内的班次（例如 7:00–9:00），避免全日数据过大。
- 内部会调用 `duarouter` 为每条 PT route 填充 `edges`，并剔除不可路由/不下游的路线，保证 SUMO/TraCI 读取稳定。

### 3) OD → 显式出行链（规划） + 验证

输入 OD JSON（数组）格式：

```json
[
  {"personId": "p0", "depart": 28800, "fromEdge": "-221444645#0", "toEdge": "-5681291#4"}
]
```

运行规划与验证（生成 `explicit_chains.json` + `validation_report.json`）：

```bash
python3 tools/od_plan_and_validate.py \
  --scenario nyc_bus_all \
  --od examples/od_planner/sample_nyc_bus/od.json \
  --output-dir out/nyc_sample \
  --pt-window-before 3600 --pt-window-after 3600 \
  --walk-factor 0.2
```

如果要同时考虑地铁+公交，使用 `--scenario nyc`（见 `tools/pt_scenarios.json`）。

输出：
- `out/nyc_sample/explicit_chains.json`：显式链（walk/ride/walk/…）
- `out/nyc_sample/validation_report.json`：验证报告（检查站点存在、ride 是否能在 PT 车辆停靠序列中实现）

`walk-factor` 说明：
- TraCI `findIntermodalRoute` 中的 `walkFactor` 是“步行速度乘子”，越小越“慢”，更倾向选择公交/地铁（而不是全程 walk）。

## 直接运行一次仿真（使用已合成的出行链）

本仓库提供了一个可直接用于 SUMO 的样例 person routes：
- `examples/od_planner/sample_nyc_bus/persons.rou.xml`

运行：

```bash
sumo -c sumo_config/nyc_sample_bus_chain.sumocfg
```

如果你修改了 `examples/od_planner/sample_nyc_bus/explicit_chains.json`，可用下面命令重新生成 `persons.rou.xml`：

```bash
python3 tools/person_chain_to_sumo.py \
  --plan examples/od_planner/sample_nyc_bus/plan.json \
  --output examples/od_planner/sample_nyc_bus/persons.rou.xml \
  --strict-continuity
```

## 目录说明（保留/备份）

- 运行必需：`Data/timetables/`、`sumo_network/`、`sumo_output/*_stops_route_mapping.csv`、`tools/`、`examples/od_planner/`
- 历史与中间产物：`backup/`（已在 `.gitignore` 排除，默认不进 git）
