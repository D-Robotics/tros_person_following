# 目标 Lost 后处理分析与重新锁定流程

本文档分析 `tros_person_following`（`PersonFollowingNode`）在跟踪目标 lost 后的处理链路，找出拖慢重新锁定目标的瓶颈，并提出改进措施供逐项讨论。

源码：`src/tros_person_following/src/person_following_node.cpp`、`include/tros_person_following/person_following_node.h`、`launch/tros_person_following.launch.py`。

> **行号说明**：文档内的 `cpp:NNN` 行号引用基于历史快照，后续重构（删除 `tryPredictAndNavigate`/`ComputePathClient`/`speed_limit`/`replay_mode` 等、参数重命名）可能导致行号偏移。定位代码时**以函数名或 `[LOST-FLOW N]` 日志关键词 grep 为准**，行号仅作辅助参考。

---

## 1. 当前 Lost 后的处理链路（已实现改进 B + M + 超时/推进修复）

### 术语说明

| 术语 | 含义 |
|------|------|
| **LKP**（Last Known Position） | 目标**最后已知位置**——`getLastKnownPose` 从 `target_pos_history_[id]` 取的最新样本的 `world_pos`（map 系）。即目标丢失前最后一次被相机检测到时的位置，是目标"最后在哪"的线索。 |
| **历史轨迹** | `target_pos_history_[id]` 整个采样序列（每帧检测到目标时追加一个 `world_pos`+`stamp`，按 `max_age_sec` 清理，保留约最近 10s）。LKP 是其中的最新点；"回溯历史"指往更早样本找。 |
| **预测点** | `predictTargetPose` 用 LKP + 速度外推的**目标未来位置**（朝目标正在去的方向）。 |
| **观测点** | `pickLkpObservationPose` 选的**机器人该站到的位置**（LKP 附近 free 点，或历史回溯点），去那里扫描看目标。区别：预测点=目标要去哪，观测点=机器人去哪看。 |
| **belief search** | LOST 后以 LKP 为中心的主动搜索（nav 到观测点→原地扫描→迭代）。 |
| **FLOW N** | 代码里 `[LOST-FLOW N]` INFO 日志编号，对应流程图节点，便于 grep 定位。 |

状态机 `TrackState`：IDLE / TRACKING / LOST。简化概览（不涉及细节）：

```
TRACKING（正常跟随）
  └─ 连续丢失超时 → LOST
      ├─ 预测点优先（默认）：导航到目标预测位置 → 到点扫描
      │   └─ 预测不可用(无样本/越界/goal点cell不free) → belief 搜索 fallback
      │       └─ 导航到 LKP 附近观测点 → 原地扫描（最多 2 轮，每轮去不同点）
      ├─ 期间每帧：找回原目标 / 接受新目标 → 恢复 TRACKING
      └─ 搜索结束或超时 → IDLE（原地旋转搜索）→ 锁定新目标 → TRACKING
```

完整流程（行号对应 `src/person_following_node.cpp`，2026-07-23 状态）：

```
TRACKING 中连续未检测到目标（!found）  (status: TRACKING)
  └─ 首帧丢失 [FLOW 1]：
      ├─ 无在飞 nav goal（!nav_goal_handle_）→ 宽限期无价值（无旧 nav 可渡遮挡），直接进 LOST [FLOW 2]  (status: LOST)
      └─ 有在飞 nav goal → target_lost_=true，记 tp_target_lost_，进入宽限期（旧 nav 继续执行渡过遮挡）  (status: WILL_BE_LOST)
          └─ 持续 > tracking_to_lost_timeout_sec_（运行时 2.0s）未找回 → 进 LOST [FLOW 2]  (status: LOST)
          └─ 宽限期内找回 → 回 TRACKING（target_lost_=false）  (status: TRACKING)
          ├─ cancel 活跃 nav goal [FLOW 3]  (status: LOST，不变)
          ├─ startBeliefSearch()  ← ★ LOST 唯一搜索入口（cpp:523）  (status: LOST，不变)
              │   内部三段：无历史 → 预测点优先 → 观测点 fallback
              │
              ├─【段1 取 LKP】getLastKnownPose → LKP + (vx,vy,speed)  (status: LOST，不变)
              │   └─ 无运动历史(样本<2) → staying put，等 LOST 超时回 IDLE [FLOW 4] → return  (status: LOST_HOLDING)
              ├─【段2 越界更新 LKP】LKP 越界时回溯历史找地图内样本更新 LKP（+重算运动方向）；LKP 在地图内则跳过 [FLOW 6 rewinding]  (status: LOST_REWIND[越界更新时] 或 LOST[在地图内，不变])
              ├─【段3 选首个观测点，预测点优先】  (status: 沿用段2结果，不变)
              │   ├─ predictTargetPose → 预测点  (status: 不变)
              │   │   ├─ 预测成功 + isGoalReachable(goal点costmap cell free) → 用【预测点】作首个观测点（默认路径）
              │   │   │   nav + searched_points 记录 [FLOW 6 predicting]  (status: LOST_PREDICTING)
              │   │   └─ 预测不可用(空/越界/cell不free) ↓ fallback  (status: 不变)
              │   └─ pickLkpObservationPose → 选观测点（belief search fallback）  (status: 不变)
              │       ├─ 沿运动方向/径向在 LKP 附近选 free 点 → 【LKP 附近观测点】
              │       └─ LKP 附近无 free → 回溯历史轨迹找 free 点（两级：样本点本身 free 直接用；非 free 则 BFS 在样本附近 2m 找 free 点）→ 【历史回退观测点】
              │       └─ 都无 free 观测点 → staying put，等 LOST 超时回 IDLE [FLOW 5] → return  (status: LOST_HOLDING)
              │   → 选到观测点：belief_search_.active=true，nav 到观测点  (status: LOST_PREDICTING[预测点] / LOST_BELIEF_NAV[LKP附近] / LOST_BELIEF_NAV_HISTORY[历史回退])
              │   → 进 continueBeliefSearch 扫描/迭代框架  (status: 不变，沿用上一个)
              │
              （段1/段3 FLOW5 staying put 后，等 LOST 超时回 IDLE，期间 status 保持 LOST_HOLDING）

LOST 期间（belief_search_.active==true 之后）：
  ├─ 每帧（detectResultCallback LOST 分支）：  (status: 沿用当前搜索状态，不变)
  │   ├─ 找回原 id → 距离门控（d(target,LKP) ≤ currentRelockDist()，分段：见下）通过 → 恢复 TRACKING + 清理 belief_search_ [FLOW 7]  (status: TRACKING)
  │   ├─ 接受 valid+moving 新目标 → 距离门控（同上）通过 → 切新目标 + 清理 belief_search_ [FLOW 8]  (status: TRACKING)
  │   └─ 都没找到 →  (status: 不变)
  │       ├─ scan_completed==true → continueBeliefSearch 推进下一轮 [FLOW 11]  (status: LOST_BELIEF_NAV)
  │       └─ 否则等 nav 到达  (status: 不变)
  │
  └─ nav 到达（resultCallback：SUCCEEDED/ABORTED 都推进）：  (status: 不变)
      ├─ belief_search_.active → continueBeliefSearch()  ← 预测点优先 & belief search 共用  (status: 不变)
      │   ├─ 超时 belief_search_timeout_sec_ → 结束 [FLOW 9]  (status: LOST，回切)
      │   ├─ !scanning && !scan_completed → SpinInPlaceWCmdVel 定向扫描（朝 LKP）[FLOW 10]  (status: LOST_BELIEF_SCAN)
      │   │   └─ 扫描线程完成 → scan_completed=true（下一帧轮询推进）  (status: LOST_BELIEF_SCAN，不变)
      │   ├─ scanning 中 → return  (status: LOST_BELIEF_SCAN，不变)
      │   └─ scan_completed → round++（最多 max_rounds=2）  (status: 不变)
      │       ├─ round < max → pickLkpObservationPose(排除已搜点) → nav + 记录 [FLOW 11]  (status: LOST_BELIEF_NAV[LKP附近] 或 LOST_BELIEF_NAV_HISTORY[历史回退])
      │       └─ round >= max / 无更多观测点 → 结束 [FLOW 12]  (status: LOST，回切)
      └─ !belief_search_.active → 不做事（nav 到达但搜索未启动[段1/段3 staying put]或已结束，多为残留 result），等 LOST 超时回 IDLE  (status: 不变)

LOST→IDLE 超时（`cpp:458`，条件 `!belief_search_.active && (now-tp_lost_)>lost_to_idle_timeout_sec_`，两个 AND）：
  ├─ 分支A：belief_search_.active==true（搜索在跑：nav 到观测点 / 扫描中 / round 迭代）
  │   → LOST 超时【挂起】，不计 1.0s；搜索跑完(active→false)后下一帧立即触发分支B
  │   (status: LOST 子状态，不变，直到搜索结束)
  └─ 分支B：belief_search_.active==false（搜索没启动[段1/段3 staying put] 或 已结束[FLOW 9/12/7/8]）
      且 进 LOST 满 lost_to_idle_timeout_sec_（运行时 1.0s）
      → 回 IDLE [FLOW 13]  (status: IDLE_SEARCHING)
      ├─ cancel nav + 清理 belief_search_  (status: IDLE_SEARCHING，已发)
      ├─ 重置 tp_target_find_start_、observe_track_id_  (status: IDLE_SEARCHING，不变)
      └─ IDLE 搜索：  (status: IDLE_SEARCHING 或 IDLE_OBSERVING)
          ├─ valid_persons 空 → 异步 spin 搜索（continue_search）  (status: IDLE_SEARCHING)
          ├─ 有 valid 但无 moving → 观察期 IDLE_OBSERVING（idle_observe_duration_sec_）  (status: IDLE_OBSERVING)
          └─ 有 moving → 选最近 → 进 TRACKING  (status: TRACKING)
```

### LOST 期重锁距离门控（relock_dist timeline，2026-07-27）

`detectResultCallback` LOST 分支每帧跑 FLOW 7（找回原 id）/ FLOW 8（接受新 moving 目标）。两者都受**距离门控**约束：新出现目标离 LKP 超过当前阈值就拒绝（避免锁错无关路人）。阈值随 **LOST 持续时间分段放宽**（早期严格、晚期宽松），由 `currentRelockDist()`（`cpp:1893`）返回：

```
LOST 持续时间 t = (now - tp_lost_).seconds()   (tp_lost_ 在 FLOW 2 enter_lost 设置，cpp:505)
  ├─ t < relock_dist_phase1_sec_(2.0s) → thr = relock_dist_phase1_(1.5m)   严格：只接受 LKP 1.5m 内（几乎只能是原目标）
  ├─ t < relock_dist_phase2_sec_(5.0s) → thr = relock_dist_phase2_(3.0m)   中等
  └─ t ≥ 5.0s                          → thr = relock_dist_phase3_(5.0m)   宽松（原目标可能已走远）
```

**门控点**（两处，都用 `currentRelockDist()`）：
- **FLOW 7**（`cpp:632`）：原 id 重现时，`d(target, LKP) > thr` → skip（log `[LOST-FLOW 7] ... (>X.XX m, t=Y.Ys), skip`）
- **FLOW 8**（`cpp:690`）：新 moving 候选，`d > thr` → skip（DEBUG log `[LOST-FLOW 8] ... (>X.XX m, t=Y.Ys), skip`）

**LKP 来源** `getRelockLkp()`（`cpp:1868`）：belief_search active 用 `belief_search_.lkp`，否则用 `target_pos_history_` 最新样本。无 LKP 时（返回 false）两处都跳过门控（FLOW 7 不进 if、FLOW 8 `have_lkp=false`）。

**语义**：A1+A3——优先找回原 id（FLOW 7），找不到才接受新目标（FLOW 8），且新目标必须在 LKP 附近（距离门控）。timeline 让"附近"随时间放大：刚丢只接受 1.5m 内（强制 belief search 靠近 LKP 才锁），久丢放宽到 5m。

**参数**（5 个，cpp declare + launch，可运行时覆盖）：
| 参数 | 默认 | 含义 |
|---|---|---|
| `relock_dist_phase1` | 1.5 | t<phase1_sec 时的距离阈值（m） |
| `relock_dist_phase2` | 3.0 | phase1_sec≤t<phase2_sec 时的距离阈值（m） |
| `relock_dist_phase3` | 5.0 | t≥phase2_sec 时的距离阈值（m） |
| `relock_dist_phase1_sec` | 2.0 | phase1→phase2 切换点（s） |
| `relock_dist_phase2_sec` | 5.0 | phase2→phase3 切换点（s） |

注：旧参数 `relock_max_dist_from_lkp`（单阈值 3.0m）已于 2026-07-27 移除，由 timeline 替代。timeline 用 LOST 持续时间而非 belief_search_.round 作索引——round 推进慢（每轮 nav+扫描数秒），pending-lost 宽限期/staying put 时 round=0 但 relock 仍每帧跑，时间更贴合"早期严格、晚期宽松"语义。

### TRACKING 期边缘转向（edge-turn）子流程（2026-07-25）

`publishGoalPose`（`cpp:1026`）每帧按顺序判断，各分支互斥（命中即 return）：

```
TRACKING 期每帧（publishGoalPose）
  ├─ dist < follow_distance_min_  (cpp:1160)
  │   ├─ 有 active nav → cancel + 清指针  (status: TRACKING，不变)
  │   └─ 发零速 cmd_vel  (status: TRACKING，不变)
  │   ↓ 不 return，fall through 到 edge-turn
  │
  ├─ edge-turn 触发条件（cpp:1187，2026-07-25 收紧）：
  │     !spin_active_ && !goal_pending_ && isTargetAtFrameEdge(target)
  │       && person_map_dist < follow_distance_min_   ← 新增距离约束
  │   ├─ 命中 → cancel nav(若有) + 起分离线程 SpinInPlaceWCmdVel(turn_angle)
  │   │         turn_angle = shortest_angular_distance(robot_yaw, yaw_to_target)  (闭环角)
  │   │   → TRACKING_EDGE_TURN:id=N:dist=X.XX  (status: TRACKING_EDGE_TURN)
  │   └─ 不命中（dist≥min 或 不在边缘 或 spin/nav 在跑）→ 跳过
  │
  ├─ withhold（dist ≤ follow_distance_max_）→ TRACKING  (status: TRACKING)
  │
  └─ follow nav（dist > follow_distance_max_）  (cpp:1224)
      ├─ spin_active_ → spin_stop_requested_=true + return  ← 2026-07-25 互斥加固
      │   (spin 期间不发 follow nav，避免与 cmd_vel 角速度竞争 / 状态震荡)
      └─ getFreePersonGoal → BFS 选 free cell → asyncNavToPose  (status: TRACKING)
```

**两处 2026-07-25 改动**：
1. **edge-turn 加 `dist < follow_distance_min_` 约束**（`cpp:1188`）：远距离 + 边缘目标**不再触发 edge-turn**，直接走 follow nav（接受"目标可能走出画面"风险，换取不再震荡）。edge-turn 只在"太近 + 边缘"时触发，与 dist<min 的 cancel+零速语义配套。
2. **follow nav 分支 spin 活跃时 return**（`cpp:1228-1231`）：之前只设 `spin_stop_requested_=true` 但继续发 follow nav，导致 spin 与 Nav2 抢 `/cmd_vel`、TRACKING_EDGE_TURN ↔ TRACKING 状态震荡。改为 return，spin 退出后下一帧再判断。

**互斥关系**：edge-turn spin（cmd_vel 角速度）与 follow nav（Nav2）通过 `spin_active_` 互斥——spin 期间不发 nav，nav 期间 `!spin_active_` 才可能起 spin。


**`/tros_tracking_status` 状态标注说明**：流程图每个环节的 `(status: X)` 标注该环节通过 `setFollowStatus` 上报到 `/tros_tracking_status` 的状态。
- **`不变`**：该环节不调 `setFollowStatus`（或调了但状态相同），topic 值保持上一环节的值（`setFollowStatus` 去重，同状态不重发）。
- 状态后缀：`TRACKING:id=N:dist=X.XX`（跟踪带 id+距离）、`TRACKING_EDGE_TURN:id=N:dist=X.XX`（跟踪期边缘转向，cmd_vel spin 中）、`WILL_BE_LOST:id=N`(pending-lost 宽限期内)、`LOST:id=N`/`LOST_PREDICTING`/`LOST_BELIEF_NAV:round=N`(LKP附近)/`LOST_BELIEF_NAV_HISTORY:round=N`(历史回退)/`LOST_BELIEF_SCAN:round=N`(扫描中)/`LOST_HOLDING`(staying put)/`LOST_REWIND`(LKP越界更新)（LOST 各子状态）、`IDLE_OBSERVING:id=N`（观察期带 id）。
- 没有单独标注的环节默认 `(status: 不变)`——即沿用当前状态，topic 不更新。

### LOST 搜索框架（预测点优先 + belief search，统一入口）

**只有一个 LOST 搜索入口**：`startBeliefSearch`（`cpp:523`）。它是 LOST 后所有主动搜索的统一入口，内部按优先级选首个观测点：
1. **段3 预测点优先**（默认）：`predictTargetPose` 成功 + `isGoalReachable(goal点costmap cell free)` → 用预测点作首个观测点，进 `continueBeliefSearch` 扫描/迭代框架。`isGoalReachable` 改为 costmap cell free 检查（不再调 Nav2 ComputePathToPose），预测点落在 occupied/unknown cell 时直接 fallback。
2. **段3 belief search fallback**：预测不可用 → `pickLkpObservationPose` 选 LKP 附近观测点（含历史回退），同样进 `continueBeliefSearch` 框架。

**预测失败的兜底**：段1（无历史）/段3 FLOW5（无观测点）直接 staying put，等 LOST 超时（`lost_to_idle_timeout_sec_`）回 IDLE spin——不再调独立的预测兜底函数（原 `tryPredictAndNavigate` 已删除：它要么必然失败——段1 同样要求 ≥2 样本；要么在边缘/障碍场景预测点大概率越界/不可达、且不进扫描框架、等价 staying put；resultCallback 的 `!active` 兜底也极少触发且 near-certain 失败）。

**`continueBeliefSearch` 是预测点优先 & belief search 共用的扫描/迭代框架**：无论首个观测点是预测点还是 LKP 附近点，到点后都走 `continueBeliefSearch`（扫描→round++→选下一观测点→nav）。区别只在"首个观测点怎么选"，后续轮次统一走 `pickLkpObservationPose`（LKP 附近，排除已搜点，含预测点）。

**一句话**：`startBeliefSearch` 是唯一入口，内部"预测点优先、belief search fallback"，两者共用 `continueBeliefSearch` 扫描迭代框架；预测失败（无历史/无观测点/nav到达未激活）一律 staying put 等 LOST 超时回 IDLE spin，无独立预测兜底。


**2026-07-23 关键修复与改进**：
1. **LOST 超时挂起**（`cpp:453`）：`!belief_search_.active &&` 条件让搜索在跑（active==true）时不计 LOST 超时，跑完（active→false）才回 IDLE。
2. **nav abort 也推进**（`resultCallback` ABORTED 分支）：nav 失败时也调 `continueBeliefSearch`，避免白等 8s。
3. **`scan_completed` 推进机制**（`cpp:2016/2036/2053/652`）：加 `scan_completed` 标志区分"到达该起扫描"与"扫描完成该推进"，并在 LOST 每帧轮询推进——否则扫描完成后无 nav result 触发、卡在 round 0。
4. **预测点方案作 LOST 默认**（`startBeliefSearch` `cpp:1991-2013`）：LOST 入口先尝试预测点（朝目标未来位置，时间对齐匀速移动目标），预测不可用(无样本/越界/cell不free)时 fallback 到 belief search（LKP 附近）。两者共用 `continueBeliefSearch` 扫描/迭代框架，预测点扫一次没找到→后续轮走 LKP 附近继续扫。
5. **观测点重复 bug 修复**（`cpp:2038/2141`）：`searched_points` 记录从 `continueBeliefSearch` 开头（依赖被 `resultCallback` 清空的 `last_nav_goal_pose_`）改为**发 nav 时立即记录**——修复 round 0/1 选同一观测点、第二轮浪费的问题。
6. **LKP 越界 / 无 free 观测点 时回溯历史轨迹（两级）**（`startBeliefSearch` rewinding + `pickLkpObservationPose` 历史回退）：目标走到地图边缘外/障碍密集区丢失时，回溯 `target_pos_history_` 找 free 点作搜索中心/观测点。历史回退用两级：①样本点本身 free → 直接用作观测点；②样本非 free（目标在障碍区、costmap 常把人位置标 lethal）→ BFS 在样本附近 `costmap_free_search_radius_m_`(2m) 找最近 free 格作观测点——覆盖"目标在障碍区边缘、附近有 free"的常见情况。
7. **预测兜底全简化（方案甲）**（段1 `cpp:1915-1924` + 段3 FLOW5 `cpp:2022-2028` + `resultCallback` `!active` 分支）：无运动历史(段1)/无 free 观测点(段3)/nav到达未激活(resultCallback) 时一律 staying put 等 LOST 超时回 IDLE spin，**删除 `tryPredictAndNavigate` 函数**。理由：段1 预测必然失败（`predictTargetPose` 要求 ≥2 样本）；段3 FLOW5 边缘/障碍场景预测点大概率越界/不可达、不进扫描框架、等价 staying put；resultCallback `!active` 兜底极少触发（staying put 不发 nav、nav result 多为残留）且 near-certain 失败。最终兜底统一为 IDLE spin，流程大幅简化。

`[FLOW N]` 标记对应代码中 `[LOST-FLOW N]` INFO 日志，便于 grep 定位（见第 2 章 Q 节与日志编号映射）。

### 相关参数（运行时默认值）

> 运行时实际值取自 `launch/tros_person_following.launch.py` 的 `default_value`（launch 覆盖了 cpp `declare_parameter` 的默认值）。cpp 声明默认值与 launch 不一致时，以 launch 为准。

| 参数 | 运行时默认 | cpp 声明默认 | 作用 |
|------|------|------|------|
| `tracking_to_lost_timeout_sec_` | 2.0 | 1.0 | 连续丢失多久才从 TRACKING 转 LOST |
| `lost_to_idle_timeout_sec_` | 1.0 | 5.0 | LOST 持续多久才回 IDLE |
| `idle_observe_duration_sec_` | 1.0 | 3.0 | IDLE 观察候选目标的时长 |
| `static_target_move_thr_` | 0.1 | 0.2 | "移动"判定阈值（窗口内世界位移 > 此值才算移动） |
| `static_switch_activity_window_sec_` | 1.0 | 2.0 | 测量移动的时间窗口 |
| `predict_window_sec_` | 6.0 | 4.0 | 预测速度估计窗口 |
| `predict_lead_sec_` | 2.0 | 2.0 | 预测前瞻时间（外推 v×(since+lead)） |
| `predict_max_dist_` | 3.0 | 3.0 | 外推位移上限（超过则按比例缩放） |
| `predict_stale_sec_` | 10.0 | 5.0 | 样本过期阈值（最新样本超此值不预测） |
| `spin_radian_` | 3.14（π=180°） | 1.57（π/2） | 每次搜索旋转角度 |
| `idle_search_total_timeout_sec_` | 60.0 | 30.0 | IDLE 搜索总时长上限（超时停 spin） |
| `belief_search_timeout_sec_` | 8.0 | 8.0 | Belief search 总超时（s） |
| `belief_search_max_rounds_` | 2 | 2 | Belief search 最大迭代轮数 |

### 目标选择的筛选条件（含宽高）

目标选择经过**三层筛选**，**包含人的宽高**。IDLE 选目标与 LOST 期接受新目标（改进 B）共用同一套筛选（LOST 期只跳过观察期）。

| 层 | 代码位置 | 条件 | 运行时阈值 |
|----|---------|------|-----------|
| 第1层 解析预筛 | `cpp:386-400` | 属性齐全(x_cm/y_cm/height_cm/width_cm/y_offset) **且** `height_m >= height_thr_` **且** 位置在范围(x∈[target_filter_range_x_min,max], y∈[target_filter_range_y_min,max]) | target_filter_height_thr=0.5(可配)、range_x=[0.1,2.0]、range_y=[-3.0,3.0] |
| 第2层 valid_persons | `cpp:670-675` | `conf >= confidence_thr_` **且** `width_m >= width_thr_ && height_m >= height_thr_` | target_filter_confidence_thr=0.5、target_filter_width_thr=0.3、target_filter_height_thr=0.5 |
| 第3层 moving 判定 | `cpp:706`/`613` | `targetMovementOverWindow(id, switch_activity_window_sec_) > static_target_move_thr_`（窗口内世界位移） | static_target_move_thr=0.2、switch_activity_window=2.0 |

**宽高的具体作用**：
- **高度**在第1层和第2层都筛（`height_m >= height_thr_`），运行时默认 **0.5**（2026-07-23 由 0.2 调为 0.5，并在 launch 暴露可配）。
- **宽度**只在第2层筛（`width_m >= width_thr_`），运行时默认 **0.3**（launch 已暴露）。
- 第1层只查高度不查宽度；第2层宽高都要满足才算 valid。
- 不满足"移动"的 valid 目标进 IDLE_OBSERVING 观察期（不直接跟随），避免锁定静止目标。

**目标选择类阈值参数表**：

| 参数 | 运行时默认 | cpp 声明默认 | launch 可配 | 作用 |
|------|------|------|:---:|------|
| `height_thr_` | 0.5 | 0.5 | ✅ | 最小人体高度（m），第1+2层筛 |
| `width_thr_` | 0.3 | 0.3 | ✅ | 最小人体宽度（m），第2层筛 |
| `confidence_thr_` | 0.5 | 0.5 | ✅ | 最小检测置信度，第2层筛 |
| `range_x_min_`/`max_` | 0.1 / 2.0 | 0.1 / 2.0 | ✅ | 相机系 x（前向距离）范围，第1层筛 |
| `range_y_min_`/`max_` | -3.0 / 3.0 | -1.0 / 1.0 | ✅ | 相机系 y（横向）范围，第1层筛 |
| `static_target_move_thr_` | 0.2 | 0.2 | ✅ | "移动"判定阈值（窗口内世界位移 > 此值），第3层 |
| `switch_activity_window_sec_` | 2.0 | 2.0 | ✅ | 测量移动的时间窗口，第3层 |

**注**：`height_thr_` 2026-07-23 前只在 cpp 声明（默认 0.2）、launch 未暴露；现已补到 launch（`default_value='0.5'`）可运行时配置，cpp 默认同步为 0.5。`target_filter_range_y_min/max` 的 cpp 默认(-1/1)与 launch(-3/3)不一致，以 launch 运行时值为准。


---

## 2 目标 Lost 后行为 / Belief 搜索时机 / 协同 / 预测限制（速查）

> 本节针对四个高频问题给出**与当前代码一致**的精确结论，便于快速查询与讨论。行号对应 `src/person_following_node.cpp`（2026-07-23 状态）。

### Q1. 跟随目标 lost 后，机器人是什么行为？

分两个阶段，由 `tracking_to_lost_timeout_sec_`(运行时 2.0s) 和 `lost_to_idle_timeout_sec_`(运行时 1.0s) 两个阈值切分：

**阶段 0：帧级丢失（TRACKING 态内，`target_lost_=true`，`cpp:480-485`）**
- 目标一帧未检测到即置 `target_lost_=true` 并记 `tp_target_lost_`，但**仍处 TRACKING 状态**。
- 机器人**被动等待**：不发新 nav goal、不 cancel 旧 goal、不 spin、不预测。若旧 nav goal 未完成仍继续执行（即可能仍朝原目标最后位置走）。
- 此阶段每帧仍尝试按原 track_id 找回（`cpp:473-479`），找回则 `target_lost_=false` 继续 TRACKING。

**阶段 1：进入 LOST（连续丢失 > `tracking_to_lost_timeout_sec_`，`cpp:486-509`）**
1. `track_state_ = LOST`，记 `tp_lost_`，`target_lost_=false`。
2. **cancel 活跃 nav goal**（`cpp:498-505`）——旧 goal 是冲着人最后位置去的，已失效；也防止后续 IDLE spin 与 Nav2 抢 `/cmd_vel`。
3. `publishBlindZoneObserving(true)` + 上报 `FollowStatus::LOST`。
4. **调 `startBeliefSearch()`**（`cpp:508`，改进 M）——LOST 入口的主动搜索，替代旧的 `tryPredictAndNavigate`。

**阶段 2：LOST 期间每帧（`cpp:576-`，detectResultCallback LOST 分支）**
- **优先按原 track_id 找回**：找到 → 距离门控通过（`d(target,LKP) ≤ currentRelockDist()`，见"LOST 期重锁距离门控"子流程）→ `belief_search_.active=false` + `spin_stop_requested_=true` → 恢复 TRACKING [FLOW 7]。
- **其次接受新目标**（改进 B）：原 id 找不到时，若有 valid(置信度+尺寸) + moving(窗口位移>阈值) + 距离门控通过的新目标 → 直接切新目标恢复 TRACKING，跳过 IDLE 观察期 [FLOW 8]。
- **都没找到** →：
  - 若 `scan_completed==true`（扫描线程刚完成）→ 调 `continueBeliefSearch()` 推进到下一轮 [FLOW 11]（2026-07-23 新增轮询推进，见下"推进机制"）。
  - 否则等 nav 到达 → `resultCallback`（SUCCEEDED **或 ABORTED** 都推进，见下）→ `continueBeliefSearch()`。

**阶段 3：LOST 超时回 IDLE（`cpp:444-468`，2026-07-23 修复）**
- **关键**：LOST→IDLE 超时在 belief search 在跑（`belief_search_.active==true`）时**挂起**——条件为 `!belief_search_.active && (now-tp_lost_ > lost_to_idle_timeout_sec_)`（`cpp:458`）。这样 belief search 能跑完整 2 轮（受 `belief_search_timeout_sec_`=8s 约束），不被 1s 的 `lost_to_idle_timeout_sec_` 打断。
- belief search 结束（`active=false`，超时/轮数耗尽/无观测点/重锁）后，下一帧 `!active` 成立，LOST 超时立即触发 [FLOW 13]。
- 回 IDLE 时：`spin_stop_requested_=true` + cancel 残留 nav → `track_state_=IDLE` + `tracking_track_id_=0` + 上报 `IDLE_SEARCHING`。
- IDLE 搜索：valid_persons 空 → 异步 spin（`continue_search`）；有 valid 但无 moving → IDLE_OBSERVING 观察期；有 moving → 选最近进 TRACKING。

#### belief search 的推进机制（2026-07-23 修复，关键）

**问题背景**：`continueBeliefSearch` 原逻辑用 `scanning` 字段判状态，但无法区分"到达观测点该起扫描"和"扫描完成该推进"——两者都是 `scanning==false`。原 1s LOST 超时掩盖了它（belief search 几乎跑不到第二轮）。放宽数后这个卡点暴露：扫描完成后无 nav result 触发（扫描发 cmd_vel 不发 nav goal），会卡在 round 0 反复扫描不推进。

**修复**：加 `scan_completed` 标志 + 三触发点推进：
1. **`scan_completed` 标志**（`BeliefSearchState`，`h:414`）：扫描线程结束时设 `scan_completed=true`（`cpp:2036`），区分于"到达未扫描"。
2. **起扫描分支**条件改为 `!scanning && !scan_completed`（`cpp:2016`）。
3. **推进分支**检测 `scan_completed`→ `round++` → 选下一观测点 → nav（`cpp:2053`），消费后重置 `scan_completed=false`。
4. **三个触发点**调 `continueBeliefSearch`：
   - nav **SUCCEEDED**（`cpp:2363`）
   - nav **ABORTED**（`cpp:2378`，2026-07-23 新增——nav 失败也推进，避免放宽数后白等 8s）
   - nav **CANCELED**（`cpp:2390`，仅 LOST+active 时推进）
   - LOST 每帧轮询 `scan_completed`（`cpp:652`，2026-07-23 新增——扫描完成后无 nav result，靠每帧轮询推进到下一轮）
5. `startBeliefSearch` 初始化时重置 `scan_completed=false`（`cpp:1963`）。

**完整推进链路**：nav 到达(succeeded/aborted)→`continueBeliefSearch`→起扫描(`scanning=true`)→扫描线程完成(`scan_completed=true`)→下一帧 LOST 轮询→`continueBeliefSearch`→`round++`→选下一观测点→nav→下一轮扫描……→`round>=max` 或超时→`active=false`→LOST 超时(挂起解除)→回 IDLE。

**一句话总结**：lost 后机器人= cancel 旧 goal → 沿目标最后运动方向主动导航到观测点并定向扫描（最多 2 轮，**不受 1s LOST 超时打断**）→ 期间优先找回原目标、其次接受 moving 新目标 → belief search 跑完或 LOST 超时（1s，仅 belief 未激活时）未恢复则回 IDLE 原地旋转搜索。

### Q2. 激活 Belief-Guided 搜索的时机是什么？

**唯一激活点**：TRACKING → LOST 转换的瞬间，`cpp:523` 调一次 `startBeliefSearch()`。

- 它**不是**每帧调用，也**不是**在 LOST 期间反复触发。LOST 期间只通过 `continueBeliefSearch()`（nav 到达时）迭代，不再重新 `startBeliefSearch()`。
- `startBeliefSearch()` 内部三段（`cpp:1910-2045`）：
  1. **段1 取 LKP**：`getLastKnownPose()` 提取 LKP + (vx,vy,speed)；**无运动历史(样本<2)** → 直接 staying put 等 LOST 超时回 IDLE [FLOW 4]，不激活 belief search（方案甲：预测也必然失败，`predictTargetPose` 同样要求 ≥2 样本）。
  2. **段2 越界更新 LKP**：LKP 越界时回溯历史找地图内样本更新 LKP（+重算运动方向）；LKP 在地图内则跳过 [FLOW 6 rewinding]。
  3. **段3 选首个观测点（预测点优先）**：`predictTargetPose` 成功 + goal点cell free（`isGoalReachable`） → 用预测点（上报 `LOST_PREDICTING`）；预测不可用 → `pickLkpObservationPose` 选 LKP 附近观测点（上报 `LOST_BELIEF_NAV`）；**无 free 观测点(含历史)** → staying put 等 LOST 超时回 IDLE [FLOW 5]，不激活（方案甲）。
  4. 选到 → `belief_search_.active=true` + nav + searched_points 记录 → 进 `continueBeliefSearch` 扫描/迭代框架。
- **激活前提**：进 LOST 时目标有 ≥2 帧运动历史样本 + 能选到 free 观测点（预测点 cell free 或 LKP 附近/历史有 free 点）。否则不激活，staying put 等 IDLE。

#### "无运动历史"的含义与触发场景

`startBeliefSearch` 段1 的条件（`cpp:1915-1924`）：

```cpp
if (!getLastKnownPose(tracking_track_id_, lkp, vx, vy, speed)) {
  // no motion history (<2 samples), staying put (wait LOST timeout -> IDLE)
  return;   // staying put，等 LOST 超时回 IDLE（方案甲，无独立预测兜底）
}
```

**"运动历史"是什么**：`target_pos_history_` 里为该 track_id 存的**世界坐标系位置采样序列**（每帧检测到目标时记录 map 系位置 + 时间戳）。有了它才能估出 `(vx,vy,speed)`、算 LKP、判定 `has_motion`。**"无运动历史" = 该 id 的样本数 < 2**（`getLastKnownPose` 返回 false 的条件，`cpp:1803-1838`），无法估计运动速度。

**何时会"样本不足 2 个"就进 LOST**：通常进 LOST 是连续 `tracking_to_lost_timeout_sec_`(2s) 没检测到，能跟踪说明之前检测到过，样本按理够。但几种情况会让样本不足：
1. **刚锁定就丢失**：新切的目标只检测到 1 帧就丢（目标快速出画面/遮挡），来不及攒第 2 个样本。
2. **目标切换瞬间丢失**：静止目标切换（`cpp:533`）切到新目标 B，B 刚被接受就丢失，B 的历史样本不足。
3. **历史被清理**：`pruneTargetPosHistory` 按年龄清理过期样本，清理后该 id 剩不到 2 个。
4. **track_id 重分配**：MOT 给了新 id，新 id 没有历史。

**触发后的链路**（直接停在原地）：
```
startBeliefSearch
  └─ getLastKnownPose 失败（样本<2）
      └─ 直接 staying put（不调 tryPredictAndNavigate）
          └─ 机器人停下，等 LOST 超时(lost_to_idle_timeout_sec_)回 IDLE spin
```
方案甲简化：此前段1 调 `tryPredictAndNavigate`，但后者内部 `predictTargetPose` 同样要求 ≥2 样本、必然返回 empty（见 Q4 限制 1），最终也是 staying put——所以该调用是必然失败的冗余，已去掉。

**设计含义（值得注意）**：belief search 的策略是"沿目标运动方向选观测点"（`has_motion` 分支沿 `(vx,vy)` 找点，`cpp:1989-1993`），没运动历史就不知道人往哪走、无法有依据地选观测点。但 `pickLkpObservationPose` 其实还有**静止目标的径向 8 方向搜索兜底**——只是 `startBeliefSearch` 在更前面（`getLastKnownPose` 这步）就拦截了，根本走不到径向搜索。因此：
- **有历史但 `speed < static_target_move_thr`（静止）**：`has_motion=false`，belief search **激活**，走径向 8 方向搜索。
- **无历史（样本<2）**：belief search **不激活**，直接 staying put 等 IDLE。

即"静止丢失"和"无历史丢失"走不同路径——前者仍能激活 belief search（径向搜索），后者完全退回被动等待。这是"无依据无法主动搜索"的合理退让，但意味着**刚锁定即丢失**的目标无法主动找回，只能靠 1s 后 IDLE 的原地旋转。

### Q3. LOST 搜索如何协同？（预测点优先 + belief search，无独立预测兜底）

**核心关系**：LOST 入口统一走 `startBeliefSearch()`，内部"预测点优先 + belief search fallback"，两者共用 `continueBeliefSearch` 扫描/迭代框架。原 `tryPredictAndNavigate` 已删除——预测失败的兜底统一为 staying put 等 LOST 超时回 IDLE spin，无独立预测兜底函数。

| 触发点 | 代码位置 | 谁先执行 | 说明 |
|--------|---------|---------|------|
| LOST 入口 | `cpp:523` | `startBeliefSearch()` | 主路径。内部预测点优先 + belief fallback；"无历史"(FLOW 4)/"无观测点"(FLOW 5) 时直接 staying put（方案甲）。 |
| nav **SUCCEEDED** 到达 | `resultCallback` | 仅推进 | `if (belief_search_.active) continueBeliefSearch();` —— 激活则继续迭代；未激活则不做事（残留 result，等 LOST 超时回 IDLE）。 |
| nav **ABORTED** | `resultCallback` | 仅推进 | 同 SUCCEEDED——nav 失败也推进 belief search（active 时），避免白等 8s；未激活不做事。 |
| nav **CANCELED** | `resultCallback` | 仅推进 | 仅 LOST+active 时调 `continueBeliefSearch`（cancel 常是主动取消）。 |
| LOST 每帧轮询 | `cpp:652` | 仅推进 | `if (active && scan_completed)` → `continueBeliefSearch`——扫描完成后无 nav result，靠每帧轮询推进到下一轮。 |

**协同流程**：
```
LOST 入口
  └─ startBeliefSearch()
      ├─ 有历史 + 选到观测点 → belief_search_.active=true → nav 到观测点 [FLOW 6]
      │     ↓ nav 到达（SUCCEEDED 或 ABORTED 都触发）
      │   continueBeliefSearch()  [belief_search_.active==true]
      │     ├─ 超时 belief_search_timeout_sec_ → 结束 [FLOW 9]
      │     ├─ !scanning && !scan_completed → 定向扫描（朝 LKP）[FLOW 10]
      │     │     └─ 扫描线程完成 → scan_completed=true
      │     │           ↓ 下一帧 LOST 轮询检测到 scan_completed
      │     │     continueBeliefSearch() → round++ → 排除已搜 → 选下一观测点 → nav [FLOW 11]
      │     │           ↓ nav 到达 → 下一轮扫描 …
      │     └─ round≥max 或无更多观测点 → 结束 [FLOW 12] → active=false
      │           ↓ 下一帧 !active
      │         LOST 超时(挂起解除) → 回 IDLE [FLOW 13]
      │
      └─ 无历史 → staying put，等 LOST 超时回 IDLE [FLOW 4]
      └─ 无观测点(预测不可用且无 free 点) → staying put，等 LOST 超时回 IDLE [FLOW 5]
            └─ 被动等 LOST 超时(lost_to_idle_timeout_sec_)回 IDLE spin
```

**关键点**：
- LOST 搜索只有 `startBeliefSearch` 一个入口，内部预测点优先 / belief fallback，共用 `continueBeliefSearch` 框架——不存在"两套机制"。
- 预测失败（段1/段3/resultCallback `!active`）一律 staying put 等 IDLE spin，**无独立预测兜底**（`tryPredictAndNavigate` 已删）。
- 恢复 TRACKING（原 id `cpp:581` 或新目标 `cpp:622`）会 `belief_search_.active=false` + `spin_stop_requested_=true`，中断任何进行中的 belief 搜索/扫描。
- nav ABORTED/CANCELED 也推进 belief search、LOST 每帧轮询 `scan_completed` 推进——这两处让 belief search 在放宽数后能完整跑完 2 轮，不再被 1s 超时打断或卡在 round 0。

#### 两轮扫描的迭代实质：每轮做什么、两轮区别

`round` 从 0 开始（`startBeliefSearch` `cpp:1944`），`max_rounds=2`，所以**两轮 = round 0 和 round 1**。每轮结构相同：**导航到一个观测点 → 原地定向扫描**，轮间靠 `round++` 切换（`cpp:2054`）。

**每轮的执行流程**（`continueBeliefSearch`，`cpp:1998`，两轮相同）：
```
nav 到达观测点（SUCCEEDED 或 ABORTED 都触发）
  → continueBeliefSearch()
    → 超时检查（belief_search_timeout_sec_=8s）[FLOW 9]
    → !scanning && !scan_completed → 起扫描线程，定向扫描 180°（朝 LKP 偏置方向）[FLOW 10]
    → 扫描线程完成 → scan_completed=true
    → 下一帧 LOST 轮询检测到 scan_completed → continueBeliefSearch()
      → scan_completed 消费 + round++
        → round < max(2) → 选下一观测点（排除已搜）→ nav [FLOW 11]  ← 进入下一轮
        → round >= max → 结束 [FLOW 12] → active=false → LOST 超时挂起解除 → 回 IDLE [FLOW 13]
```

**两轮的唯一实质区别 = 观测点位置不同**，由 `pickLkpObservationPose`（`cpp:1840`）的"排除已搜"机制保证两轮去不同地方：

- **观测点选取**：
  - 有运动方向（`has_motion`）：沿 `(vx,vy)` 在 LKP 前方试 3 距离(`obs_dist` 2m/3m/1m)×3 角度(0/±30°)，取第一个 free。
  - 静止（`!has_motion`）：以 LKP 为中心 8 方向 × 3 环(1/2/3m)径向，取第一个 free。
- **排除已搜**（`cpp:1846-1851`）：每轮 nav 完成后把当前观测点加入 `searched_points`（`cpp:2067`）；下一轮选点时跳过 0.5m 内已搜过的点。
  - **round 0**：选第 1 个 free 观测点（沿运动方向最近的 / 径向第 1 环）。
  - **round 1**：排除 round 0 去过的点，选**第 2 个不同的 free 观测点**（沿运动方向更远/偏另一侧 30°；径向则不同方向或更远环）。

**扫描动作两轮基本相同**：都是 `SpinInPlaceWCmdVel(spin_radian_ * dir)` 原地转 180°，方向朝 LKP 偏置（`cpp:2025-2032`：`rel_angle=atan2(LKP-robot)-robot_yaw`，LKP 在左就左转）。隐含区别：两轮 nav 到了不同观测点→机器人位置变了→"朝 LKP 方向"算出的 `rel_angle`/`dir` 可能不同，故两轮**实际旋转方向可能不同**（但角度 180° 和机制一样）。

**两轮协同逻辑**：在目标最后位置(LKP)附近，从两个不同观测点各扫一次，扩大重锁覆盖、减少盲区——尤其目标绕到障碍物另一侧时，换个观测点可能就看到了。只 1 轮只从 1 个角度/位置看，2 轮从 2 个位置看覆盖更广。

**固有局限**：两轮都围绕 LKP 附近选观测点。若目标 lost 后**继续移动远离 LKP**，两轮都在 LKP 附近找可能都找不到——靠 `has_motion` 时观测点沿运动方向外推覆盖"目标可能走到的地方"，但若目标移动距离 > `obs_dist`(2m) 覆盖范围，2 轮够不着。这是 belief search 对"快速远离目标"的固有局限，此时靠 LOST 超时回 IDLE 的 60s spin 兜底。

**一句话**：每轮 = nav 到一个观测点 + 朝 LKP 方向原地扫 180°，动作相同；两轮唯一实质区别是**观测点位置不同**（靠"排除 0.5m 内已搜点"保证），两轮协同 = 从 LKP 附近两个不同位置各扫一次以扩大重锁覆盖。

### Q4. 轨迹预测（predictTargetPose）有哪些限制条件？

`predictTargetPose(id)`（`cpp:1651-1710`）返回 empty（不预测）的全部条件，任一满足即放弃预测：

| # | 限制条件 | 代码位置 | 含义 |
|---|---------|---------|------|
| 1 | 无运动历史 / 样本 < 2 | `cpp:1657-1660` | `target_pos_history_` 中该 id 样本不足 2 个，无法估速度 |
| 2 | 最新样本过期 | `cpp:1665-1667` | `(now - newest.stamp) > predict_stale_sec_`(运行时 10.0s)，目标太久没见，外推无意义 |
| 3 | 窗口内样本不足 | `cpp:1669-1678` | `predict_window_sec_`(运行时 6.0s) 内找不到 oldest 样本，或 oldest==newest（单点） |
| 4 | 时间差过小 | `cpp:1679-1682` | `dt = newest-oldest <= 1e-6`，除零保护 |
| 5 | 目标静止 | `cpp:1685-1689` | `speed < static_target_move_thr_`(0.2)，无明确运动方向，外推无意义 |
| 6 | **外推点越出 costmap 边界** | `cpp:1706-1722` | 外推点 `worldToCell` 越界 → 返回 empty。**2026-07-23 新增**，防止越界 goal 让 planner_server 疯狂报 `worldToMap failed` 并白等 `isGoalReachable` 超时（`isGoalReachable` 已改为 costmap cell free 检查，2026-07-24） |

**通过以上限制后**，外推还受两层约束：
- **位移幅度 clamp**（`cpp:1695-1699`）：外推距离 > `predict_max_dist_`(3.0m) 则按比例缩放，限制最远预测距离。
- **可达性校验**（`startBeliefSearch` `cpp:2080`）：外推点经 `isGoalReachable`（costmap cell free 检查，2026-07-24 由 ComputePathToPose 改为 cell free）验证 goal cell 是 free 才发 nav goal；cell 不 free 则 fallback 到 `pickLkpObservationPose`。
- **冷却约束**：已废弃——`tryPredictAndNavigate` 函数与 `predict_cooldown_sec_`/`last_predict_time_` 参数已于 2026-07-27 一并删除（prediction-first 改由 `startBeliefSearch` 直接调 `predictTargetPose`，不再走独立预测兜底+冷却的链路）。nav 到达回调的无限循环由 LOST 超时挂起 + belief search `active` 门控兜底，无需冷却限流。


**无 costmap 时的例外**（`cpp:1714-1716`）：若 `latest_costmap_` 为空（启动初期 costmap 未就绪），限制 6 **跳过**、保留预测，避免启动期回归。

**典型失败日志对照**：
- `LOST; no usable motion history for target id=N, staying put` → 限制 1-5 之一命中。
- `LOST; predicted target id=N at (x,y) is outside the costmap, not predicting` → 限制 6 命中（新增）。
- `isGoalReachable: goal (x,y) cell not free in costmap, treating as unreachable` → 预测点 cell 不 free（occupied 或 unknown），fallback 到 `pickLkpObservationPose`（2026-07-24 改动，原 `Recv goal response timeout` / `Path planning aborted` 不再出现）。

---
