# Commissioning Spec (current implementation)

この文書は ductgraph/commissioning_scale.py の現行仕様を記述する。

## 1. 目的
- full-load で基準周波数 speed_full を決める
- partial-load を設計風量比でスケーリングして成立性を判定する

## 2. 主要入力
- full_active_cav_edge_ids: full-load で active な CAV edge
- q_design_by_edge: edge ごとの設計風量 [m3/s]
- scaling_cases: (case_name, active_edges) の列
- s_min, s_max: speed ratio 探索範囲
- theta_center, theta_band: full-load 目標開度帯
- eps_under_rel: 不足判定の許容
- off_damper_u / theta_off_deg: OFF 端末の閉止方法

## 3. full-load 基準周波数決定

### 3.1 評価方式
各 speed 候補で full-active CAV を同時解し、以下を判定:
- feasible:
  - solver 収束
  - 全 active edge で Q >= Qdes*(1-eps_under_rel)
  - 各 edge の theta が取得できる
- in_band:
  - 開度上位 critical_n（1〜2本）の全てが
    theta_center-theta_band <= theta <= theta_center+theta_band

### 3.2 探索
- coarse scan: n=25 分割で [s_min, s_max]
- feasible 候補がある場合:
  - dense scan: n=120
  - in-band 候補があれば最小 speed を採用
  - 左境界を二分探索で refine（28反復）
- in-band 候補が無い場合:
  - best-fit を採用
  - 優先順位:
    1. 過開（90deg 側）回避
    2. 過閉回避
    3. 帯からの距離最小
    4. 低速優先

### 3.3 feasible 候補ゼロ時
- speed_full = s_max
- full-open 診断で分類:
  - capacity_shortage: 全開でも不足
  - solver_unstable: full-open では不足しないが CAV 同時解が不安定

### 3.4 full-load 診断コード
- ok
- under_static
- over_static
- capacity_shortage
- solver_unstable

## 4. partial-load

### 4.1 速度決定
- 初期速度:
  s_case_raw = speed_full * (ΣQdes_active / ΣQdes_full)
- 適用速度:
  s_case = clamp(s_case_raw, s_min, s_max)

### 4.2 CAV 同時解
- active edge のみを未知数として同時解
- OFF edge は閉止
  - off_damper_u が有効なら u 固定
  - 無効時は theta_off_deg 固定

### 4.3 判定
- NG 条件:
  - solver 未収束
  - 任意 active edge で Q < Qdes*(1-eps_under_rel)
- 不足/未収束時は full-open チェックを実施し、
  WARN full-open undershoot をメッセージへ追加
- s_case_raw が clamp された場合は WARN speed clamped

## 5. 出力
CommissionScaleResult:
- speed_full
- res_full
- cases[]
- index_edge_id
- full_load_band_ok
- full_load_diag, full_load_diag_msg
- full_load_critical_edges, full_load_critical_thetas

ScalingCaseResult:
- name, speed_ratio, thetas, res, ok, msg
- max_err（最大不足量）
- saturated_90（full-open でも不足の edge）
- achieved_q_abs

## 6. 補足
- seed_index_edge_id は full-load feasible 候補ゼロ時の fallback index 用であり、
  基準周波数の主判定には使わない。
- q65/q85 は互換のため残るが NaN を返す deprecated property。
