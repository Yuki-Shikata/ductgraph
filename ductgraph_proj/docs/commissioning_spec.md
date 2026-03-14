# Commissioning 仕様（正本）: Two-stage commissioning for duct network

このファイルは commissioning（試運転調整）ロジックの**唯一の正本**（Single Source of Truth）です。
実装・テスト・会話で仕様がぶれたら、必ずこの文書へ合わせます。

---

## 1. 対象と目的

### 対象
- ダンパ付き端末（terminal edges with damper）を持つダクトネットワーク
- 代表例: CAV 端末を複数持つ系統（全台運転時）

### 目的（実務の合否）
- **全端末で設計風量を下回らないこと（不足NG）**
- ただし、**数値計算上の微小誤差**は「誤差として無かったもの」として扱える（後述の eps_under_rel）

---

## 2. 用語と変数（略語を使う前に必ず意味を書く）

### 風量
- `Q_i` : 端末 i の最終解での風量（m3/s）
- `Q_design,i` : 端末 i の設計風量（m3/s）
- `Q_design` : 端末が全て同じ設計風量の場合の共通値（m3/s）
- `Q_maxload` : maxload 端末の風量（m3/s）

### ダンパ開度
- `theta_i` : 端末 i のダンパ開度（deg）
- `theta_center` : maxload の目標開度（例: 75 deg）
- `theta_band` : maxload 目標の許容帯（現実装は ± の対称帯）
  - 注意: 実務上は「+5/-10（80/65）」のような非対称帯を使うことがある。その場合は実装拡張する。

### maxload（最大負荷端末）
- **定義（実務）**: 全台運転時に **最も開度が大きい端末**（= いちばんきつい端末）
- 現実装では `maxload_edge_id` を入力で与える（将来的に自動判定する場合は仕様追記）

---

## 3. “tol” を用途別に必ず分ける（混ぜると事故る）

ここが一番重要。用途が違うのに同じ tol 変数を使うと、合否・収束・警告が衝突する。

### (A) 合否用（不足NGの数値誤差吸収）: eps_under_rel
- `eps_under_rel` : 数値誤差吸収の相対許容（無次元、例: 2e-4 = 0.02%）
- **合否判定は片側（下側のみ）**
  - Pass 条件（端末 i）:
    - `Q_i >= Q_design,i * (1 - eps_under_rel)`
- これは「数値誤差だけ許す幅」。大きくして実務許容を表す目的では使わない。

**推奨値**
- `eps_under_rel = 2e-4`（0.02%）をデフォルト推奨
- 目安レンジ: `1e-4` ～ `5e-4`
  - これを超えるなら solver 側の収束精度（残差）やアルゴリズム側を疑う

---

### (B) 収束（停止）用: tol_conv_rel
- `tol_conv_rel` : 調整アルゴリズムが「止まる」ための相対目安（無次元）
- これは**合否ではない**。あくまで反復調整（rounds/bisection）の停止条件。
- 不足NG運用では、停止条件の設計として自然なのは：
  - 「不足側では止めない（必ず Q>=Q_design 側に寄せて止める）」または
  - 「最終解の合否判定は eps_under_rel で別に行い、停止条件は “十分近い” を見て止める」

**推奨値**
- `tol_conv_rel = 1e-3`（0.1%）をまず推奨
- 目安レンジ: `1e-3` ～ `1e-2`
  - 収束が遅い/振動する場合は `rounds` を増やすか、アルゴリズム改善を検討

---

### (C) 警告用（合理性・運用）: tol_warn_rel / theta_warn_*
- `tol_warn_rel` : 風量が設計から外れている等の警告の目安（無次元）
- `theta_warn_low/high` : maxload の開度が合理性・制御余裕の観点から外れている警告

**警告は合否に使わない**（WARN のみ）

**推奨値**
- `tol_warn_rel = 0.03`（3%）※これは「目安」。合否ではない。
- `theta_warn_low = 65 deg`
- `theta_warn_high = 85 deg`
- 実務の中心目標: `theta_center = 75 deg`
  - 目標帯（実務イメージ）: 65～80（+5/-10）を好む  
    ※現実装の `theta_band` が対称なら、暫定は `theta_band=10`（65～85）として運用し、非対称帯は将来拡張

---

### (D) solver 収束（別物）
- ネットワーク solver が方程式を解けたか（`res.converged`）の判定は、
  commissioning の tol 群とは別。ここが悪いと commissioning 判定自体が意味を持たない。

---

## 4. 必要な包含関係（大小関係）
衝突を避けるため、必ず次を満たす。

- `eps_under_rel << tol_conv_rel <= tol_warn_rel`
  - 合否用 eps は極小
  - 収束用 conv はその上（止め時）
  - 警告用 warn はさらに上（3% など）

---

## 5. 合否と警告の最終判定（最終解のみ）

### 合否（Pass/Fail）: 最終解でのみ判定
- Pass 条件:
  1) solver が収束: `res.converged == True`
  2) 全端末で不足NG（数値誤差だけ許容）:
     - `Q_i >= Q_design,i * (1 - eps_under_rel)`  for all terminals i

### WARN（合否に影響しない）
- maxload 開度が合理性範囲から外れる（例: 65～85 を外れた等）
- 風量が設計から大きく外れている（例: ±3% など）  
  ※ただし不足NGの合否判定は eps_under_rel で別に行う

---

## 6. Two-stage commissioning（アルゴリズム概要）

### Stage 1: fan speed tuning（maxload を 75 deg 目標に）
- maxload 端末のダンパを `theta_center` に固定し、
- fan speed ratio `s` を調整して `Q_maxload` が `Q_design,maxload` に近づくようにする
- ただし他端末の開度が相互干渉するので、実装では「他端末の現在開度」を反映した状態で速度を探す

### Stage 2: other damper tuning（他端末をスロットル範囲で合わせる）
- 速度 `s` を固定し、
- 他端末 i の `theta_i` を範囲 `[other_theta_min, other_theta_max]` で調整して `Q_i` を設計へ寄せる
- 反復（rounds）で全端末が安定するまで繰り返す（Gauss-Seidel 的）

---

## 7. この文書に対応する“推奨デフォルト”まとめ

- eps_under_rel = 2e-4（0.02%）
- tol_conv_rel  = 1e-3（0.1%）
- tol_warn_rel  = 0.03（3%）
- theta_center  = 75 deg
- theta_warn_low/high = 65 / 85 deg
- other_theta_max = 75 deg（スロットルのみ: 0..75）

