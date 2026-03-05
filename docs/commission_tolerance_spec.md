# Commissioning tolerance spec (single source of truth)

## 記号（変数の意味）
- Q_i : 端末 i の流量（m3/s）。符号は無視して |Q_i| を合否・比較に使う。
- Qd_i : 端末 i の設計流量（m3/s）。
- theta_i : 端末 i のダンパ開度（deg）。大きいほど開く（=流量が増える前提）。
- maxload : 「全台運転時に最も開度が大きい端末」(max(theta_i))。

## tolerance（用途別に分離する）
### (A) 合否（不足NG）用：eps_under_rel
- 意味：数値計算の丸め等で「ほんの僅かに下回った」ように見える誤差を吸収するための“極小”相対許容。
- 合否判定（不足NG）：
  - Pass iff すべてのダンパ付き端末 i で
    Q_i >= Qd_i * (1 - eps_under_rel)

- 注意：これは「合否専用」。±許容ではなく“下側のみ”。
- 推奨デフォルト：eps_under_rel = 1e-4 〜 1e-3（まずは 1e-4）

### (B) 収束（止め時）用：tol_conv_rel
- 意味：反復調整（ダンパ二分探索やround反復）を“止める目安”。
- これは合否ではない。合否に混ぜない。
- 例：ダンパ調整は「不足側で止めない」を原則にする
  - 目標を超えた側（Q_i >= Qd_i）を維持しつつ、
    (Q_i - Qd_i)/Qd_i <= tol_conv_rel になったらその端末調整を終了
- 推奨デフォルト：tol_conv_rel = 1e-3（0.1%）〜 1e-2（1%）

### (C) 警告（合理性）用：tol_warn_rel / theta_warn
- 意味：設備評価や合理性の目安。合否ではない。
- 例：過大風量警告： (Q_i - Qd_i)/Qd_i > tol_warn_rel
- 例：maxload開度の合理性（制御余裕）警告：
  - 目標 theta_center = 75deg
  - 許容（非対称）：-10deg / +5deg
  - よって warn 範囲： [65, 80] から外れたら WARN

- 推奨デフォルト：tol_warn_rel = 0.03（3%）

## 包含関係（衝突を避けるための大小関係）
- 0 <= eps_under_rel  <<  tol_conv_rel  <=  tol_warn_rel
- 重要：tol_warn_rel(=3%) を合否に使うのは禁止（不足NGと矛盾するため）。

