# ductgraph (runtime README)

この README は ductgraph_proj/ の現行 runtime を運用するための正本です。

## 1. 目的
- 樹形ダクトネットワーク（ループなし）の定常計算
- CAV 端末同時解による full-load commissioning（基準周波数決定）
- 部分負荷の速度スケーリングと不足判定

## 2. 正式な実行入口
- tools/run_realcase_commission_scale.py

実行（repo root）:
./.venv/bin/python ductgraph_proj/tools/run_realcase_commission_scale.py

実行（ductgraph_proj/ 内）:
cd ductgraph_proj
../.venv/bin/python tools/run_realcase_commission_scale.py

## 3. 現行アルゴリズム要点

### 3.1 full-load
- active CAV 全数を同時に解く（単一端末固定方式ではない）
- 候補 speed ごとに成立性を評価
  - solver 収束
  - 全端末 Q >= Qdes * (1 - eps_under_rel)
- 上位 1〜2 本の厳しい端末開度が目標帯に入る最小 speed を採用
- 帯内候補がなければ best-fit 候補を採用し、診断を返す

診断コード:
- ok
- under_static
- over_static
- capacity_shortage
- solver_unstable

### 3.2 partial-load
- 初期速度 s = s_full * (ΣQdes_active / ΣQdes_full)
- s_min..s_max でクランプ
- active CAV を同時解
- OFF 端末は既定で off_damper_u=1e-3（必要なら theta_off 指定に切替可）
- 不足/未収束時は full-open チェックで能力不足を追加診断

## 4. real_case 既定値
- damper model: expk
- gamma: 3.0
- base Hz: 50.0
- Hz 範囲: 25.0..80.0
- full-load 目標帯: 75.0 ± 7.5 deg
- eps_under_rel=0.01, tol_warn_rel=0.03

定義元: cases/real_case_defaults.py

## 5. 主要ファイル
- cases/real_case.py: real case 公開 API（互換 facade）
- cases/real_case_network.py: r / damper_k / fan curve を含むネットワーク生成
- ductgraph/commissioning_scale.py: commissioning 本体
- ductgraph/report_commission_scale.py: レポート整形

## 6. 非対象
- 時系列制御（PID）
- 温度/密度の動的変化
- 実機固有の詳細弁特性の忠実再現

詳細仕様は docs/commissioning_spec.md を参照。
