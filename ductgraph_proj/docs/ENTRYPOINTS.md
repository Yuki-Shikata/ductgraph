# Entrypoints (ductgraph_proj)

実行入口と責務の境界をこの文書で固定する。

## 1. Primary Entrypoint（運用入口）
- tools/run_realcase_commission_scale.py

責務:
- real_case ネットワーク生成
- commission_and_scale 実行
- 入力条件表示
- ケース別の Hz/Q/Pelec と terminal テーブル表示

## 2. Secondary Entrypoint（検証用）
- tools/sweep_realcase_dampers.py

責務:
- ダンパ特性の掃引確認
- 単調性/能力のスポット診断

注意:
- 運用入口ではない

## 3. 計算本体
- ductgraph/commissioning_scale.py
- ductgraph/control_cav.py
- ductgraph/solver_nodehead.py

## 4. ケース定義
- cases/real_case.py（公開 facade）
- cases/real_case_constants.py
- cases/real_case_defaults.py
- cases/real_case_network.py

## 5. 表示層
- ductgraph/report_commission_scale.py

runner から表示ロジックを分離し、計算結果オブジェクトとの責務を分ける。

## 6. 実行時に使わないアーカイブ
- archive/legacy_unused/
- archive/_cmp_d34/
- archive/_cmp_d4/

上記は runtime path ではない。
