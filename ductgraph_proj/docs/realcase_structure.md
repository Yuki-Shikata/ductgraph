# Real-Case Structure (current)

## 1. 実行導線
- tools/run_realcase_commission_scale.py
- ductgraph/commissioning_scale.py
- cases/real_case.py

上記3つを runtime の主線とする。

## 2. cases 分割
- cases/real_case_constants.py
  - edge/node ID
  - 設計風量（cmh）
  - 固定圧境界
  - 物理定数、局部損失係数テーブル
- cases/real_case_defaults.py
  - モデル既定値（damper model/gamma, fan poly, Hz bounds, band, off-u）
  - scaling cases
- cases/real_case_network.py
  - Q_DESIGN 生成
  - 設計点ベースの r 計算
  - damper 目標圧損から damper_k 逆算
  - make_net()
- cases/real_case.py
  - 安定公開 API と再エクスポート

## 3. レポート分離
- ductgraph/report_commission_scale.py

計算結果の整形・表示をここに集約し、runner 側の責務を軽くしている。

## 4. アーカイブ
- archive/legacy_unused/: 旧 CLI/旧物理モジュール
- archive/_cmp_d34/, archive/_cmp_d4/: 比較用ワーク

いずれも runtime path ではない。
