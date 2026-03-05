# Entrypoints (ductgraph_proj)

このリポジトリは「本体(ductgraph)」と「入口(tools / cli)」を分けて運用する。
迷子防止のため、入口を明示して固定する。

---

## Primary Entrypoint（正）

### 1) 実運用の入口（real_case commissioning + scaling + power(all)）
- `tools/run_realcase_commission_scale.py`

このスクリプトが「正」の入口。
- 入力条件（commission_and_scale の kwargs）を print して再現性を担保する
- `scaling_cases` は通常 `all` のみを使う（現時点の運用）
- all の結果から消費電力を計算して表示する

オプション方針：
- 入力ログを消したい場合は環境変数でOFFにできる（実装側に依存）

---

## Secondary Entrypoints（診断・解析）

### 2) ダンパモデルの単調性/能力診断（real_case掃引）
- `tools/sweep_realcase_dampers.py`

用途：
- ダンパ角度の掃引で単調性・収束性を確認
- full-open / full-close で「能力不足（Authority/Capacity）」を診断

注意：
- 本体を変更しない診断ツール
- 実運用の入口ではない（あくまで検証用）

---

## Utility（安全置換）

### 3) compileが通った時だけファイル差し替え
- `tools/swap_if_ok.py`

目的：
- 編集したファイルを差し替える前にコンパイル検査し、事故を防ぐ

運用：
- デフォルトはバックアップ無し（ゴミ増殖を避ける）
- 必要なときだけ `--backup` で `.bak` を作る

---

## CLI（ductgraph内のコマンド入口）

- `ductgraph/cli_commission_scale.py`
- `ductgraph/cli_cav.py`

位置付け：
- tools/ を “正の入口” とする
- CLI は補助（将来の配布・利用のための薄いラッパ）

---

## 本体の分類（読む順序のガイド）

読む順序の推奨：
1. `cases/real_case.py`（入力データ）
2. `tools/run_realcase_commission_scale.py`（入口：何を呼んでいるか）
3. `ductgraph/commissioning_scale.py`（調整＋スケーリング）
4. `ductgraph/control_cav.py` / `ductgraph/control_damper.py`（CAV・ダンパ）
5. `ductgraph/solver_nodehead.py`（流量計算の核）
6. `ductgraph/power_curve.py`（電力計算：曲線＋アフィニティ＋効率）
7. `ductgraph/physics/*` と `ductgraph/ductloss.py`（損失モデル）