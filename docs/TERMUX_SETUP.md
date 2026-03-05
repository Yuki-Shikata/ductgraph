# Termux Setup (ductgraph_proj)

この手順は **Termux (F-Droid版)** を前提に、`ductgraph_proj` を確実に動かすための最小セットアップです。
「Pythonが無い」「venvが壊れた」「numpyが無い」などの手戻りを潰します。

---

## 0) リポジトリ位置（例）

本プロジェクトは次のような場所にある想定：

- `~/storage/shared/projects/ductsim/ductgraph_proj`

確認（例）：
- `pwd` で現在位置
- `ls` で `ductgraph/` `tools/` `cases/` `tests/` が見えること

---

## 1) Termux パッケージ（Python / 依存）

目的：Termux側に Python 実行環境と最低限の依存を入れる。

インストールするもの：
- `python`
- `which`
- `python-numpy`

確認：
- `python -V` が表示できること

---

## 2) venv 作成（推奨：毎回これを使う）

目的：Termux更新・入れ替え時の破損リスクを局所化し、再現性を上げる。

方針：
- venv は `~/venvs/ductsim` に固定
- 壊れたら「消して作り直す」が正規手順

やること（概要）：
1. venvが有効なら deactivate
2. `~/venvs/ductsim` を削除
3. `python -m venv ~/venvs/ductsim` で作成
4. `source ~/venvs/ductsim/bin/activate` で有効化
5. `python -V` と `pip -V` で確認

---

## 3) 動作確認（最短）

目的：入口スクリプトが動くことを確認する。

確認対象：
- `tools/run_realcase_commission_scale.py` がコンパイルできること
- 実行して結果が出ること

---

## 4) よくあるトラブル

### A. `python: not installed` が出る
原因：Termuxに python パッケージが入っていない。
対策：`python` をインストール。

### B. `ModuleNotFoundError: numpy`
原因：numpy が無い。
対策：`python-numpy` をインストール。

### C. venv の python が死んでいる (`bad interpreter`)
原因：Termuxの入れ替え/更新で、venv が参照していた python のパスが無効になった。
対策：venv を削除して作り直す（2章の手順）。

---

## 5) 運用ルール（手戻り防止）

- 実行前に必ず venv を有効化する。
- 依存を追加したら docs に追記する（後で詰まらないため）。
- 「動かない」ときは、まず `python -V` と `pip -V` と `which python` を確認する。