# Termux Setup (minimal, current)

ductgraph_proj/ を Termux で実行する最小手順。

## 1. 前提パッケージ
- python
- python-numpy
- git

確認:
python -V

## 2. venv 作成
python -m venv ~/venvs/ductgraph
source ~/venvs/ductgraph/bin/activate
python -m pip install -U pip pytest numpy

## 3. 実行
repo root で:
python ductgraph_proj/tools/run_realcase_commission_scale.py

## 4. テスト
repo root で:
python -m pytest -q ductgraph_proj/tests/nodehead

python が見つからない場合は、venv を有効化して同じコマンドを実行する。

## 5. 典型トラブル
- ModuleNotFoundError:
  - venv 未有効化、または依存未インストール
- bad interpreter:
  - venv 破損。削除して再作成
