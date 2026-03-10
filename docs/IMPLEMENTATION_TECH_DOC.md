# ductgraph_repo 実装ベースレビュー資料

この文書は `ductgraph_repo` を README/SPEC ではなくコード実装とテストから読んで整理したレビュー用メモである。目的は「この実装を、シミュレーションとして何がレビューできるか」を明確にすることにある。

前提:
- 主な実装本体は `ductgraph/*.py`
- 実案件の入力は `cases/real_case.py`
- 保証範囲の手掛かりは `tests/nodehead/*.py`
- 現状の主対象は「定常」「樹形ネットワーク」「ファン1台」「CAV/ダンパ付き端末」の系である

## 1. シミュレーション全体像

### 関連コード
- `ductgraph/model.py`
- `ductgraph/solver_nodehead.py`
- `ductgraph/control_damper.py`
- `ductgraph/control_speed.py`
- `ductgraph/control_cav.py`
- `ductgraph/commissioning.py`
- `ductgraph/commissioning_scale.py`
- `cases/real_case.py`

### 何をしているか
ダクト系を有向グラフで表現し、各枝に抵抗・ファン・端末・ダンパを持たせ、定常状態の圧力と流量を解く。その上で、試運転調整を「ファン回転数」と「ダンパ角度」の探索として再現している。

### 入力
- ノード集合、枝集合、基準ノード
- 枝ごとの `r`, `fan_poly`, `terminal_orifice`, `damper_k`, `damper_u`
- 固定圧境界 `fixed_p`
- 設計風量 `q_design`
- maxload 端末の指定

### 出力
- ノード圧 `p`
- 枝流量 `q`
- 収束可否
- 端末ダンパ角度
- fan speed ratio
- commissioning の pass/fail, warning

### 計算
- ネットワーク方程式を Newton 法で解く
- 抵抗枝は `dp = r * Q|Q| - h`
- ファン枝は多項式ヘッド曲線を速度比で相似変換
- ダンパは角度から開度 `u` を経由して `r_damper = K / u^2`

### 実装要約
実装は「物理ネットワークソルバ」と「制御ロジック」が分離されている。これはレビューしやすい構造で、流体計算と調整アルゴリズムを別々に検証できる。

### 妥当性・設備動作として何を見られるか
- 与えた抵抗バランスで流量配分がどう変わるか
- maxload 端末に対して必要な fan speed がどの程度か
- 端末を絞ると他端末流量がどう変わるか
- 部分運転で線形スケーリング仮定が成立するか

### 現状実装の限界
- 非定常、PID、起動停止過渡、騒音、温度、密度変化は扱わない
- 樹形前提が強く、実案件 `real_case` では aggregated resistance を直接与えている
- 実機同定済みファン・ダンパ性能ではなく、近似モデルベースである

## 2. データモデル

### 関連コード
- `ductgraph/model.py`

### 何をしているか
シミュレーションの最小単位として `Node`, `Edge`, `Network` を定義している。

### 入力
- `Node.id`, `Node.name`
- `Edge.id`, `frm`, `to`, `r`, `h`, `fan_poly`, `speed_ratio`, `terminal_orifice`, `damper_k`, `damper_u`

### 出力
- 後続ソルバに渡す immutable なネットワーク定義

### 計算
ここでは計算しない。計算に必要なパラメータの格納のみ。

### 実装要約
`Edge` に流体抵抗、ファン、端末オリフィス、ダンパを同居させている。モデルは簡潔だが、複合設備を 1 枝へ集約しやすい。

### 妥当性・設備動作として何を見られるか
機器構成をどの粒度でモデル化しているかを確認できる。特に「枝 = 実ダクト1本」ではなく、「枝 = まとめた等価要素」にもなり得る点が重要。

### 現状実装の限界
- Tee 自体は専用エッジではなく、枝分岐の抵抗に吸収されることが多い
- 複数ファン同期、制御器、センサ等の概念はない

## 3. ネットワークソルバ

### 関連コード
- `ductgraph/solver_nodehead.py`

### 何をしているか
自由ノード圧とファン流量を未知数にして、連続の式とファン式を同時に解いている。

### 入力
- `Network`
- 境界圧 `fixed_p`
- 外部流量 `d`
- Newton パラメータ `max_iter`, `tol`, `damping`
- fan 初期値 `fan_q_init`, 上限制約 `fan_q_cap`

### 出力
- `SolveResult.p`
- `SolveResult.q`
- `SolveResult.converged`
- `SolveResult.iters`
- `SolveResult.residual_norm`

### 計算
- インシデンス行列 `A`
- 連続式 `A_free @ q - d_free = 0`
- ファン枝は `dp = r*Q|Q| - h_fan(Q, s)`
- 抵抗枝は `Q = sign(eff) * sqrt(|eff| / r_eff)`
- ヤコビアンを組んで Newton 更新

### 実装要約
ソルバは node-head 法の簡略版で、流量未知を fan 枝に限り、それ以外の枝流量は圧力差から直接計算する。この構造は単純で高速だが、非線形強結合が増えるほど初期値依存が出やすい。

### 妥当性・設備動作として何を見られるか
- 固定圧境界を与えたときの流量分配
- fan speed を上げたときの全体流量増加
- 抵抗の大きい枝が maxload になるか

### 現状実装の限界
- 抵抗係数 `r` は基本的に固定値で、Re 変化に応じて逐次更新しない
- ループ系テストはあるが、実案件 `real_case` は tree 前提で作られている
- 収束失敗時のロバスト化は限定的

## 4. 実案件入力モデル

### 関連コード
- `cases/real_case.py`

### 何をしているか
fan 1台、幹線 2 本、端末 `a/c/d` の 3 端末系を `Network` として定義している。これが現状の実案件相当入力。

### 入力
- 設計風量: `a=690 CMH`, `c=990 CMH`, `d=990 CMH`
- 固定圧境界: ambient, room_A, room_CD
- 集約抵抗 `r_fan_internal`, `r_trunk_t1`, `r_trunk_t2`, `r_a`, `r_c`, `r_d`
- damper 定数 `damper_k`

### 出力
- `make_net()` が `Network` を返す
- `Q_DESIGN`, `CAV_EDGES`, `SCALING_CASES`, `MAXLOAD_EDGE`

### 計算
- maxload 初期候補は `dp ~ r * Q^2` の proxy で選ぶ

### 実装要約
このファイルは実機の形状明細から抵抗をその場で計算していない。既に収束した等価抵抗を手入力している。

### 妥当性・設備動作として何を見られるか
- いまのパラメータで流量分配・調整結果がどうなるか
- `a` と `c/d` が別室境界としてどう振る舞うか

### 現状実装の限界
- 幾何情報からの自動生成ではない
- Tee, reducer, hood などの部品別根拠はこのケース定義には残っていない
- 「現状のシミュレーション結果を再現する入力」であり、「設備仕様書そのもの」ではない

## 5. ダンパモデル

### 関連コード
- `ductgraph/control_damper.py`

### 何をしているか
ダンパ角度 `theta` をソルバが使う開度 `u` に変換し、抵抗へ反映している。

### 入力
- 角度 `theta [deg]`
- モデル種別 `sin`, `linear`, `exp`, `expk`
- パラメータ `gamma`

### 出力
- 開度 `u`
- damper 更新済み `Network`

### 計算
- `sin`: `u = sin(theta)^gamma`
- `linear`: `u = (theta/90)^gamma`
- `expk`: 実質的に `K` が指数増加するよう `u` を定義
- 抵抗反映は `r_damper = damper_k / u^2`

### 実装要約
設備的には「角度 -> Cv/K 変化」をかなり単純化している。とはいえ、調整アルゴリズム検証には十分な単調性モデルを与えている。

### 妥当性・設備動作として何を見られるか
- 開けるほど流量が増えるか
- モデル選択で閉止近傍の効きがどれだけ強くなるか

### 現状実装の限界
- 実ダンパの製品曲線ではない
- leakage, hysteresis, authority の詳細までは表現していない

## 6. fan speed 調整

### 関連コード
- `ductgraph/control_speed.py`

### 何をしているか
maxload 端末を基準に fan speed ratio を探索する。

### 入力
- ネットワーク
- maxload 端末 ID
- 設計風量
- `theta_center`, `theta_band`
- `s_min`, `s_max`

### 出力
- `best_s`
- `q65`, `q85`
- 最終 `SolveResult`

### 計算
- maxload 端末の角度を固定し、速度を二分探索
- 速度決定後に `theta_center ± theta_band` で流量を再評価

### 実装要約
maxload を 75deg 近辺で使うという commissioning 発想を、速度探索として実装している。

### 妥当性・設備動作として何を見られるか
- maxload が 75deg 近辺で設計風量を出せるか
- 前後 10deg に authority があるか

### 現状実装の限界
- 真の VFD 制御ではなく、オフライン探索
- `q_design` を1点合わせするだけで、消費電力最適化はしない

## 7. Two-stage commissioning

### 関連コード
- `ductgraph/commissioning.py`
- `tests/nodehead/test_commission_two_stage.py`
- `tests/nodehead/test_commission_tree5_insufficient.py`

### 何をしているか
全端末が設計風量を下回らないように、まず fan speed、次に他端末ダンパを調整する。

### 入力
- maxload 端末
- 他端末 damper 群
- 共通設計風量 `q_design`
- `eps_under_rel`, `tol_conv_rel`, `eps_guard_rel`

### 出力
- `CommissionResult(speed_ratio, thetas, res, ok, msg)`

### 計算
- Stage 1: maxload を `theta_center` 固定で speed 探索
- Stage 2: 他端末を最小開度探索で `Q >= Q_design` に寄せる
- pass 条件は `Q_i >= Q_design * (1 - eps_under_rel)`

### 実装要約
この実装は「不足NG」を最優先にしている。過大風量は warning 側で扱い、合否は下側のみで切っている。

### 妥当性・設備動作として何を見られるか
- 到達不能なら `out.ok=False` にできているか
- 物理的に足りないとき speed が上限張り付きになるか

### 現状実装の限界
- 端末ごと別設計風量ではなく、関数自体は共通 `q_design` 想定
- maxload は自動判定ではなく入力指定

## 8. CAV 同時解と部分運転スケーリング

### 関連コード
- `ductgraph/control_cav.py`
- `ductgraph/commissioning_scale.py`
- `tests/nodehead/test_commissioning_scale_contracts.py`

### 何をしているか
全台運転で決めた `speed_full` を基準に、部分運転ケースへ speed を線形縮尺し、active な CAV 端末の角度を同時に解く。

### 入力
- `full_active_cav_edge_ids`
- `q_design_by_edge`
- `scaling_cases`
- `speed_full`
- `theta_off`

### 出力
- `CommissionScaleResult`
- ケースごとの `ScalingCaseResult`

### 計算
- `speed_ratio_case = speed_full * (Qsum_active / Qsum_full)`
- active 端末のみ Broyden 法で `|Q|-Qtarget` を最小化
- undershoot 時は full-open でも不足かを追加診断

### 実装要約
部分運転の仮定は明確で、「速度は線形スケール」「端末は CAV として追い込む」「不足なら NG」。ここは設備レビューで最も議論しやすい箇所。

### 妥当性・設備動作として何を見られるか
- OFF 端末を閉じないと漏れること
- 線形 speed 縮尺で足りるケース/足りないケース
- 全開でも足りないなら capacity 不足であること

### 現状実装の限界
- 線形スケールは実務近似であり、物理保証ではない
- CAV 解法は数値的には有効だが、実装上は装置の制御シーケンスそのものではない

## 9. 電力計算

### 関連コード
- `ductgraph/power_curve.py`

### 何をしているか
収束した fan 流量から、軸動力・電力を cubic polynomial で推定する。

### 入力
- fan 流量 `Q`
- `speed_ratio`
- 軸動力多項式係数 `a,b,c,d`
- 総合効率 `eta_total`

### 出力
- fan ごとの `FanPowerCurveResult`
- 合計 `PowerCurveResult`

### 計算
- `Ps(Q) = s^3 * P0(Q/s)`
- `Pelec = Pshaft / eta_total`

### 実装要約
流量から電力を引く後段モデルであり、ネットワークソルバとは独立している。

### 妥当性・設備動作として何を見られるか
- ケースごとの電力差
- 速度比変更による動力の増減

### 現状実装の限界
- 効率は簡略モデル
- 実機 motor/inverter 効率マップではない

## 10. 部品ベース損失モデルの現状

### 関連コード
- `ductgraph/ductloss.py`
- `ductgraph/physics/friction.py`
- `ductgraph/physics/loss.py`

### 何をしているか
幾何・粗さ・K 値から `r` を計算する補助関数群を持つ。

### 入力
- 直管長さ、径、粗さ、流量
- local K
- hood の定圧損

### 出力
- solver で使う等価抵抗 `r`

### 計算
- Swamee-Jain による Darcy friction factor
- `dp = (fL/D + ΣK) * rho/(2A^2) * Q^2`
- hood は `r = dp / Q_design^2`

### 実装要約
この層は「将来の部品ベース化」に必要な道具はあるが、`real_case.py` ではまだ本格利用していない。

### 妥当性・設備動作として何を見られるか
- aggregated resistance の代わりに部品積み上げへ移行可能か

### 現状実装の限界
- 実案件入力へ未統合
- mixed diameter を 1 edge に混ぜると制約が出る
- 設計点固定の近似が強い

## 11. テストが保証していること

### 関連コード
- `tests/nodehead/test_commissioning_scale_contracts.py`
- `tests/nodehead/test_commission_two_stage.py`
- `tests/nodehead/test_commission_tree5_insufficient.py`

### 何をしているか
アルゴリズムの「設計思想」を固定している。

### 入力
- 小規模な人工ネットワーク
- 十分な fan / 不十分な fan
- 部分運転ケース

### 出力
- pass/fail 期待
- warning 期待
- speed 張り付き、漏れ低減などの契約確認

### 計算
- 速度線形スケール契約
- OFF 端末閉止の有効性
- 全開でも不足なら `full-open undershoot`
- 不足 fan なら `out.ok == False`

### 実装要約
テストは数値完全一致よりも「この系はこう判定すべき」という契約を押さえている。レビュー時には README よりこちらを優先して読むべき。

### 妥当性・設備動作として何を見られるか
- 実装が少なくとも意図どおりの commissioning 判定をしているか

### 現状実装の限界
- 実案件 `real_case` そのものを強く縛るテストはまだ少ない
- 部品ベース `ductloss` の統合テストは薄い

## 12. この実装をシミュレーションとしてレビューするときの論点

### 人間がレビューすべき妥当性
- `real_case.py` の `r` がどの根拠で決まっているか
- `damper_k` と model/gamma が実機の操作感に近いか
- `fan_poly` が実機曲線のどの運転帯を表しているか
- 部分運転の線形 speed スケール仮定が設備的に受け入れられるか

### 設備動作として確認できること
- maxload と他端末の相互干渉
- damper を閉めると他枝に流量が回ること
- fan 能力不足時に「調整でごまかさず NG にする」こと

### 技術的に現状実装で言えること
- 定常流量ネットワーク + 簡略 fan/damper モデルとしては一貫している
- commissioning 判定ロジックはテストで押さえられている
- 部品ベース化への拡張余地はあるが、実案件入力はまだ aggregated resistance 主体

### 現状実装でまだ言えないこと
- 実機忠実な absolute accuracy
- 制御盤シーケンスや時系列応答
- ダクト部品ごとの厳密圧損内訳
- 実機 CAV ダンパの製品固有特性再現

## 13. 結論

`ductgraph_repo` は、現状では「設備シミュレータ」というより「定常ネットワーク計算 + commissioning 判定ロジックの検証器」と捉えるのが正確である。

レビュー可能な中心テーマは次の 4 点:
- 与えた等価抵抗ネットワークに対する流量分配の妥当性
- maxload 基準の fan speed tuning の妥当性
- damper/CAV 調整ロジックが不足 NG を守るか
- 部分運転での線形 speed スケール仮定が許容できるか

逆に、実機絶対値の保証や部品ごとの厳密モデル妥当性をレビューしたい場合は、`cases/real_case.py` を `ductloss.py` / `physics/*` ベースへ移行し、抵抗値の由来をコード上に戻す作業が必要である。
