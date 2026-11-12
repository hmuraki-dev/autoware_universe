# Vissim-CARLA-Autoware: Vissimの信号設置(交差点ごとの信号制御) 実装計画 v1.0

作成日: 2026-11-12

対象ブランチ:

| リポジトリ                                                          | ブランチ                       | 主に変えるもの                                                                 |
| ------------------------------------------------------------------- | ------------------------------ | ------------------------------------------------------------------------------ |
| CARLA(`~/CARLA`)                                                    | `feature/vissim_windows`       | `Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx`、`data/signal_mapping.json`、`util/` のツール、`docs/TRAFFIC_SIGNAL_TODO.md` |
| autoware_universe(`~/autoware.1.9.0/src/universe/autoware_universe`) | `feature/vissim_windows_co-sim` | `vissim_integration/data/signal_mapping.json`、照合ツール、起動手順書・README |

参考:

- `docs/Vissim移植_信号状態publish_信号設置_引き継ぎ.md` の3章(機能B)・4章
- `docs/Vissim_CARLA_Autoware_信号状態publish_実装計画_v1.0.md`(機能A。V7・V8の結果)
- SUMO版の手順: `git show feature/sumo_co-sim:.claude/skills/carla-town-signals/SKILL.md`、
  `git -C ~/CARLA show main:Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`「SUMOの信号(Town01)」
- `~/CARLA` の `Co-Simulation/PTV-Vissim/docs/TRAFFIC_SIGNAL_TODO.md` のタスク4(今のマッピングの作り方)

---

## 0. 目的・スコープ

Vissimの `Town01.inpx` の信号を、**交差点ごとに独立して制御する**構成に作り直す。CARLAの信号36個
(landmark 360〜395)は、それぞれ1つのVissim信号グループ(SG)に属するようにする。CARLAの信号、
Vissimの車、Autoware(Lanelet2)のEGOが、同じ信号に同じ位置で従うようにする。

今の構成は試験用です。Signal Controller(SC)が1つ、SGが2つしかなく、Town01の12交差点が同時に同じ現示で切り替わります。

### 0.1 目標の構成

- **1交差点 = 1 SC(Fixed time)**。Town01はT字路が12か所なので、SCは12個にする。
- **1つのSCにSGは2つ**。SG1は主道路(向かい合う2本の進入路)、SG2は従道路(残りの1本)。
  SUMO版の `add_carla_tls.py` と同じ分け方。
- **信号パターン**: SG1が青 → 黄 → SG2が青 → 黄、の順に回す(主道路が先)。
  青の長さはSUMO版にそろえて42秒にする。黄と全赤は今のinterstageのまま(黄3秒、全赤2秒)とし、周期は94秒になる。
- **1つのlandmarkは、常に1つの(SC, SG)だけに属する**。
- **Vissimの車が止まる位置**(signalHeadの位置)を、EGOが止まる位置(Lanelet2の停止線)に近づける。

### 0.2 スコープ外

- Town01以外のTown(Town05・Town10HD)のVissimネットワーク。Vissim側に `.inpx` がない。
- 歩行者用の信号と横断歩道。SUMO版のjunction 60には横断歩道があるが、Vissimでは扱わない。ただし、既存の横断歩道(link 203・204)の歩行者用信号を別のSGにすることだけは、利用者の要望で扱う(B2.4)。
- 感応制御(actuated)、交差点間のオフセット最適化(系統制御)。
- Linux側の信号同期処理(`simulation_synchronization.py`)の変更。SCが増えても今のコードのまま動く(1章の6)。
- Windows側アダプタ(`PTV-Vissim_windows/`)の変更。
  `VISSIM_GetSignalStates` は全SCの状態を返し、`server.py` はそれをそのままLinux側へ送る(1章の6)。
- Lanelet2地図(`lanelet2_map_with_tl.osm`)の作り直し。機能Aで作ったものをそのまま使う。

---

## 1. 事前調査で分かったこと(2026-11-12)

| #   | 項目                         | 内容 |
| --- | ---------------------------- | ---- |
| 1   | 今の `.inpx`                 | リポジトリの `examples/Town01/Town01.inpx`(最終コミット `40b2628`)は、SC 1個(`no=1`、FIXEDTIME)、SG 2個、signalHead 38個。SCの参照先は、signalHeadの `sg` 属性(38か所)と、SCの中の `sigTmsTabConfig/signalOutputConfigurationElement`(2か所)だけ。 |
| 2   | 今の信号プログラム           | SCの `intSupplyData`(base64のXML)の中身。周期60秒、stage 1(SG1が青)からstage 2(SG2が青)へ切り替える。interstageは2つで、開始は25秒と55秒、長さはどちらも5秒(黄3秒、全赤2秒)。そのため、各SGの青は約25秒。`<sc>` 要素に `id="1"` と `checkSum` 属性がある。 |
| 3   | 主道路と従道路の分け方       | 今のSG2の12信号(361・363・368・370・372・377・378・381・385・387・392・394)は、どれも、その交差点で向かい合う相手のいない1本(従道路)だった。`carla_tl.json` の停止位置の向きで確かめた。**交差点ごとのSG1とSG2の分け方はすでに目標どおり**で、足りないのは「SCが全体で1つ」という点だけ。 |
| 4   | 交差点のまとまり             | SUMO版のtlLogicで見ると、Town01の12交差点の信号は、landmarkの連番3つずつ(360〜362、363〜365、…、393〜395)。CARLA自身のまとまり(`get_group_traffic_lights()`)と一致することを、B0で確かめた。 |
| 5   | signalHeadとCARLAの信号の対応 | 位置で対応付けると、38個すべてが、36信号のどれかに2.7〜8.6 mの距離で対応した。36信号すべてにsignalHeadがある。**372と374には2個ずつ**ある。追加分はsignalHead 37と38で、名前のないlink 203・204の先頭(`pos` 0.13・0.16)に置かれ、どちらもSG1。B0で調べたところ、この2つは**歩行者用の信号**だった(link 203・204は横断歩道。B0の結果)。 |
| 6   | 同期の経路                   | Windows側の `vissim_kernel_session.py` は、`VISSIM_GetSignalStates` が返す全件を `(controller_id, signal_group_id)` 付きで送る。Linux側は `signal_mapping.json` のキー `(SC, SG)` で引くだけなので、SCの数は制限にならない。`signal_mapping.json` はCARLAリポジトリとautoware_universeの2か所にあり、今は同じ内容。 |
| 7   | 停止位置                     | signalHeadから、CARLAの停止位置(`carla_tl.json` の `stops`)までの距離は6.6〜17.8 m。Lanelet2の停止線は、CARLAの停止位置を6 m前に出したものなので、signalHeadとの距離はおよそ1〜12 mになるが、向きはまだ測っていない。V8では、372の赤でVissimの車がEGOと一緒に止まったことは確かめた。停止位置のずれは記録していない。 |
| 8   | 交通量                       | vehicleInputは7か所。B0で正にしたWindows PCの版では、すべて300台/h(計2100台/h)。SUMO版のTown01(入力2か所×600台/h、のちに減らした)より多い。全交差点を交差点ごとの信号にしたあとに投入待ちが出ないかは、B5で確かめる。 |
| 9   | 実際に読まれる `.inpx`      | Windows PC上の `server.py` は、`--vissim-network "..\Vissim\work_muraki\CARLA\Town01.inpx"`(Windowsのローカルファイル)を読む。リポジトリの `.inpx` と同じかどうかは、まだ確かめていない。 |

---

## 2. 方針

### 2.1 `.inpx` はスクリプトで編集する(GUIは確認と調整に使う)

1章の3のとおり、交差点ごとのSG1とSG2の分け方はすでにできている。必要な変更は次の3つで、どれも機械的にできる。

1. SC 1を、同じ形のSC 12個に分ける。
2. 各signalHeadの `sg` 属性を、`1 1`・`1 2` から `<SC番号> <SG番号>` に書き換える。
3. 信号プログラムの青の長さを42秒にする。

12交差点・38 signalHeadをGUIで作り直すと、手作業が多く、間違いも起きやすい。そのため、CARLAリポジトリにスクリプト
`util/split_signal_controllers.py` を作る。

- **SCの雛形はVissimのGUIで作る**。`intSupplyData` には `checkSum` があり、外から書き換えた中身をVissimが受け付けるかはわからない。
  そこで、GUIでSC 1の信号プログラムを目標の時間(1章の2を42秒青に変えたもの)に直して保存し、スクリプトはそのSC要素を複製するだけにする。
  複製先で変えるのは `no` と `name` だけにする。
  - `<sc id="1">` の `id` が `no` と一致している必要があるかは、B2.1の試作(SC 2個)で確かめる。
  - 必要なら、`intSupplyData` の中の `id` だけを書き換えた版も試す。どちらもだめなら、GUIで12個作る(3章のB2の代替案)。
- **XMLは文字列のまま最小限だけ書き換える**。`.inpx` はWindowsで保存したままの改行(CRLF)なので、改行も変えない。ElementTreeで読み書きすると、2.5 MBのファイル全体の書式や属性の順番が変わり、差分が読めなくなる。
  そのため、`signalHead` の `sg="…"` と `signalController` のブロックだけを置き換える。
- **交差点の割り当ては位置から自動で決める**。交差点のまとまりは、CARLAの `get_group_traffic_lights()` を使う(B0の結果。停止位置の距離ではうまく分けられない)。各signalHeadの位置は、`generate_signal_mapping.py` と同じ計算(linkのポリライン上の `pos`、y反転)で求める。
  その位置に最も近いCARLAの信号を探し、その信号が属する交差点(B0で作る、landmark→交差点の表)でSC番号を決める。
  SG番号は今の値を残す(1章の3)。
- 出力は別ファイル(`Town01.inpx` を直接上書きしない)にし、`--dry-run` で表(signalHead → landmark → SC/SG)だけを出せるようにする。

### 2.2 `signal_mapping.json` は既存のツールで作り直す

`util/generate_signal_mapping.py <新しいinpx>` で作り直す。CARLAサーバー(Town01)が必要。

- 出力を2か所に置く。CARLAリポジトリの `data/signal_mapping.json` と、autoware_universeの `vissim_integration/data/signal_mapping.json`。
- 1つのlandmarkに2つ以上の車用signalHeadがあるときは、どれも同じ(SC, SG)でなければならない。照合ツールで検出する(2.3)。
- **歩行者用のsignalHead(歩行者エリアのlink、`isPedArea="true"` の上にあるもの)は、マッピングに入れない**。今の `generate_signal_mapping.py` は区別せずに一番近い車用信号へ対応付けるため、372がSG1とSG2の両方に入ってしまう(B0で確かめた)。B3でツールを直す。

### 2.3 照合ツール `check_vissim_lanelet2_tl_ids.py`(autoware_universe)

SUMO版の `tools/check_sumo_lanelet2_tl_ids.py` に合わせて、`tools/check_vissim_lanelet2_tl_ids.py` を作る。
CARLAとROSがなくても動くようにする。

```text
python3 tools/check_vissim_lanelet2_tl_ids.py SIGNAL_MAPPING.json MAP_with_tl.osm \
    [--carla-json carla_tl.json] [--inpx Town01.inpx] [--group-id-offset N]
```

確かめること:

1. CARLAの全信号(`--carla-json`)が、`signal_mapping.json` とLanelet2の両方にある。
   片方にしかないlandmarkも出す。
2. 1つのlandmarkが、2つ以上の(SC, SG)に入っていない。
3. `--carla-json` があるときは、1つのSCに別の交差点のlandmarkが混ざっていない。1つの交差点が2つのSCに分かれてもいない。
   交差点のまとまりは、`carla_tl.json` の各信号の `group`(CARLAの `get_group_traffic_lights()` のOpenDRIVE ID)を使う。`group` は、B1で `carla_tl_to_lanelet2.py --dump-json` に足す。
   `group` のない古い `carla_tl.json` では、この確認は飛ばして警告を出す(停止位置の距離では分けられない。B0の結果)。
4. `--inpx` があるときは、歩行者用のsignalHead(`isPedArea` のlink上)を除き、各signalHeadを位置で最も近い信号に対応付ける。そのうえで、`--max-distance`(既定10 m)を超えるもの、
   同じlandmarkに対応するのに `sg` が違うもの、`signal_mapping.json` の(SC, SG)と食い違うものを出す。
5. Lanelet2側は、`refers`(信号灯)を共有するregulatory elementを1信号としてまとめる。SUMO版と同じ。

問題がなければ `OK` を出して終了コード0、あれば1。テストは `test/test_check_vissim_lanelet2_tl_ids.py`(pytest)。

### 2.4 停止位置をそろえる

| 誰が止まる              | 何で決まるか                                                                   |
| ----------------------- | ------------------------------------------------------------------------------ |
| Vissimの車(CARLAに映る) | signalHeadの位置(linkの `pos`)。車はその手前で止まる                          |
| EGO(Autoware)           | Lanelet2の停止線(CARLAの停止位置を6 m前に出したもの)                          |
| CARLA自身の車           | CARLAの停止位置(このco-simではCARLAのautopilot車は走らせないので関係しない)   |

- 照合ツールに `--report-stop-offset` を付け、`--inpx` と `--carla-json` から各signalHeadとLanelet2の停止線との距離を出す。
  距離は進む向きに沿って測り、正なら停止線より先(交差点寄り)。
  - Lanelet2の停止線の位置は、`--carla-json` の停止位置に `--stop-line-forward-offset`(既定6)を足して求めるか、
    osmの `ref_line` から読む(`monitor_tl_stop.py` と同じ)。後者のほうが確実なので、osmから読む。
- 目安は、signalHeadが停止線の0〜2 m先にあること(Vissimの車の前端が、停止線のあたりで止まる)。
  外れたものは `split_signal_controllers.py --align-stop-lines` で `pos` を動かすか、GUIで直す。
  - signalHeadはlinkの上になければならない。connectorの上に置かないことと、linkの長さを超えないことを、スクリプトで確かめる。
  - 動かすかどうかは、B4でずれの表を見てから決める。数が少なければGUIで直す。

---

## 3. タスク一覧

| #   | タスク                                                         | 場所                    | 状態 |
| --- | -------------------------------------------------------------- | ----------------------- | ---- |
| B0  | 事前確認(使う `.inpx`、交差点のまとまり、signalHead 37・38)    | Windows・Linux・CARLA   | 完了(2026-11-12) |
| B1  | 照合ツール `check_vissim_lanelet2_tl_ids.py` とテスト         | autoware_universe       | 完了(2026-11-12) |
| B2  | SCの雛形(GUI)と分割スクリプト `split_signal_controllers.py`    | Windows GUI・CARLA      | 完了(2026-11-12。B2.4の歩行者用信号を含む) |
| B3  | `signal_mapping.json` の作り直しと照合                        | CARLA・autoware_universe | 完了(2026-11-12) |
| B4  | 停止位置のずれを測って直す                                    | CARLA・Windows GUI      | 完了(2026-11-12。スクリプトで直した。GUIでの確認はB5) |
| B5  | Vissim単体での確認(交差点ごとの切り替わり、投入待ち)          | Windows                 | 完了(2026-11-12、利用者の確認) |
| B6  | ブリッジ単体での実機確認(Autowareなし)                        | Linux + Windows         | 完了(2026-11-12。目視はB7へ) |
| B7  | Autoware全体での実機確認(機能Aで残った黄の確認を含む)         | Linux + Windows         | 未着手 |
| B8  | ドキュメントとコミット                                        | 両リポジトリ            | 未着手 |

依存関係: B0 → B1・B2(並行してよい) → B3 → B4 → B5 → B6 → B7 → B8。
B4で `pos` を動かしたときは、B3(マッピングの作り直しと照合)をやり直す。

---

## 4. 各タスクの内容

### B0: 事前確認

1. **使う `.inpx` を決める**。Windows PCの `..\Vissim\work_muraki\CARLA\Town01.inpx` とリポジトリの `examples/Town01/Town01.inpx` を比べる
   (WindowsのPowerShellで `Get-FileHash`、Linuxで `sha256sum`)。
   - 違うときは、両方の `signalController`・`signalGroup`・`signalHead` の数と、更新日時を比べて、新しい方を正にする。
   - 正にした方をリポジトリに入れてから作業を始める。元のファイルは `Town01.inpx.before_split` として残す(コミットはしない)。
2. **交差点のまとまり**。CARLA(Town01)に読み取り専用でつなぐ。各信号の `get_group_traffic_lights()` を出し、1章の4(連番3つずつ)と同じかを確かめる。
   - 結果は `carla_tl.json` には入っていない。そこで、`carla_tl_to_lanelet2.py --dump-json` に `group` を足すか、
     照合ツールの距離によるまとめ(2.3の3)だけで済ませるかを、ここで決める。
   - 推奨は距離によるまとめ。CARLAが要らず、`--dump-json` の形も変えずに済む。CARLAの結果は、距離でまとめたものを確かめるのに1回だけ使う。
3. **signalHead 37・38(link 203・204)**。VissimのGUIで、link 203・204が何のlinkかを確かめる。
   - 見るのは、どこから来てどこへ行くか、名前がない理由、372・374の進入路と重なっていないか。
   - 確かめたら、次のどれにするかを決める。
     - (a) 不要なら消す。
     - (b) 372・374の別の車線なら、同じlandmarkのSGにそろえる。37をSG2にする。
     - (c) 別の進入路なら、正しい信号に付け直す。
   - 37が372の進入路にあるのにSG1なら、今の構成でも、Vissimの車の一部がCARLAの372と違う色で動いていることになる。

完了条件: 使う `.inpx` がリポジトリに入っている。交差点の表(landmark → 交差点番号1〜12)ができている。37・38の扱いが決まっている。

#### 結果(B0、2026-11-12)

| #   | 項目                 | 結果 |
| --- | -------------------- | ---- |
| 1   | 使う `.inpx`         | **Windows PCの版を正にした**。Windows PC(`C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town01.inpx`)のsha256は `f5d4c23f…1024943` で、リポジトリの版(`f2fdf07b…2950fc2`、`40b2628`)と違った。利用者がWindowsの版でリポジトリを上書きし、CARLAリポジトリ `c5df1a6` でコミットした。違いは、車種・3Dモデルの分布、車両の投入量(7か所すべて300台/h。前は50〜250台/h)、歩行者の投入量(300人/h)、シミュレーション期間(3600秒)、改行(CRLF)。**信号・link・signalHeadの差分はなく**、SC・SG・signalHeadの数、信号プログラム、`checkSum`、歩行者エリアのlink、下の2〜3'の結果は、新しい版でも同じだった。 |
| 2   | 交差点のまとまり     | CARLA(Town01)の `get_group_traffic_lights()` は12組で、landmarkの連番3つずつ(360〜362、…、393〜395)と**完全に一致**した。SUMO版のtlLogicとも同じ。新しく接続したクライアントからは信号が見えなかったので、同期モードのワールドを1回だけtickしてから読んだ。 |
| 2'  | 距離によるまとめ     | **使えない**。信号灯の位置では、同じ交差点の中の距離が最大31.8 m、別の交差点との距離が最小33.5 mで、差がほとんどない。停止位置で比べると、交差点の中が最大47.8 m、交差点どうしが最小17.5 mで、逆転している。40 mの閾値で単連結にまとめると、4つの塊になってしまった。そのため、まとまりはCARLAの `group` を `carla_tl.json` に保存して使う(5章の4)。 |
| 3   | signalHead 37・38    | link 203・204は、Town01でただ2本の**歩行者エリアのlink**(`isPedArea="true"`、長さ9.9 m、幅3.5 m、向きが逆の2本)。場所はCARLA座標のx 332〜342、y≈320.5で、信号372の停止線(y 308.7)と交差点の間で従道路を横切る横断歩道。37・38は歩行者用の信号で、SG1(主道路が青、372が赤)に入っている。従道路の車が止まっている間に歩行者が渡る現示なので、正しい。`pedestrianInput` は2つある。 |
| 3'  | 今のマッピングとの関係 | 今の `generate_signal_mapping.py` を今の `.inpx` にかけ直すと(CARLAの位置には `carla_tl.json` を使用)、37が372に対応付けられ、**372がSG1とSG2の両方に入る**。コミット済みの `signal_mapping.json` は372がSG2だけなので、手で直したものと考えられる。B3で、歩行者エリアのlink上のsignalHeadを除くようにツールを直す。 |

### B1: 照合ツール

- 2.3の仕様で、`tools/check_vissim_lanelet2_tl_ids.py` と `test/test_check_vissim_lanelet2_tl_ids.py` を作る。
- `tools/carla_tl_to_lanelet2.py --dump-json` の各信号に `group`(`get_group_traffic_lights()` のOpenDRIVE ID、昇順)を足す。`--from-json` では、`group` がなくても読めるようにする。
  テストを `test/test_carla_tl_to_lanelet2.py` に足し、CARLA(Town01)で `~/autoware_map/Town01/carla_tl.json` を作り直す(停止位置などほかの値が変わらないことを `diff` で確かめる)。
- テストは小さい合成データで書く(`signal_mapping.json`、osm、`carla_tl.json`、`.inpx` の断片)。
  - 正常
  - 片側にしかない
  - 1 landmarkが2つのSGにある
  - 1つのSCに2つの交差点がある
  - signalHeadの `sg` が食い違う
  - 距離超え
  - `refers` を共有するregulatory element
- まず今の `signal_mapping.json`・`Town01.inpx` にかける。期待する結果はNGで、出るべき指摘は「1つのSCに12交差点」。
  signalHead 37・38は歩行者用(B0)なので、landmarkの確認からは外れる。
- 停止位置の表(`--report-stop-offset`)もここで作り、今の構成のずれを記録する(B4の入力)。

完了条件: `python3 -m pytest test/test_check_vissim_lanelet2_tl_ids.py` が通る。今の構成で、期待どおりのNGが出る。

#### 結果(B1、2026-11-12)

作ったもの・変えたもの(autoware_universe):

| ファイル | 内容 |
| -------- | ---- |
| `tools/check_vissim_lanelet2_tl_ids.py` | 2.3の照合ツール。歩行者用のsignalHead(`isPedArea` のlink上)は、landmarkの確認から外す。そのかわり、「最も近い信号の交差点と同じSCにあるか」を確かめる。`--report-stop-offset` で、停止位置の表を出す(停止線はosmの `ref_line` から読む。目安の範囲は `--stop-offset-range`、既定0〜2 m)。 |
| `test/test_check_vissim_lanelet2_tl_ids.py` | 17件。計画に書いた7つのケースに加え、次を確かめる。`group` のない `carla_tl.json`、`--group-id-offset`、CARLAにだけある信号、車用signalHeadのない信号、歩行者用signalHeadの除外とSCの確認、停止位置の符号(東行き・西行き・停止線の手前)、終了コード。 |
| `tools/carla_tl_to_lanelet2.py` | `--dump-json` の各信号に `group` を足した。地図を作る処理は変えていない。 |
| `test/test_carla_tl_to_lanelet2.py` | 2件を足した。`group` を記録すること(偽の `carla` モジュールで確かめる)と、`group` があっても地図が変わらないこと。 |
| `tools/check_tl_topic_vs_carla.py` | `--carla-json` を足した。メッセージごとに、交差点の全信号が同時に青ならNGとし、交差点の中で色が違うメッセージの数を出す(B6で使う)。 |

`~/autoware_map/Town01/carla_tl.json` は、CARLA(Town01)から読み直して置き換えた。元のファイルは `carla_tl.json.bak_before_group` として残してある。
- 変わったのは `group` が加わったことだけ。36信号の位置・停止位置・actor IDは同じだった。
- 同じデータから `--stop-line-forward-offset 6` で作った地図は、今の `lanelet2_map_with_tl.osm` と(timestampを除いて)同じだった。
- `group` は12組で、B0の結果と同じ。

今の構成(コミット済みの `signal_mapping.json`、CARLAリポジトリ `c5df1a6` の `Town01.inpx`)での結果は、期待どおり **NG**。

```text
Vissim: 36 landmarks in 2 signal groups of 1 signal controllers
Lanelet2: 36 traffic lights in 36 regulatory elements (group id offset 0)
CARLA: 36 traffic lights
CARLA: 12 intersections (groups)
Vissim network: 36 vehicle and 2 pedestrian signal heads
  pedestrian head 37 (link 203): SC 1 SG 1, nearest light 372 (4.1 m, SC [1]) OK
  pedestrian head 38 (link 204): SC 1 SG 1, nearest light 374 (8.6 m, SC [1]) OK
NG signal controllers with lights of several intersections: ['SC 1 (12 intersections)']
```

ほかの確認は通った。結果は次のとおり。
- 36信号は、`signal_mapping.json`・Lanelet2・CARLAのどれにもあり、どれも1つのSGだけに入っている。
- 車用signalHeadは各信号に1個ずつあり、SGはマッピングと一致する。
- 距離はどれも10 m以内。

**停止位置**(`--report-stop-offset`、B4の入力): 車用signalHeadは36個とも、Lanelet2の停止線より**先(交差点寄り)にあり、その距離は+0.55〜+8.99 m**。
目安の0〜2 mに入るのは7個だけで、29個は外れている。

| ずれ | 信号 |
| ---- | ---- |
| 0〜2 m(7個) | 362(+1.94)、368(+1.76)、369(+0.55)、376(+0.79)、386(+1.54)、388(+0.65)、394(+1.26) |
| 2〜4 m(10個) | 363(+2.77)、370(+2.12)、371(+2.82)、372(+3.38)、373(+2.03)、375(+2.72)、378(+2.78)、383(+2.27)、391(+3.76)、392(+3.32) |
| 4 m以上(19個) | 360(+4.70)、361(+6.36)、364(+6.83)、365(+4.45)、366(+8.99)、367(+8.92)、374(+6.40)、377(+4.40)、379(+5.69)、380(+5.51)、381(+4.24)、382(+4.09)、384(+5.53)、385(+5.71)、387(+7.40)、389(+5.77)、390(+7.12)、393(+5.45)、395(+7.21) |

- signalHeadの多くは、交差点の手前のlinkの始め(`pos` 1〜3 m)、または長いlinkの終わりにある。そのため、Vissimの車はEGOより交差点寄りで止まる。
- V8で使った372では、EGOは停止線の0.5 m手前で止まった。Vissimの車は、それより約3.4 m先(signalHeadの位置)で止まることになる。

### B2: SCの雛形と分割スクリプト

**B2.1 雛形と試作(Windows GUI + Linux)**

1. VissimのGUIで、使う `.inpx` のSC 1を開く。stage programの青を42秒に変える(interstageの開始を42秒と89秒、周期を94秒にする)。
   黄3秒と全赤2秒は変えない。保存する。
2. 保存した `.inpx` をLinuxへコピーする。`split_signal_controllers.py` の最初の版でSCを2個にしたファイルを作る。
   - 交差点1(360〜362)をSC 1、ほかの11交差点をSC 2にする。
3. それをWindowsのVissim GUIで開き、次を確かめる。
   - エラーなく開ける。
   - SC 2の信号プログラムがSC 1と同じ。
   - Simulation runでSC 1とSC 2が独立して動く(信号時間表で確認)。
   - 確かめられたら、`intSupplyData` の `id` を書き換える必要があるかどうかが決まる。

**B2.1 試作ファイルと手順(2026-11-12)**

順番を変えた。先に、スクリプト(B2.2)で、雛形を今のSC 1のままにした試作ファイルを作る。
GUIで時間を直す作業は、試作Aの上で行い、SCが独立して動くことの確認とまとめる。

試作ファイル(CARLAリポジトリ `Co-Simulation/PTV-Vissim/examples/Town01/trial/`、コミットしない):

| ファイル | 内容 |
| -------- | ---- |
| `Town01_trialA_sc2.inpx` | SC 2個。交差点1(360〜362)がSC 1、ほかの11交差点がSC 2。SC 2の `intSupplyData` はSC 1のコピーそのまま(中の `id="1"` もそのまま) |
| `Town01_trialB_sc2_supplyid.inpx` | Aと同じだが、SC 2の `intSupplyData` の中の `<sc id>` を2に書き換えた(`checkSum` はそのまま)。Aが開けなかったときに試す |
| `Town01_trialC_sc12.inpx` | SC 12個(本番の形。信号プログラムはまだ周期60秒のまま)。仮のマッピング(歩行者用を除いて位置で作ったもの)で、照合ツールがOKになることを確かめてある |

手順(Windows PC):

1. 3つのファイルを、`C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\`(`Town01.inpx` と同じフォルダ)にコピーする。`#data#` の相対パスをそろえるため。
2. VissimのGUIで `Town01_trialA_sc2.inpx` を開く。確かめることは次のとおり。
   - エラーや警告のダイアログが出ない。
   - Signal Controllersの一覧に2つある(1: `J1 360-362`、2: `J2 11 intersections`)。
   - SC 2の信号制御エディタを開くと、SC 1と同じ信号プログラム(周期60秒)が見える。
   - 開けなかった場合は、`Town01_trialB_sc2_supplyid.inpx` で同じことを試す。どちらもだめなら、GUIで作る方法に切り替える(B2の代替案)。
3. 開けたファイルで、**SC 1だけ**、信号プログラムを目標の時間にする。
   - 周期を94秒にする。
   - interstage 1の開始を42秒、interstage 2の開始を89秒にする。
   - interstageの中身(黄3秒、全赤2秒)は変えない。
   - これで、SG1の青は0〜42秒、SG2の青は47〜89秒になる。
4. Simulation runで200秒ほど回し、信号時間表(Signal Times Table)か画面で、次を確かめる。
   - SC 1(周期94秒)とSC 2(周期60秒)が、別々に切り替わる。
   - 交差点1では、主道路(360・362)と従道路(361)が交互に青になる。
5. SC 1を直したファイルを、`Town01_template_sc1_94s.inpx` として保存する。
   - 保存したファイルをLinuxの `Co-Simulation/PTV-Vissim/examples/Town01/trial/` へコピーする。
   - 本番(B2.3)では、このファイルのSC 1を雛形にする(`--template-from`)。
6. 結果を伝える。伝えることは、AとBのどちらで開けたか、出たメッセージ、SCが独立して切り替わったかどうか。

結果(2026-11-12、Vissim 2026、Windows):
- 手順2: **試作Aが開けた**(利用者が確認)。SCは2つあり、どちらも同じ制御だった。
  `intSupplyData` をそのまま複製したSC(中の `id="1"` と `checkSum` も同じ)を、Vissimが受け付けることがわかった。
  そのため、`--renumber-supply-id`(試作B)は要らない。
- 手順3・5: 利用者がSC 1の信号プログラムを周期94秒にした。保存先は別名ではなく、`Town01_trialA_sc2.inpx` への上書きだった。
  - 中身を確かめた結果、SC 1は周期94秒・interstageの開始42秒と89秒、SC 2は元のまま(60秒)。
  - `checkSum` はVissimが計算し直していた(3025973488 → 1200156291)。
  - **SC以外にも、保存で2か所が変わっていた**。`netPara` の `drivSimActive`(true → false)と、vehicleInput 7か所の投入量(300 → 100台/h)。
    本番は、コミット済みの `Town01.inpx` を元にし、このファイルからはSC 1(`--template-from`)だけを取るので、この2つは本番に入らない。
    `drivSimActive=false` だとDriving Simulator Interfaceが使えない可能性があるため、本番では元のtrueのままにする。投入量はB5で決める。
- 手順4(SCが別々に切り替わるか): 未確認。

**B2.3 本番(SC 12個)**: `trial/Town01_sc12_94s.inpx` を作った(2026-11-12)。

```bash
cd ~/CARLA/Co-Simulation/PTV-Vissim
python3 util/split_signal_controllers.py examples/Town01/Town01.inpx \
    --carla-json ~/autoware_map/Town01/carla_tl.json \
    --template-from examples/Town01/trial/Town01_trialA_sc2.inpx --template-sc 1 \
    -o examples/Town01/trial/Town01_sc12_94s.inpx
```

- SCは12個(`J1 360-362` 〜 `J12 393-395`)で、どれも周期94秒、interstageの開始は42秒・89秒。
- SC以外は元の `Town01.inpx` と同じ(`drivSimActive=true`、投入量300台/h、CRLF)。
- 仮のマッピング(歩行者用を除き、位置で作ったもの)で照合ツールにかけると、**OK**(SC 12個、SG 24個、36 landmark、歩行者用の37・38はSC 5)。
- WindowsのVissim GUIでの確認(利用者、2026-11-12): SCは12個。各交差点で、主道路と従道路が交互に青になる。
- 投入量の100台/h(試作Aの保存で変わっていたもの)は、**利用者が意図して減らしたもの**だった。本番にも入れる(B2.4と一緒に行う)。
- `drivSimActive` がfalseになったのは、GUIでSimulation runを回すため(trueのままだと、Driving Simulatorからの接続を待つ)と考えられる。
  co-simで使うファイルは、trueに戻してから使う。

**B2.4 歩行者用信号を別のSGにする(利用者の要望、2026-11-12)**

今は、横断歩道(link 203・204)の歩行者用signalHead 37・38が、車用と同じSG1に入っている(SC 5)。そのため、歩行者用の信号も車用と同時に黄・赤になる。
要望は、「車用が黄になる前に、歩行者用が青点滅になり、車用より早く赤になる」こと。0.2のスコープ外としていた歩行者用信号のうち、この1か所だけを扱う。

- SC 5(`J5 372-374`)に、歩行者用のSG3を足す。
  - 信号の順序(signal sequence)は、赤 → 青 → 青点滅 → 赤。
  - 青はSG1と同時に始め、SG1の青が終わる(42秒)より前に、青点滅を経て赤にする。
  - 推奨: 青0〜34秒、青点滅34〜38秒(4秒)、38秒から赤。横断歩道の長さは9.9 m。赤になってから従道路(372)が青になる47秒まで、9秒ある。
- signalHead 37・38の `sg` を `5 3` にする。
- 作業はVissimのGUIで行う(SGの追加と信号の順序は、`intSupplyData` の中にあり、`checkSum` があるため)。
  - 元にするのは `Town01_sc12_94s.inpx`。投入量の100台/hへの変更も、同じファイルで行う。
  - 保存すると `drivSimActive` がfalseになるので、Linux側でtrueに戻し、ほかに差分がないことを確かめる。
- Linux側への影響:
  - CARLAには歩行者用の信号がない。SG3は `signal_mapping.json` に入らない(B3で直す生成ツールは、歩行者用を除く)。
  - 同期処理はマッピングにある(SC, SG)だけを読むので、SG3の状態は使われない。
  - 照合ツールは、歩行者用signalHeadについて「同じ交差点のSC(5)にあるか」だけを見るので、SG3でもOK。

結果(B2.4、2026-11-12): 利用者がVisSigで設定し、`trial/Town01_sc12_94s_ped.inpx` として保存した。

- SC 5に、SG3(`Red-Green-Flashing Green`)を足した。最小時間は、赤1秒・青0秒・青点滅4秒。
  - 青の最小時間を5秒のままにすると、interstage 2で「minimum green time violated」になった。VisSigは、interstageの終わりに始まる青の最小時間を、interstageの中で数えるため。SG1も0なので、そろえた。
- 中間時間の行列で、SG1とSG3の組を空欄にした(同時に青になる、交差しない組)。
  - 0を入れたままだと「intergreen violated」になる。0も「交差する」という意味になるため。
  - SG2とSG3の組は、0のまま(交差する)。
- interstage 1(Stage 1→2)は、13秒にした。内訳は次のとおり。
  - SG1: 8秒まで青、黄3秒、11秒から赤
  - SG2: 13秒から青
  - SG3: 0〜4秒が青点滅、4秒から赤
- interstage 2では、SG3を5秒から青にした(SG1と同時)。
- 信号プログラムでは、interstage 1の開始を34秒にした(周期94秒、interstage 2は89秒のまま)。
- 結果として、SC 5は次の時間で動く。ほかの11個のSCは変えていない。

  | SG | 青 | 黄・青点滅 | 赤 |
  | -- | -- | ---------- | -- |
  | SG1(主道路) | 0〜42秒 | 黄42〜45秒 | 45〜94秒 |
  | SG3(歩行者) | 0〜34秒 | 青点滅34〜38秒 | 38〜94秒 |
  | SG2(従道路・372) | 47〜89秒 | 黄89〜92秒 | 92〜47秒 |

- signalHead 37・38を `5 3` にした。
- 投入量は、車両7か所を100台/h、歩行者2か所を100人/hにした(利用者の判断)。
- 保存したファイルとSC 12個版(`Town01_sc12_94s.inpx`)の差分は、上のSC 5、signalHead 37・38、投入量、`drivSimActive`(false)だけだった。
  ほかの11個のSCの `intSupplyData` は同じだった。

**B2の成果物**: `drivSimActive` をtrueに戻したものを、`examples/Town01/Town01.inpx` に置いた。
- 照合ツールでは、仮のマッピング(歩行者用を除き、位置で作ったもの)で **OK**。SC 12個・SG 24個・36 landmark、歩行者用の37・38はSC 5のSG3。
- `signal_mapping.json` は、まだ前の構成(SC 1)のまま。B3で作り直すまで、co-simでの信号同期は正しく動かない。

**B2.2 スクリプト(CARLAリポジトリ `Co-Simulation/PTV-Vissim/util/split_signal_controllers.py`)**

```text
python3 util/split_signal_controllers.py IN.inpx --carla-json ~/autoware_map/Town01/carla_tl.json \
    -o OUT.inpx [--template-sc 1] [--template-from INPX] [--first-intersections K] \
    [--renumber-supply-id] [--dry-run]
```

作ったもの(2026-11-12): `util/split_signal_controllers.py` と `util/split_signal_controllers_test.py`。
- `--offset` と `--align-stop-lines` は、まだ入れていない。オフセットは0のまま(5章の2)とし、停止位置はB4で決める。
- 試作のために、次の2つを足した。
  - `--first-intersections K`: 先頭のK交差点だけに自分のSCを持たせ、残りはSC K+1にまとめる。
  - `--template-from`: 雛形のSCを、GUIで保存した別の `.inpx` から取る。
- 雛形のSCを複製するときに変えるのは、`no`、`name`(`J<番号> <最小landmark>-<最大landmark>`)、`sigTmsTabConfig` のSC番号だけ。
- 改行は、編集する側のファイルに合わせる(今はCRLF)。
- signalHeadで変えるのは `sg` だけ。歩行者用(37・38)は、最も近い信号の交差点のSCに入れる。
- テストでは、次を確かめる。
  - SCの複製と、signalHeadの書き換え
  - 編集していない部分が変わらないこと(CRLFを含む)
  - `--renumber-supply-id`、`--first-intersections`、`--template-from`
  - エラーになる場合(`group` がない、距離が遠すぎる、1つの信号が2つのSGに入る、入力ファイルを上書きしようとする)

- 交差点(`carla_tl.json` の `group`)ごとに、SC番号を1から振る。順番は、landmarkの小さい順。
- 各SCの `name` は、そこに含まれるlandmark(例: `J1 360-362`)にする。
- 雛形SCを複製し、`no`・`name`・`sigTmsTabConfig` の `sg` を書き換える。`--offset` で全SCに同じオフセットを入れられるようにする(既定0)。
- signalHeadの `sg` を `<SC> <元のSG>` に書き換える。歩行者用のsignalHead(37・38)も、最も近い信号の交差点のSCに入れ、SGは元のまま(SG1)にする。
  歩行者用は、交差点を決めるのにだけ位置を使い、landmarkの重複の確認からは外す。
- 書き換え後、同じlandmarkのsignalHeadが別の(SC, SG)になっていたら、止まる。
- `--dry-run` は、signalHead → landmark → (SC, SG) と距離の表だけを出す。
- テストは `util/split_signal_controllers_test.py`。既存の `*_test.py` と同じく、スクリプトとして実行する形にする。
  - 小さい `.inpx` の断片で次を確かめる: 文字列の置き換え、SCの複製、SGの決め方。
  - あわせて、書き換えていない部分のバイト列が変わらないことも確かめる。

**B2.3 本番(SC 12個)**

- `--dry-run` の表を確かめてから、`Town01.inpx` を作る。
- Windows GUIで開いて、次を確かめる。
  - SCが12個ある。
  - 交差点ごとにSG1・SG2が交互に切り替わる。
  - 警告がない。

完了条件: SC 12個の `Town01.inpx` が、Windows GUIで開けて動く。

代替案: B2.1で、複製したSCをVissimが受け付けなかった場合は、GUIでSCを12個作る。
- 雛形SCのコピー機能があれば、それを使う。
- signalHeadの `sg` の書き換えだけは、スクリプトで行う。

### B3: `signal_mapping.json` の作り直しと照合

1. B2の `.inpx` をLinuxのリポジトリ(`examples/Town01/Town01.inpx`)に置く。
2. `generate_signal_mapping.py` を直す。歩行者エリアのlink(`isPedArea="true"`)上のsignalHeadは、マッピングに入れない(何個除いたかはログに出す)。
   スタブテスト(`util/signal_sync_stub_test.py` と同じ形)を足す。直したあと、今の `.inpx` にかけると、コミット済みの `signal_mapping.json` と同じ内容になることを確かめる。
3. CARLA(Town01)を起動し、直した `generate_signal_mapping.py` を新しい `.inpx` にかける。
   `python3 util/generate_signal_mapping.py examples/Town01/Town01.inpx --output data/signal_mapping.json`。
   - `--max-distance` を超えた候補は、目視で確かめる。
4. 出力をautoware_universeの `vissim_integration/data/signal_mapping.json` にコピーし、2つが同じであることを `diff` で確かめる。
5. 照合ツールを実行する。
   `python3 tools/check_vissim_lanelet2_tl_ids.py <signal_mapping.json> ~/autoware_map/Town01/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town01/carla_tl.json --inpx <Town01.inpx>`。
   `OK` が出ること。
6. Vissim用の既存テスト(`util/signal_sync_stub_test.py`、autoware_universeの `test/vissim_*_test.py`)を実行し、
   `signal_mapping.json` の形に依存した失敗がないことを確かめる。

完了条件: 照合ツールで `OK` が出る。期待するマッピングの形は「`signal_mapping.json` にSCが12個、各SCにSGが2個、36 landmarkがすべて1回ずつ」。

#### 結果(B3、2026-11-12)

`util/generate_signal_mapping.py`(CARLAリポジトリ)の変更:
- 歩行者エリアのlink(`isPedArea="true"`)上のsignalHeadを除く。除いたものはログに出す。
- 1つの信号が2つ以上の(SC, SG)に入るマッピングは、書き出さずに終了コード1で止まる。
- `--carla-json` を足した(CARLAサーバーのかわりに `carla_tl.json` を読む)。`carla` は、サーバーにつなぐときだけimportする。
- CARLAが同期モードで信号が見えないときは、1回だけtickする(`carla_tl_to_lanelet2.py` と同じ)。
  ブリッジを止めたあとのCARLAでは、誰もtickしないので、新しく接続したクライアントからは信号が0個に見えた。
- SCとSGを番号順に並べて書き出す(実行するたびに同じファイルになる)。
- テストを足した: `util/generate_signal_mapping_test.py`(歩行者用の除外、2つのSGに入る信号での停止、`--carla-json`)。

直したツールを前の `Town01.inpx`(SC 1つ、`c5df1a6`)にかけると、コミット済みの `signal_mapping.json`(手で直してあったもの)と同じになった。

新しい `Town01.inpx`(`0e45900`)から、CARLA(Town01)につないで作り直した結果は次のとおり。`--carla-json` で作ったものと同じだった。

| SC | SG1(主道路) | SG2(従道路) |
| -- | ----------- | ----------- |
| 1 | 360, 362 | 361 |
| 2 | 364, 365 | 363 |
| 3 | 366, 367 | 368 |
| 4 | 369, 371 | 370 |
| 5 | 373, 374 | 372 |
| 6 | 375, 376 | 377 |
| 7 | 379, 380 | 378 |
| 8 | 382, 383 | 381 |
| 9 | 384, 386 | 385 |
| 10 | 388, 389 | 387 |
| 11 | 390, 391 | 392 |
| 12 | 393, 395 | 394 |

- SC 5のSG3(歩行者用)は、マッピングに入らない。
- 2か所(CARLAの `data/signal_mapping.json`、autoware_universeの `vissim_integration/data/signal_mapping.json`)は同じ内容。
- 照合ツール(`--carla-json`・`--inpx` 付き)の結果は **OK**。

既存のテスト:
- `util/signal_sync_stub_test.py` は、前のマッピング(SC 1、SG1に10個)を書き込んでいたので落ちた。
  期待値を実際の `signal_mapping.json` から作るように書き直した。あわせて、1つのSCだけが切り替わり、ほかのSCの信号はそのままであること(交差点ごとの独立)も確かめるようにした。
  書き直したテストは、新旧どちらのマッピングでも通る。
- CARLAリポジトリの `util/*_test.py` 8本、autoware_universeの `test/test_*.py`(89件)と `test/vissim_*_test.py` 8本は、すべて通った。

### B4: 停止位置のずれを測って直す

1. 照合ツールの `--report-stop-offset` で、36信号(車用のsignalHead 36個。歩行者用の37・38は除く)のずれの表を作る。
2. 2.4の目安(停止線の0〜2 m先)から外れるsignalHeadについて、`pos` を動かすかを決める。
   - 動かす方法は、`--align-stop-lines` か、GUIのどちらか。
   - linkの上にない場合や、linkの長さを超える場合は、GUIでlinkの区切りを見て直す。
3. 動かしたら、B3の3〜5をやり直す。
4. 表(変える前、変えた後)を、計画書のB4の結果に残す。

完了条件: 全signalHeadがおおむね目安の範囲に入っている。外れたものは理由を書く。

#### 結果(B4、2026-11-12)

利用者の要望は、「EGOもVissimの車も、停止線から異常に離れて止まらないようにする」こと。基準は、**交差点の入口**(Lanelet2の進入路laneletの終わり)にした。

**測った結果(直す前、交差点の入口からの距離。マイナスは入口の手前)**:
- EGOの停止線(Lanelet2、CARLAの停止位置+6 m)は、-0.96〜-7.08 m。366(-7.08)・367(-6.55)・387(-6.25)は、6 m以上手前で止まる。
- Vissimの車用signalHeadは、-1.57〜**+3.76 m**。18個が交差点の中にあった。
- **signalHeadの多くが、交差点の中の1つの進路のlinkの上にあった**。そのため、同じ進入路からほかの進路へ行く車は、赤でも止まらなかった。
  - 進入路のlinkから2つの進路に分かれるのに、片方にしかsignalHeadがない進入路は8本あった(361・372・377・384・385・387・390・391)。
  - V8で見た、EGOの前をふさいだ左折のバスも、これで赤を通った可能性がある。
- 368のsignalHead 9は、交差点から**出ていく側**のconnector(link 191→50)の上にあった。368の進入路(link 49)の車は、信号で止まっていなかった。

**直し方**:
1. Lanelet2の停止線: `tools/carla_tl_to_lanelet2.py` に `--stop-line-before-junction D` を足した。
   - 停止線を、信号ごとに「laneletの終わりからD m手前」に置く。`--stop-line-forward-offset` のように全信号を同じ量だけずらす方法とは違う。
   - D=1.0で `~/autoware_map/Town01/lanelet2_map_with_tl.osm` を作り直した。前の地図(offset 6)は `lanelet2_map_with_tl.osm.bak_offset6` に残してある。
   - CARLAの停止位置から前へずらした量は、5.96〜12.08 mになった。WARNINGはない。
   - テストを2件足した(`test_carla_tl_to_lanelet2.py`)。
2. Vissimのsignalheadの位置: CARLAリポジトリに `util/align_signal_heads.py` を作った。
   - 車用signalHeadを、そのCARLA信号のLanelet2の停止線の中点に最も近い、同じ向きの**進入路のlink**の上に移す。connectorと歩行者エリアのlinkは除く。
   - そのlinkから出るconnectorの開始位置より、`--connector-margin`(0.2 m)手前に収める。
     - これを入れたのは、366で、停止線(68.63 m)が、connectorの開始位置(68.60 m)より先だったため。
   - 変えるのはsignalHeadの `lane` と `pos` だけ(文字列のまま置き換え、CRLFは変えない)。
   - テストは `util/align_signal_heads_test.py`。

**直した後**:
- 36個とも、進入路のlinkに移り、その進入路の全進路(connector 2本)の手前にある。横ずれはどれも0.00 m。
- 照合ツール(`--report-stop-offset --stop-offset-range -0.5 0.5`)の結果は **OK**。
  - signalHeadとLanelet2の停止線の距離は、-0.23〜0.00 m。0でないのは、connectorの手前に収めた6個(366:-0.23、ほかは-0.09以下)だけ。
- `signal_mapping.json` は作り直しても変わらなかった。
- 想定する停止位置は、EGOもVissimの車も、前端が交差点の入口の約1 m手前(EGOは停止線の手前0.5 m前後。V8の実績)。
- 停止線の位置が変わったので、起動手順書2.7の地図の作り方(`--stop-line-forward-offset 6`)は、B8で `--stop-line-before-junction 1.0` に書き換える。
- 実機(Vissim GUI、co-sim)での確認は、B5〜B7で行う。

### B5: Vissim単体での確認(Windows)

1. Windows PCの `server.py` が読む `Town01.inpx` を、B3(またはB4)の版に置き換える。元のファイルは残す。
2. VissimのGUIで1800秒回し、次を見る。
   - 交差点ごとに、主道路と従道路が交互に青になる。
   - 交差道路が同時に青にならない。
   - Vehicle Input の投入待ち(vehicle network performanceの `VehDelay`・`VehLatent` などの評価、または目視)。
   - 渋滞で、交差点の中に車が詰まったまま動かなくならないか。
3. 投入待ちが増え続けるときは、vehicleInputの交通量を下げる(SUMO版のTown01では、主な入力を半分にした)。
   - 下げた値と理由を残す。

完了条件: 1800秒で投入待ちが増え続けない。交差点で詰まらない。

#### 結果(B5、2026-11-12)

- 利用者がWindowsのVissim GUIで、B4の `Town01.inpx`(CARLAリポジトリ `4213552`)を確かめ、「問題ない」との回答だった。
  - 主に見たのは、signalHeadの位置(372・373・368の進入路。位置を図で示して確かめた)。
  - 372は、交差点の中(link 93)から進入路(link 39、横断歩道の手前)に移った。
  - 368は、交差点から出ていく側の車線(connector 10226)から、入ってくる側の車線(link 49)に移った。
- 投入量は、B2.4で決めた値(車両7か所100台/h、歩行者2か所100人/h)のまま。変えていない。

### B6: ブリッジ単体での実機確認(Autowareなし)

#### B6.0 準備

| 端末              | やること |
| ----------------- | -------- |
| Windows           | `C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town01.inpx` が、CARLAリポジトリ `4213552` の版になっていることを確かめる。B5でGUIから保存した場合は、`drivSimActive` がfalseになっているので、コピーし直す。そのあと、起動手順書2.2のとおりアダプタ(`server.py`)を起動し直す |
| Linux ターミナル1 | 起動手順書2.3のとおりCARLAを起動する |
| Linux ターミナル2 | ブリッジ |
| Linux ターミナル3 | 確認用スクリプト |

ターミナル2・3では、最初に次を実行する(機能AのV7.0と同じ)。

```bash
source /opt/ros/humble/setup.bash
source ~/autoware.1.9.0/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=33
VISSIM_HOST=192.168.16.56
MAP_DIR=$HOME/autoware_map/Town01
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
```

- ビルドは不要。installの `signal_mapping.json` は、ソースへのシンボリックリンクになっている(B3で更新済み)。
- `lanelet2_map_with_tl.osm` はB4で作り直したが、信号灯の位置は変わっていない。そのため、ブリッジの位置マッチングの結果は変わらないはず。

#### B6.1 起動(ターミナル2)

```bash
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml \
  map_path:=$MAP_DIR use_vissim:=true vissim_adapter_host:=$VISSIM_HOST vissim_adapter_port:=5555 \
  vissim_connect_timeout_ms:=120000 vissim_sim_period:=600 spectator_follow:=true \
  spawn_point:=334.74,263.00,0.30,0.0,0.0,90.0 sync_traffic_lights:=true \
  traffic_light.publish:=true traffic_light.map_path:=$MAP_DIR/lanelet2_map_with_tl.osm
```

- ブリッジのログに `Traffic-light matching: 36 matched, 0 ambiguous, 0 too far` が出ることを確かめる。

#### B6.2 確認(ターミナル3)

```bash
python3 tools/check_tl_topic_vs_carla.py --duration 300 --carla-json $MAP_DIR/carla_tl.json
```

- `--duration` は実時間。Vissimの周期は94秒で、co-simは実時間より遅くなることがある。そのため、1周以上入るように300秒にした。

| 項目 | 期待 |
| ---- | ---- |
| `publishers on the topic` | `1 ['/autoware_carla_interface']` |
| `groups per message` | `{36: N}` |
| `match (same tick)` | ほぼ全件。`mismatch` は0 |
| `colors published` / `color transitions` | `RED`・`AMBER`・`GREEN` があり、`GREEN->AMBER`・`AMBER->RED`・`RED->GREEN` がある |
| `intersections` | `12`。`messages with differing colors inside an intersection` がほぼ全件(主道路と従道路が別の色になる) |
| `every light of an intersection green at once` | 出ない(交差点の全信号が同時に青にならない) |
| 最後の行 | `OK` |

目視(CARLAの画面。スペクテーターはEGOを追い、EGOは信号372の停止線の手前にいる):
- 372が赤の間、372の進入路のVissimの車が、左折も右折も停止線の手前で止まる。前は、交差点の中の1つの進路にしかsignalHeadがなかった。
- 止まる位置は、交差点の入口の手前(EGOの停止線とほぼ同じ位置)。
- 372が赤の間、交差する主道路(373・374)の車が通る。

#### B6.3 終了後(ブリッジをCtrl+Cで止めた後)

```bash
python3 tools/check_tl_topic_vs_carla.py --frozen-only
```

- 期待は `CARLA traffic lights: 36, frozen: 0` と `OK`。

#### 結果(B6、2026-11-12、CARLA 0.9.15、Town01、Vissim 2026(Windows)、ブリッジ単体)

- B6.1: `Traffic-light matching: 36 matched, 0 ambiguous, 0 too far, 0 overridden (map has 36 heads / 36 groups)`。
  - B4で作り直した地図でも、マッチングの結果は変わらない。
  - アダプタのログには、`25 signal(s)`(12交差点×SG 2個+歩行者用SG3)が出ていた。新しい `Town01.inpx` が使われている。
- B6.2: `OK`。

```text
CARLA ticks seen          : 2291
messages received         : 2288
publishers on the topic   : 1 ['/autoware_carla_interface']
groups per message        : {36: 2288}
match (same tick)         : 2288 / 2288   1 tick behind: 0   mismatch: 0
stamp vs CARLA elapsed    : offset 399.989 s, spread 0.000 s
colors published (count)  : {'GREEN': 37248, 'RED': 41520, 'AMBER': 3600}
color transitions         : {'GREEN->AMBER': 60, 'AMBER->RED': 60, 'RED->GREEN': 48}
intersections             : 12, messages with differing colors inside an intersection: 2168 / 2288
OK
```

  - 2288メッセージは、シミュレーション時間で約114秒にあたる。周期94秒の1周と少し。実時間300秒に対して約0.4倍で、これまでのVissim連携と同じ傾向。
  - 交差点の中で色がそろったメッセージは120件。全赤(45〜47秒、92〜94秒)の時間と合う。12交差点はオフセット0なので、全赤も同時に来る。
  - 「交差点の全信号が同時に青」は出なかった。
- B6.3: `CARLA traffic lights: 36, frozen: 0`、`OK`。
- 目視(Vissimの車が、372の赤で左折も右折も止まるか)は、報告を受けていない。B7の2で見る。
- 1回目の起動で、connectが120秒でタイムアウトした。原因は、Vissim側でDriving Simulatorが無効になっていたことと、Vissimが複数起動していたこと(利用者の確認)。今回の変更とは関係しない。
- B6.1を待つ間に、利用者が「Autowareが起動しない」と思ってブリッジを止めた。B6はブリッジだけを起動する手順で、Autoware本体はB7で起動する。手順書に書くときは、このことを明記する。

### B7: Autoware全体での実機確認

機能AのV8と同じシナリオで確かめる。
- 起動コマンド: V8.1
- スポーン地点: `334.74,263.00,0.30,0.0,0.0,90.0`
- 信号: 372(交差点 372〜374の従道路)
- 監視: `monitor_tl_stop.py --group 372`

| #   | 確認                                         | 見るもの |
| --- | -------------------------------------------- | -------- |
| 1   | 赤で停止線に止まり、青で発進する            | `monitor_tl_stop.py` の停止位置(V8では0.5 m手前)と、赤の通過0回 |
| 2   | Vissimの車とEGOが同じ信号で、近い位置に止まる | CARLAの画面で、372の進入路のVissimの車の先頭と停止線の距離。B4の表と比べる |
| 3   | 交差する主道路(373・374)が、372が赤の間に青 | 主道路をVissimの車が通る。V8では全交差点が同時だったので、主道路と従道路の関係は確かめていない |
| 4   | 走行中の黄(機能Aで残った項目)               | 黄になったときのEGOの動き。止まるか通過するかと、そのときの停止線までの距離と速度。`monitor_tl_stop.py` のログで記録する |
| 5   | 目的地まで走れる                            | V8では、Vissimのバイクの追突とバスで止まった。交通量をB5で変えたときは、その影響も見る |

### B8: ドキュメントとコミット

- **CARLAリポジトリ(`feature/vissim_windows`)**
  - `docs/TRAFFIC_SIGNAL_TODO.md`: タスク4に「交差点ごとのSCへの作り直し」を追記する。
    - 中身は、SCと交差点の対応表、信号パターン、再生成のコマンド。
    - 「5交差点・15 signalHead」などの古い記述を、今の内容に直す。
  - `docs/Vissim(win)-CARLA_co-sim_起動手順.md`: Windows PCへ `.inpx` をコピーし直すことと、信号の構成を書く。
  - コミットするもの: `.inpx`、`data/signal_mapping.json`、`util/split_signal_controllers.py` とテスト。
- **autoware_universe(`feature/vissim_windows_co-sim`)**
  - 起動手順書 2.7: 「試験用の信号構成」の注意を、交差点ごとの構成に書き換える(2.6.1のオプション表は変えない)。
    あわせて、`signal_mapping.json` の作り直し手順と照合ツールの使い方を書く。
  - README: Known Issuesから「試験用の構成」を消す。照合ツールを、「Publishing CARLA Traffic-Light States」節に足す。
  - 計画書(本書)に、各タスクの結果を書く。機能Aの計画書の残課題(5章)に、機能Bで対応したことを書く。
  - 引き継ぎメモ(`Vissim移植_信号状態publish_信号設置_引き継ぎ.md`)は、機能A・Bが終わった時点で、次のどちらかにする。
    - コミットして残す。
    - 必要な部分だけ本書に移して消す。
- **コミット**
  - リポジトリごとに分け、タスクの区切りでコミットする。
  - autoware_universeは `feat(autoware_carla_interface): ...` / `docs(vissim-carla-autoware co-sim): ...`、CARLAは今のブランチの書き方(`feat: Vissim(Win)-CARLA(Linux)対応 _...`)にそろえ、どちらも日本語で書く。
  - 実機で確かめた結果は、`docs(...)` のコミットに残す。
- **Windows PCへの反映**: `.inpx` を置き換えたことと、元のファイルの場所を、手順書と計画書の結果に書く。

---

## 5. 決めること(作業の前か途中)

| #   | 内容                                   | 推奨                                         | 決める時期 |
| --- | -------------------------------------- | -------------------------------------------- | ---------- |
| 1   | 青の長さと周期                         | 青42秒・黄3秒・全赤2秒(周期94秒)。SUMO版にそろえ、全赤は今の値を残す | B2.1の前 |
| 2   | 交差点ごとのオフセット                 | 0(SUMO版と同じ)。ずらす必要が出たら、`--offset` を交差点ごとに指定できるようにする | B2.2 |
| 3   | signalHead 37・38の扱い                | 決定(B0): 歩行者用の信号として残す。交差点372〜374のSCのSG1にし、マッピングからは外す | B0 |
| 4   | 交差点のまとめ方                       | 決定(B0): CARLAの `get_group_traffic_lights()`。`carla_tl.json` に `group` として保存する。距離では分けられない | B0 |
| 5   | 停止位置を動かすかどうか              | B4の表を見て、外れが多ければスクリプト、少なければGUI | B4 |

---

## 6. リスクと対応

| リスク                                                             | 対応 |
| ------------------------------------------------------------------ | ---- |
| 複製したSCの `intSupplyData`(`checkSum` 付き)を、Vissimが受け付けない | B2.1で、SC 2個で試す。だめなら `id` を書き換えた版を試し、それもだめならGUIで作る(B2の代替案) |
| スクリプトの書き換えで、`.inpx` のほかの部分が壊れる               | 文字列の最小限の置き換えにする。書き換えていない部分が変わらないことをテストする。Windows GUIで開いて確かめ、元のファイルも残す |
| Windows PCの `.inpx` とリポジトリの `.inpx` が食い違う               | B0で正を決め、作業後はリポジトリの版をWindowsへコピーする。手順書に書く |
| 交差点ごとの信号で、渋滞や投入待ちが増える                         | B5で確かめ、交通量を下げる |
| signalHeadを動かすと、Vissimの車が交差点の中で止まる、または手前すぎる | B4で、linkの上にあることと長さを確かめる。B7で目視する |
| `signal_mapping.json` の2か所が食い違う                              | B3で `diff` する。照合ツールにはautoware_universe側を渡す(Linux側で読まれる方) |
| 主道路と従道路の判定が、交差点の形によっては合わない(Town01以外) | 本計画はTown01だけ。ほかのTownでは、SUMO版の `add_carla_tls.py` と同じ判定(向きの差が135度より大きい2本)をスクリプトに入れる |
