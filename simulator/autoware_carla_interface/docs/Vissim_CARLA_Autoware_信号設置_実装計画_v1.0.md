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
- 歩行者用の信号と横断歩道。SUMO版のjunction 60には横断歩道があるが、Vissimでは扱わない。
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
| B1  | 照合ツール `check_vissim_lanelet2_tl_ids.py` とテスト         | autoware_universe       | 未着手 |
| B2  | SCの雛形(GUI)と分割スクリプト `split_signal_controllers.py`    | Windows GUI・CARLA      | 未着手 |
| B3  | `signal_mapping.json` の作り直しと照合                        | CARLA・autoware_universe | 未着手 |
| B4  | 停止位置のずれを測って直す                                    | CARLA・Windows GUI      | 未着手 |
| B5  | Vissim単体での確認(交差点ごとの切り替わり、投入待ち)          | Windows                 | 未着手 |
| B6  | ブリッジ単体での実機確認(Autowareなし)                        | Linux + Windows         | 未着手 |
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
- まず今の `signal_mapping.json`・`Town01.inpx` にかける。期待する結果はNGで、出るべき指摘は次の2つ。
  - 「1つのSCに12交差点」
  - signalHead 37の食い違い
- 停止位置の表(`--report-stop-offset`)もここで作り、今の構成のずれを記録する(B4の入力)。

完了条件: `python3 -m pytest test/test_check_vissim_lanelet2_tl_ids.py` が通る。今の構成で、期待どおりのNGが出る。

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

**B2.2 スクリプト(CARLAリポジトリ `Co-Simulation/PTV-Vissim/util/split_signal_controllers.py`)**

```text
python3 util/split_signal_controllers.py IN.inpx --carla-json ~/autoware_map/Town01/carla_tl.json \
    -o OUT.inpx [--template-sc 1] [--offset 0] [--align-stop-lines MAP_with_tl.osm] [--dry-run]
```

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

### B4: 停止位置のずれを測って直す

1. 照合ツールの `--report-stop-offset` で、36信号(車用のsignalHead 36個。歩行者用の37・38は除く)のずれの表を作る。
2. 2.4の目安(停止線の0〜2 m先)から外れるsignalHeadについて、`pos` を動かすかを決める。
   - 動かす方法は、`--align-stop-lines` か、GUIのどちらか。
   - linkの上にない場合や、linkの長さを超える場合は、GUIでlinkの区切りを見て直す。
3. 動かしたら、B3の3〜5をやり直す。
4. 表(変える前、変えた後)を、計画書のB4の結果に残す。

完了条件: 全signalHeadがおおむね目安の範囲に入っている。外れたものは理由を書く。

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

### B6: ブリッジ単体での実機確認(Autowareなし)

機能AのV7と同じ準備(`Vissim_CARLA_Autoware_信号状態publish_実装計画_v1.0.md` のV7.0)で、Windows側の `server.py` とブリッジを起動する。
起動するときは、`use_vissim:=true sync_traffic_lights:=true traffic_light.publish:=true lanelet2_map_file:=lanelet2_map_with_tl.osm` を付ける。

| #   | 確認                                         | 見るもの |
| --- | -------------------------------------------- | -------- |
| 1   | マッピングの読み込み                        | ブリッジのログ `Vissim signal group(s) mapped` に24組((1,1)〜(12,2))、`Carla traffic light(s) mapped` に36個。警告なし |
| 2   | 対応付け                                    | `Traffic-light matching: 36 matched, 0 ambiguous, 0 too far` |
| 3   | トピックとCARLAが同じtickで一致             | `tools/check_tl_topic_vs_carla.py --duration 120` が `OK`。V7と同じく、全メッセージが同じtickで一致 |
| 4   | 交差点の中で主道路と従道路が逆の色になる    | 同じスクリプトの出力、またはCARLAの画面で、例えば360・362が青のとき361が赤。各交差点で、3信号すべてが同時に青にならない |
| 5   | 黄が届く                                    | `colors published` にAMBER。遷移 `GREEN->AMBER->RED` |
| 6   | 終了後のfreeze                              | `--frozen-only` で `frozen: 0` |

交差点ごとのチェック(4)を自動でできるように、`check_tl_topic_vs_carla.py` に `--carla-json` を足す。これで交差点ごとに「同時に青」を数えられる。
小さい変更なので、B1と同じコミットにする。

`--offset` が0のときは、12交差点が同じ時刻に切り替わる。SUMO版も全tlLogicのオフセットが0なので、同じ挙動になる。
ただし、交差点の中では主道路と従道路の色が違う。

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
