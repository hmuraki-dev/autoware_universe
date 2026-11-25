# SUMOのシナリオ(SUMO-CARLA-Autoware co-sim)

co-simの`sumo_cfg_file`に渡すSUMOのシナリオ。CARLAの版(0.9.15・0.9.16)によらず、ここのものを使う。

`share/autoware_carla_interface/sumo/`にインストールされ(`CMakeLists.txt`)、`sumo_cfg_file:=Town01.sumocfg`のような相対パスは、
そこから数える(`src/autoware_carla_interface/sumo_integration/sumo_scenarios.py`)。`--symlink-install`ならインストール先はここへのリンクなので、
ファイルを書き換えてもビルドし直さなくてよい(ファイルを足したときはビルドし直す)。

| ファイル | 内容 |
| --- | --- |
| `Town01.sumocfg`・`net/Town01.net.xml`・`rou/Town01.rou.xml` | Town01。CARLAの信号36個に対応する信号、Vissimと同じ車両グループ |
| `Town05.sumocfg`・`net/Town05.net.xml`・`rou/Town05.rou.xml` | Town05。CARLAの信号54個に対応する信号 |
| `Town10HD.sumocfg`・`net/Town10HD.net.xml`・`rou/Town10HD.rou.xml` | Town10HD。CARLAの信号15個に対応する信号 |
| `Town10HD_vissim.sumocfg`・`tll/Town10HD_vissim.tll.xml`・`rou/Town10HD_vissim.rou.xml` | Town10HD(網は`net/Town10HD.net.xml`)を、Vissim構成の`vissim/Town10HD.inpx`と同じ信号・需要・車両寸法で走らせる。下の「inpxから作るシナリオ」 |
| `carlavtypes.rou.xml` | CARLAの車の寸法のvType(0.9.15のCARLAから作ったもの、41種) |
| `vissim_fleet_vtypes.rou.xml` | Town01の車両グループのvType(Vissimの`Town01.inpx`のモデル) |
| `viewsettings.xml` | sumo-guiの表示設定 |

- 2026-11-17に、CARLAリポジトリ(`~/CARLA`)の`main`(`4e22c5e`)の`Co-Simulation/Sumo/examples/`から写した。
- net・ルートを作り直すツール(`add_carla_tls.py`・`make_vissim_fleet_vtypes.py`など)と、その手順書(`Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`)は`~/CARLA`にある。
  `~/CARLA`で作り直したら、ここの同じ相対位置へ写す。
- Town01のroute-files(`carlavtypes.rou.xml`・`vissim_fleet_vtypes.rou.xml`・`rou/Town01.rou.xml`)を変えたら、
  ブループリント表`src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml`を作り直す
  (`docs/SUMO-CARLA-Autoware_co-sim_起動手順.md`の2.2.3)。作り直さないと、co-simの起動時に`variant_source_changed`で止まる。

## inpxから作るシナリオ(`Town10HD_vissim`)

`vissim/Town10HD.inpx`(Vissim構成のTown10HD)から、信号プログラムと交通需要・車両寸法を作り、今の`net/Town10HD.net.xml`と組み合わせる。
設計と確認結果は`docs/SUMO_CARLA_Autoware_inpxからの信号・需要生成_実装計画_v1.0.md`。

| ファイル | 中身 |
| --- | --- |
| `tll/Town10HD_vissim.tll.xml` | 信号5つの新しいプログラム(`programID="vissim"`)。周期94 s(青42・黄3・全赤2 ×2)。`linkSignalID`は網のものを写す |
| `rou/Town10HD_vissim.rou.xml` | 流入19本・合計600台/hから、0〜3600 sに582台。経路は車1台ごとに分岐ごとに等分して選び、2000 m進んだら消える。vTypeは8つ(inpxの日本の車の3Dモデルの寸法) |
| `../src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd_vissim.yaml` | ブループリント表(0.9.16_jp)。Vissim構成の`carla_variants_town10hd.yaml`と同じ割り当て |

- `vissim/Town10HD.inpx`を変えたとき(Vissimで車両設定を直したときなど)は、次の順に作り直す。
  確認の仕方・テストが失敗したときの見方を含む手順は、`docs/SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md`の2.2.3。
  `.rou.xml`を作り直したらブループリント表も作り直す(作り直さないと、co-simの起動時に`variant_source_changed`で止まる)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/sumo_inpx/vissim_to_tll.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \
    --out sumo/tll/Town10HD_vissim.tll.xml   # ← 生成物
python3 tools/sumo_inpx/vissim_to_rou.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \
    --out sumo/rou/Town10HD_vissim.rou.xml   # ← 生成物
python3 tools/carla_fleet/build_carla_variants.py --sumocfg sumo/Town10HD_vissim.sumocfg \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json \
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules_carla0.9.16_jp.yaml \
    --out src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd_vissim.yaml   # ← 生成物
python3 -m pytest test/test_sumo_inpx.py test/test_sumo_fleet_plan.py
```

> 📄 **生成物:** inpxから作った信号・需要と、そのブループリント表\
> `sumo/tll/Town10HD_vissim.tll.xml`、`sumo/rou/Town10HD_vissim.rou.xml`、`src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd_vissim.yaml`

- テストは、作り直した`.tll.xml`・`.rou.xml`がコミットしたものと同じかを確かめる。inpxを変えて作り直したときは、作り直したものをコミットする。
- `vissim_to_rou.py`のオプション: `--route-length`(経路の長さ、既定2000 m)、`--horizon`(車を出す時間、既定3600 s)、`--seed`(乱数の種、既定1)、
  `--report`(書き出さずに流入・車種・台数を表示)。
- Vissimの網もSUMOの網も閉じている(出口が無い)ので、流し続けると台数が増え、SUMO単体では600 sで平均1.3 m/s、860 sごろから詰まり始める。
  Vissimの車は網から出ないので、Vissimでも同じように溜まる見込み(計画書のS3)。
- 新しいファイルを足したときは、ビルドし直す(`--symlink-install`でも、足したファイルはビルドするまでインストール先に無い)。
