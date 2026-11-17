# SUMOのシナリオ(SUMO-CARLA-Autoware co-sim)

co-simの`sumo_cfg_file`に渡すSUMOのシナリオ。CARLAの版(0.9.15・0.9.16)によらず、ここのものを使う。

`share/autoware_carla_interface/sumo/examples/`にインストールされ(`CMakeLists.txt`)、`sumo_cfg_file:=Town01.sumocfg`のような相対パスは、
そこから数える(`src/autoware_carla_interface/sumo_integration/sumo_scenarios.py`)。`--symlink-install`ならインストール先はここへのリンクなので、
ファイルを書き換えてもビルドし直さなくてよい(ファイルを足したときはビルドし直す)。

| ファイル | 内容 |
| --- | --- |
| `Town01.sumocfg`・`net/Town01.net.xml`・`rou/Town01.rou.xml` | Town01。CARLAの信号36個に対応する信号、Vissimと同じ車両グループ |
| `Town05.sumocfg`・`net/Town05.net.xml`・`rou/Town05.rou.xml` | Town05。CARLAの信号54個に対応する信号 |
| `Town10HD.sumocfg`・`net/Town10HD.net.xml`・`rou/Town10HD.rou.xml` | Town10HD。CARLAの信号15個に対応する信号 |
| `carlavtypes.rou.xml` | CARLAの車の寸法のvType(0.9.15のCARLAから作ったもの、41種) |
| `vissim_fleet_vtypes.rou.xml` | Town01の車両グループのvType(Vissimの`Town01.inpx`のモデル) |
| `viewsettings.xml` | sumo-guiの表示設定 |

- 2026-11-17に、CARLAリポジトリ(`~/CARLA`)の`main`(`4e22c5e`)の`Co-Simulation/Sumo/examples/`から写した。
- net・ルートを作り直すツール(`add_carla_tls.py`・`make_vissim_fleet_vtypes.py`など)と、その手順書(`Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`)は`~/CARLA`にある。
  `~/CARLA`で作り直したら、ここの同じ相対位置へ写す。
- Town01のroute-files(`carlavtypes.rou.xml`・`vissim_fleet_vtypes.rou.xml`・`rou/Town01.rou.xml`)を変えたら、
  ブループリント表`src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml`を作り直す
  (`docs/SUMO-CARLA-Autoware_co-sim_起動手順.md`の2.2.3)。作り直さないと、co-simの起動時に`variant_source_changed`で止まる。
