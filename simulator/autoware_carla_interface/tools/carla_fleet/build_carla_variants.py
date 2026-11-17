#!/usr/bin/env python3
"""Vissim の inpx か SUMO の .sumocfg から carla_variants(モデル -> CARLA の体の並び)を作る。CARLA は要らない。

See docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md (F4) and
docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md (S3).

    python3 tools/carla_fleet/build_carla_variants.py \\
        --inpx <Town01.inpx> \\
        --inpx-ref "~/CARLA feature/vissim_windows 4213552 Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx" \\
        --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json \\
        --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml \\
        --out src/autoware_carla_interface/vissim_integration/data/carla_variants_town01.yaml

    python3 tools/carla_fleet/build_carla_variants.py \\
        --sumocfg sumo/Town01.sumocfg \\
        --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json \\
        --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml \\
        --out src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml

Vissim(--inpx):
- モデル = inpx の 2D/3D モデル(model2D3D)。寸法は全セグメントを繋いだ車体全体
  (vissim_integration/inpx_models.py)。
- 割合は「需要が使う車両構成の車種割合 × 車種ごとのモデル分布」。歩行者の車種と、自車(運転シミュレータ)の
  車種は対象にしない。割合 0 のモデルは skipped に理由を残す。割合が 0 でないのに割り当てを作れなければ止める。
- 生成物の assignment・skipped には、モデルの先頭セグメントのファイル名(file。DSI の ModelFileName と同じ鍵)と
  車種(types)を書く。ブリッジは実行時に inpx を読まず、これで (VehicleType, ModelFileName) からモデルを引く。
- sources の geometry・rules のパスは、生成物のあるディレクトリからの相対パスで書く(ブリッジは起動時に sha256 を
  照合する)。inpx は記録だけ(Windows 側にあり、ブリッジは照合しない)。

SUMO(--sumocfg。sumo_integration/sumo_fleet.py):
- モデル = .sumocfg の route-files の vType を寸法(L・W)でまとめたもの(名前 L<長さ>_W<幅>)。割合は flow・vehicle の
  台数 × vTypeDistribution の確率。歩行者の vType は対象にしない。
- 生成物の各モデルには、まとめた vType の ID(vtypes)を書く。SUMO ブリッジは走行中、車の vType ID でモデルを引く。
- sources には .sumocfg と、route-files の各ファイル(.sumocfg に書かれた名前)の sha256 を書く。SUMO ブリッジは
  起動時に、渡された .sumocfg の route-files と照合する。.sumocfg のパスは、geometry・rules と同じく生成物のある
  ディレクトリからの相対パスで書く(記録だけ。ブリッジは照合に使わない。置き場所が違う PC でも同じ生成物になる)。

移植元: SBIR_sim `tools/carla_fleet/build_carla_variants.py`(3ea1650)。変えたところ: 入力は --inpx の
ファイルと、SUMO の --sumocfg(移植元の --sumo-inputs と網の登録簿 received_networks への依存を外した)、生成物に
file・types(Vissim)・vtypes(SUMO)・segments を書く、sources のパスの書き方。
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE / 'src'))
from autoware_carla_interface import carla_fleet  # noqa: E402
from autoware_carla_interface.carla_fleet import VariantError  # noqa: E402
from autoware_carla_interface.vissim_integration.inpx_models import InpxModels  # noqa: E402

GENERATED_BY = 'tools/carla_fleet/build_carla_variants.py'


def network_models(inpx) -> dict:
    """InpxModels -> {モデル: {L, W, segments, name, file, categories, types, share}}。

    types は {車種: 車種内のモデルの割合}、share は網全体での割合(合計 1)。どちらも inpx の定義を読んだもの。
    歩行者の車種と自車の車種しか使わないモデルは入らない。
    """
    shares = inpx.model_shares()
    if not any(share > 0 for share in shares.values()):
        raise VariantError('variant_no_active_models')
    models = {}
    for vehicle_type, item in inpx.vehicle_types.items():
        if not inpx.is_traffic_vehicle_type(vehicle_type):
            continue
        for model, share_in_type in item['model_shares'].items():
            if model not in shares:
                continue
            body = inpx.models[model]
            entry = models.setdefault(model, dict(
                L=body['length'], W=body['width'], segments=body['segments'], name=body['name'],
                file=body['file'], categories=set(), types={}, share=shares[model]))
            entry['categories'].add(item['category'])
            entry['types'][vehicle_type] = share_in_type
    return models


def build(models, rules, mesh) -> tuple[dict, dict]:
    """(assignment, skipped)。割合が 0 でないのに割り当てを作れないモデルがあれば止める。"""
    assignment, skipped, empty = {}, {}, []
    for model in sorted(models, key=carla_fleet._model_order):
        entry = models[model]
        if entry['share'] <= 0:
            skipped[model] = dict(L=entry['L'], W=entry['W'], reason='share_zero')
            continue
        result = carla_fleet.assign(entry, rules, mesh)
        if result is None:
            empty.append(dict(model=model, name=entry['name'], L=entry['L'], W=entry['W'],
                              categories=sorted(entry['categories'])))
            continue
        assignment[model] = dict(L=entry['L'], W=entry['W'], **result)
    if empty:
        raise VariantError('variant_model_empty', models=empty)
    return assignment, skipped


def expected_bodies(models, assignment) -> dict:
    shares = {model: models[model]['share'] for model in assignment}
    demand = carla_fleet.blueprint_demand(shares, assignment)
    return {name: 100.0 * weight
            for name, weight in sorted(demand.items(), key=lambda item: (-item[1], item[0]))}


# ---------- 書き出し(読む人のために 1 モデル 2〜数行の YAML。JSON と同じ書き方の flow 形式) ----------

def _scalar(value, signed=False, digits=3):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return 'null'
    if isinstance(value, (int, float)):
        text = f'{float(value):+.{digits}f}' if signed else f'{float(value):.{digits}f}'
        return '0.0' if float(text) == 0 and not signed else text
    text = str(value)
    if text and all(c.isalnum() or c in '._-/:' for c in text) and not text[0].isdigit():
        return text
    return json.dumps(text, ensure_ascii=False)


def _actor(actor):
    parts = [f"blueprint: {actor['blueprint']}",
             f"front_offset_m: {_scalar(actor['front_offset_m'], digits=6)}"]
    if actor.get('follows_path'):
        parts.append('follows_path: true')
    return '{' + ', '.join(parts) + '}'


def _flow(mapping):
    items = []
    for key, value in mapping.items():
        if isinstance(value, dict):
            items.append(f'{key}: {_flow(value)}')
        elif isinstance(value, (list, tuple)):
            items.append(f'{key}: [' + ', '.join(_scalar(v) for v in value) + ']')
        else:
            items.append(f'{key}: {_scalar(value, digits=2) if isinstance(value, float) else _scalar(value)}')
    return '{' + ', '.join(items) + '}'


def _identity(info):
    """モデルを引くための項目。Vissim: file・types・segments(DSI から引く)。SUMO: vtypes・segments(vType ID で引く)。"""
    if 'vtypes' in info:
        return f"vtypes: [{', '.join(_scalar(v) for v in info['vtypes'])}], segments: {info['segments']}"
    types = ', '.join(f"'{t}': {_scalar(s)}"
                      for t, s in sorted(info['types'].items(), key=lambda item: carla_fleet._model_order(item[0])))
    return f"file: {json.dumps(info['file'], ensure_ascii=False)}, types: {{{types}}}, segments: {info['segments']}"


def render(document, command) -> str:
    sumo = 'sumocfg' in document['sources']
    model_lines = ([
        '# assignment: モデル(SUMO の vType を寸法でまとめたもの。名前 L<長さ>_W<幅>) -> CARLA の体の並び',
        '#   vtypes: まとめた vType の ID。share: 出現割合(flow・vehicle の台数 × vTypeDistribution の確率を読んだもの)',
        '#           ブリッジは車の vType ID で引く。無ければ止める(npc_model_unknown)',
    ] if sumo else [
        '# assignment: モデル(inpx の 2D/3D モデル番号) -> CARLA の体の並び',
        '#   file:  モデルの先頭セグメントのファイル名(パス・拡張子なし)。DSI の ModelFileName と同じ鍵',
        '#   types: {車種: 車種内のモデルの割合}、share: 網全体での割合(どちらも inpx の定義を読んだもの)',
        '#          ブリッジは (VehicleType, ModelFileName) を types と file で引く。無ければ止める(npc_model_unknown)',
    ])
    source_lines = ([
        '#   sumocfg は .sumocfg(パスはこのファイルのあるディレクトリからの相対パス。記録だけ)。route_files は',
        '#   route-files の各ファイル(.sumocfg に書かれた名前)。ブリッジは起動時に、渡された .sumocfg の',
        '#   route-files の sha256 と照合し、違えば止める(variant_source_changed)',
    ] if sumo else [
        '#   inpx は記録だけ(Windows 側にあり、ブリッジは照合しない。モデルが変われば走行中に npc_model_unknown で止まる)',
    ])
    lines = [f"# {document['name']} — 生成物。手で書き換えない",
             '# 作り方:'] + [f'#   {line}' for line in command] + [
        '#',
        *model_lines,
        '#   front_offset_m: 車両の前端から、その actor の前端までの距離 [m]',
        '#   follows_path:   true なら車両の軌跡を遡った位置・向きに置く(連結の近似)',
        '#   dL・dW = CARLA の外形(LiDAR で測った外形、幅はミラー込み) − モデルの寸法 [m]',
        f"name: {document['name']}",
        f'generated_by: {GENERATED_BY}',
        f"generated_at: '{document['generated_at']}'",
        '',
        '# 生成元。ブリッジは起動時に geometry と rules の sha256 を照合し、違えば止める(variant_source_changed)',
        '#   geometry・rules のパスは、このファイルのあるディレクトリからの相対パス',
        *source_lines,
        'sources:']
    for key, source in document['sources'].items():
        lines.append(f'  {key}: ' + '{' + ', '.join(f'{k}: {_scalar(v)}' for k, v in source.items()) + '}')
    lines += ['', '# 適用した規則の写し(読む人のため。ブリッジは使わない)', 'rules:']
    for key, value in document['rules'].items():
        if key == 'name':
            continue
        if key == 'excluded':
            value = list(value)
        lines.append(f'  {key}: ' + (_flow(value) if isinstance(value, dict) else
                                     '[' + ', '.join(_scalar(v) for v in value) + ']' if isinstance(value, list) else
                                     _scalar(value, digits=2) if isinstance(value, float) else _scalar(value)))
    lines += ['', 'assignment:']

    def first_type(model):
        info = document['models'][model]
        if 'vtypes' in info:      # SUMO: vTypeDistribution(無ければカテゴリ)でまとめる
            return sorted(info.get('distributions') or info['categories'])[0]
        return sorted(info['types'], key=carla_fleet._model_order)[0]

    group = None
    for model in sorted(document['assignment'],
                        key=lambda m: (carla_fleet._model_order(first_type(m)), carla_fleet._model_order(m))):
        spec, info = document['assignment'][model], document['models'][model]
        vehicle_type = first_type(model)
        if vehicle_type != group:
            group = vehicle_type
            label = '車両グループ' if 'vtypes' in info else '車種'
            lines.append(f"  # --- {label} {vehicle_type}({'/'.join(sorted(info['categories']))}) ---")
        indent = ' ' * 8
        lines.append(f"  '{model}': {{L: {_scalar(spec['L'])}, W: {_scalar(spec['W'])}, "
                     f"share: {_scalar(info['share'], digits=4)}, method: {spec['method']}, "
                     f"dL: {_scalar(spec['dL'], signed=True)}, dW: {_scalar(spec['dW'], signed=True)},")
        lines.append(f'{indent}{_identity(info)},')
        actors = [_actor(actor) for actor in spec['actors']]
        lines.append(f'{indent}actors: [' + f',\n{indent}         '.join(actors) + f"]}}   # {info['name']}")
    lines += ['', '# 割り当てなし(割合 0 のモデル。交通シムが出さない。走行中に来たら variant_model_empty で止まる)',
              'skipped:']
    if not document['skipped']:
        lines[-1] = 'skipped: {}'
    for model, spec in document['skipped'].items():
        info = document['models'][model]
        lines.append(f"  '{model}': {{L: {_scalar(spec['L'])}, W: {_scalar(spec['W'])}, reason: {spec['reason']},")
        lines.append(f"        {_identity(info)}}}   # {info['name']}")
    lines += ['', '# 100 台あたりのブループリントの体数(レビュー用。ブリッジは使わない)',
              'expected_bodies_per_100_vehicles:']
    width = max(len(name) for name in document['expected_bodies_per_100_vehicles']) + 1
    for name, value in document['expected_bodies_per_100_vehicles'].items():
        lines.append(f'  {(name + ":").ljust(width)} {value:6.1f}')
    return '\n'.join(lines) + '\n'


def _shown_path(path) -> str:
    """作り方のコメントに書くパス。今のディレクトリの下なら相対パスにする(絶対パスを生成物に残さない)。"""
    relative = os.path.relpath(os.path.abspath(path))
    return str(path) if relative.startswith('..') else relative


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--inpx', type=Path, help='Vissim の inpx(Windows で読まれるものと同じ版)')
    source.add_argument('--sumocfg', type=Path, help='SUMO の .sumocfg(co-sim で使うもの)。route-files を読む')
    parser.add_argument('--inpx-ref', help='inpx の出どころ(例: リポジトリ・コミット・パス)。渡すと、inpx のパスの代わりに記録する')
    parser.add_argument('--sumocfg-ref', help='.sumocfg の出どころ(例: リポジトリ・コミット・パス)。渡すと、パスの代わりに記録する')
    parser.add_argument('--geometry', type=Path, required=True,
                        help='measure_blueprint_mesh.py の出力(mesh_measurements_*.json)')
    parser.add_argument('--rules', type=Path, required=True, help='carla_variant_rules.yaml')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--name', help='既定は --out のファイル名(拡張子なし)')
    parser.add_argument('--generated-at', help='生成日時(ISO 8601)。既定は今。作り直しの差分を見るときに固定する')
    args = parser.parse_args(argv)

    out_dir = args.out.resolve().parent
    try:
        rules = carla_fleet.load_rules(args.rules)
        mesh, meta = carla_fleet.load_mesh(args.geometry)
        if args.inpx:
            models = network_models(InpxModels.from_file(args.inpx))
        else:
            # SUMO の読み込みは SUMO 構成のブランチにだけある(sumo_integration)
            from autoware_carla_interface.sumo_integration import sumo_fleet
            names = sumo_fleet.route_file_names(args.sumocfg)
            models, _ = sumo_fleet.read_models([path for _, path in names])
        assignment, skipped = build(models, rules, mesh)
    except VariantError as exc:
        print(f'[build_carla_variants] 止めた: {exc}', file=sys.stderr)
        return 1
    if args.inpx:
        # --inpx-ref があれば、パスの代わりに出どころを記録する(渡すのが git から取り出した一時ファイルのとき)
        traffic = (dict(ref=args.inpx_ref) if args.inpx_ref else dict(path=str(args.inpx)))
        traffic['sha256'] = carla_fleet.digest(args.inpx)
        traffic_sources = dict(inpx=traffic)
    else:
        # パスは生成物のディレクトリからの相対パス(絶対パスにすると、置き場所の違う PC で生成物が変わる)
        traffic = (dict(ref=args.sumocfg_ref) if args.sumocfg_ref
                   else dict(path=carla_fleet.relative_source(args.sumocfg, out_dir)))
        traffic['sha256'] = carla_fleet.digest(args.sumocfg)
        traffic_sources = dict(sumocfg=traffic,
                               route_files={name: carla_fleet.digest(path) for name, path in names})
    document = dict(
        name=args.name or args.out.stem,
        generated_at=args.generated_at or datetime.now().astimezone().isoformat(timespec='seconds'),
        sources=dict(**traffic_sources,
                     geometry=dict(path=carla_fleet.relative_source(args.geometry, out_dir),
                                   sha256=carla_fleet.digest(args.geometry),
                                   carla_server_version=meta.get('carla_server_version')),
                     rules=dict(path=carla_fleet.relative_source(args.rules, out_dir),
                                sha256=carla_fleet.digest(args.rules))),
        rules=rules, assignment=assignment, skipped=skipped, models=models,
        expected_bodies_per_100_vehicles=expected_bodies(models, assignment))
    if args.inpx:
        input_lines = [(f'    --inpx <{args.inpx_ref} を取り出したファイル> \\' if args.inpx_ref else f'    --inpx {args.inpx} \\'),
                       *([f'    --inpx-ref {json.dumps(args.inpx_ref, ensure_ascii=False)} \\'] if args.inpx_ref else [])]
    else:
        input_lines = [f'    --sumocfg {_shown_path(args.sumocfg)} \\',
                       *([f'    --sumocfg-ref {json.dumps(args.sumocfg_ref, ensure_ascii=False)} \\'] if args.sumocfg_ref else [])]
    command = ['python3 tools/carla_fleet/build_carla_variants.py \\',
               *input_lines,
               f'    --geometry {_shown_path(args.geometry)} \\',
               f'    --rules {_shown_path(args.rules)} \\',
               f'    --out {_shown_path(args.out)}']
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(render(document, command), encoding='utf-8')
    print(f'[build_carla_variants] {args.out}: 割り当て {len(assignment)} モデル、skipped {len(skipped)}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
