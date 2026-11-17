"""
Where the `sumo_cfg_file` launch parameter points to.

The SUMO scenarios of the co-simulation (`.sumocfg`, net, routes) live in this package under
`sumo/examples/` and are installed to `share/autoware_carla_interface/sumo/examples/`
(CMakeLists.txt). A relative `sumo_cfg_file` (e.g. `Town01.sumocfg`) is taken from there, like a
relative `sumo_carla_variants` is taken from `sumo_integration/data/`. An absolute path (or one
starting with `~`) is used as it is.

Not part of the vendored upstream bridge (this file is new in autoware_carla_interface). Importable
without `traci`/`sumolib`/`carla`; `ament_index_python` is imported only when the share directory
is needed.
"""

import os

PACKAGE_NAME = 'autoware_carla_interface'


def examples_dir():
    """`share/autoware_carla_interface/sumo/examples` of the installed package."""
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory(PACKAGE_NAME), 'sumo', 'examples')


def resolve_sumo_cfg_file(value, base_dir=None):
    """
    Return the `.sumocfg` path SUMO is started with.

    - `''` stays `''` (no SUMO).
    - `~` is expanded; an absolute path is returned unchanged.
    - A relative path is joined to `base_dir` (default: `examples_dir()`).

    Raises ValueError when the resulting file does not exist, naming the path that was tried, so a
    wrong name is reported before SUMO is started.
    """
    value = str(value).strip()
    if not value:
        return ''
    path = os.path.expanduser(value)
    note = ''
    if not os.path.isabs(path):
        if base_dir is None:
            base_dir = examples_dir()
        path = os.path.join(base_dir, path)
        note = ' (a relative sumo_cfg_file is taken from %s)' % (base_dir,)
    path = os.path.normpath(path)
    if not os.path.isfile(path):
        raise ValueError('sumo_cfg_file %r not found: %s%s' % (value, path, note))
    return path
