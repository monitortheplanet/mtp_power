#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: the launch file must keep binding the node the parameters name.

"""
The launch file must keep binding the node the parameter file names.

ROS 2 silently ignores a ``ros__parameters`` block whose root key matches no
node, so renaming a node without renaming the matching YAML key leaves every
parameter at its default with nothing in the log. That failure is invisible at
runtime and expensive to find, which is what these tests exist to prevent.

Two checks are package-wide, over every launch file there is:

* every non-wildcard root key of every parameter file is a node name some
  launch file of this package actually starts;
* every launch file still builds, verified with ``ros2 launch ... --show-args``
  (skipped when the package is not on the ``AMENT_PREFIX_PATH``).

The rest resolve ``power_manager.launch.py`` and assert on what the node would
actually receive. Node names, like everything else here, are read back from
the resolved launch description rather than matched in the source text: a
name built from a constant or a substitution is a name a regex over the file
would miss, and missing it is how the mismatch these tests exist to catch
would get through. What they pin is the part of the launch file that fails
quietly:

* the node keeps the name ``power_manager``, which is the root key of
  ``config/power_manager.yaml`` -- a mismatch leaves every value at its code
  default, and that default set includes the button topic and ``dry_run:
  false``, so it is not a cosmetic failure;
* it keeps its package, executable, global namespace and empty remap list,
  because ``mtp_carrier_io`` and the HMI bridge were written against
  ``/power/state``;
* it is handed this package's parameter file and no inline parameter, so the
  configuration stays in one place;
* the configuration argument is reachable, has a description, and is *not*
  called ``config_file``: ``mecgeos_bringup`` declares a top-level
  ``config_file`` and ``IncludeLaunchDescription`` does not scope launch
  configurations, so the shared name is inherited in one direction and leaks
  in the other. That was a real bug in ``mtp_carrier_io``.

Nodes are expanded, never visited, so no process is ever started by this file.
"""

import os
import pathlib
import shutil
import subprocess

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import get_launch_description_from_python_launch_file
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml

PACKAGE = 'mtp_power'

ROOT = pathlib.Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / 'launch'
CONFIG_DIR = ROOT / 'config'
POWER_LAUNCH = LAUNCH_DIR / 'power_manager.launch.py'

# A parameter file root key may be a wildcard instead of a node name.
WILDCARDS = {'/**', '/*', '**'}

# The arguments power_manager.launch.py offers: the contract mecgeos_bringup
# was written against, and the handle the live dry-run rehearsal is done with.
# The default is a FindPackageShare substitution, so it is pinned by the path
# it has to end in rather than by an absolute path.
EXPECTED_ARGUMENT_DEFAULT_TAILS = {
    'power_manager_config': (PACKAGE, 'config', 'power_manager.yaml'),
}

# Names no launch file of this package may declare. `config_file` is the
# top-level argument of mecgeos_bringup/launch/parametric_bringup.launch.py;
# sharing it means an include that forgets to pass it inherits the robot
# config, and one that passes it overwrites the parent's value for every
# include after it. Neither shows up in a log.
FORBIDDEN_ARGUMENT_NAMES = {'config_file'}

# The one node this launch file starts, and its identity. The name is also the
# root key of config/power_manager.yaml; test_the_node_name_is_the_parameter_
# file_root_key asserts that the two are still the same string.
EXPECTED_NODE_NAME = 'power_manager'
EXPECTED_NODE_PACKAGE = PACKAGE
EXPECTED_NODE_EXECUTABLE = 'power_manager'
EXPECTED_NODE_OUTPUT = 'screen'

# The node is started once, in the global namespace, and dies with the
# launch: there is no respawn today. Turning one on is a behaviour change --
# a node that restarts mid-gesture forgets that the button is held -- so it is
# pinned here rather than left unstated.
EXPECTED_RESPAWN = False
EXPECTED_RESPAWN_DELAY = None


def package_is_installed():
    """Whether FindPackageShare can resolve this package's share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(PACKAGE)
    except Exception:
        return False
    return True


# Resolving the launch description performs FindPackageShare, which needs the
# package installed. A bare source checkout keeps the text-only checks above.
requires_installed_package = pytest.mark.skipif(
    not package_is_installed(),
    reason=f'{PACKAGE} is not installed: FindPackageShare cannot resolve it',
)


def launch_files():
    """Every launch file of this package, in a stable order."""
    return sorted(LAUNCH_DIR.glob('*.launch.py')) if LAUNCH_DIR.is_dir() else []


def parameter_files():
    """Parameter files: a mapping whose values carry a `ros__parameters` block."""
    if not CONFIG_DIR.is_dir():
        return []
    found = []
    for path in sorted(CONFIG_DIR.glob('*.yaml')):
        try:
            doc = yaml.safe_load(path.read_text())
        except yaml.YAMLError:
            continue
        if isinstance(doc, dict) and any(
                isinstance(v, dict) and 'ros__parameters' in v for v in doc.values()):
            found.append((path, doc))
    return found


def load_launch_description(path=POWER_LAUNCH, /, **arguments):
    """
    Load a launch file and return it with a context holding its arguments.

    Argument values are seeded into the context before the declarations run,
    because `DeclareLaunchArgument` only applies its default to a name the
    context does not already hold -- which is exactly how the command line
    overrides one.
    """
    description = get_launch_description_from_python_launch_file(str(path))
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return description, context


def declared_node_names():
    """
    Every node name any launch file of this package would start.

    Conditions are ignored on purpose: a parameter block for a node that is
    only started under a condition is still a legitimate parameter block.
    """
    names = set()
    for path in launch_files():
        description, context = load_launch_description(path)
        for entity in description.entities:
            if isinstance(entity, Node):
                entity._perform_substitutions(context)
                names.add(entity.node_name.rsplit('/', 1)[-1])
    return names


def declared_arguments(path=POWER_LAUNCH):
    """Map every declared argument of a launch file to its declaration."""
    description = get_launch_description_from_python_launch_file(str(path))
    return {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def resolve_nodes(**arguments):
    """
    Resolve the Nodes of the launch file that would actually be started.

    A Node whose condition is false is dropped here exactly as launch drops
    it, so a node that stops being started has to show up as a missing node.
    """
    description, context = load_launch_description(POWER_LAUNCH, **arguments)
    nodes = []
    for entity in description.entities:
        if not isinstance(entity, Node):
            continue
        if entity.condition is not None and not entity.condition.evaluate(context):
            continue
        entity._perform_substitutions(context)
        nodes.append((entity, context))
    return nodes


def nodes_by_short_name(**arguments):
    """Resolve the launch file into a {short node name: Node} mapping."""
    return {node.node_name.rsplit('/', 1)[-1]: node
            for node, _ in resolve_nodes(**arguments)}


def the_node(**arguments):
    """Resolve the single node this launch file starts."""
    nodes = nodes_by_short_name(**arguments)
    assert EXPECTED_NODE_NAME in nodes, \
        f'{EXPECTED_NODE_NAME} is not started: {sorted(nodes)}'
    return nodes[EXPECTED_NODE_NAME]


def parameter_arguments(node):
    """
    Read back the parameter entries a resolved Node hands to the process.

    launch_ros exposes them only under a name-mangled attribute, as
    `(value, is_file)` pairs. They are never unlinked here: a path-valued
    `parameters=` entry is passed through untouched, so the path in the pair
    is the repository's own config file, not a temporary copy of it.
    """
    return list(getattr(node, '_Node__expanded_parameter_arguments', None) or [])


def respawn_settings(node):
    """
    Read back the respawn settings of a resolved Node.

    They live on ExecuteLocal under name-mangled attributes with no public
    accessor; they are asserted on anyway, because whether the node comes
    back after a crash decides whether the operator keeps a 3D map.
    """
    return (node.__dict__.get('_ExecuteLocal__respawn'),
            node.__dict__.get('_ExecuteLocal__respawn_delay'))


@requires_installed_package
@pytest.mark.parametrize('path,doc', parameter_files(),
                         ids=lambda a: getattr(a, 'name', ''))
def test_root_keys_are_declared_node_names(path, doc):
    """A root key no launch file declares would be ignored without a word."""
    declared = declared_node_names()
    keys = {k for k in doc if k not in WILDCARDS}
    unknown = sorted(k for k in keys if k.lstrip('/') not in declared)
    assert not unknown, (
        f'{path.name}: root key(s) {unknown} match no node name declared in '
        f'{[p.name for p in launch_files()]}. Declared names: {sorted(declared)}. '
        'Rename the YAML key and the launch `name=` in the same commit.'
    )


@pytest.mark.parametrize('path', launch_files(), ids=lambda p: p.name)
def test_launch_file_builds(path):
    """`--show-args` builds the whole launch description, so it catches typos."""
    if shutil.which('ros2') is None:
        pytest.skip('ros2 CLI not on PATH')
    if PACKAGE not in os.environ.get('AMENT_PREFIX_PATH', ''):
        pytest.skip(f'{PACKAGE} is not on AMENT_PREFIX_PATH')
    result = subprocess.run(
        ['ros2', 'launch', PACKAGE, path.name, '--show-args'],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, f'{path.name}:\n{result.stdout}\n{result.stderr}'


@requires_installed_package
def test_the_argument_list_is_unchanged():
    """Dropping or renaming an argument breaks every caller silently."""
    assert sorted(declared_arguments()) == sorted(EXPECTED_ARGUMENT_DEFAULT_TAILS)


@requires_installed_package
@pytest.mark.parametrize('name,tail', sorted(EXPECTED_ARGUMENT_DEFAULT_TAILS.items()))
def test_argument_defaults_are_unchanged(name, tail):
    """The default has to keep pointing at this package's own parameter file."""
    _, context = load_launch_description()
    value = pathlib.Path(context.launch_configurations[name])
    assert value.is_absolute(), value
    assert value.parts[-len(tail):] == tail, value


@requires_installed_package
@pytest.mark.parametrize('name', sorted(EXPECTED_ARGUMENT_DEFAULT_TAILS))
def test_every_argument_has_a_description(name):
    """`--show-args` is the only documentation an operator reads at 2 a.m."""
    description = declared_arguments()[name].description
    assert description and description != 'no description given', name


@requires_installed_package
@pytest.mark.parametrize('path', launch_files(), ids=lambda p: p.name)
def test_no_argument_shadows_the_bringup_top_level_name(path):
    """A shared name is inherited, or leaked, through the include chain."""
    clashing = sorted(FORBIDDEN_ARGUMENT_NAMES & set(declared_arguments(path)))
    assert not clashing, (
        f'{path.name} declares {clashing}, which mecgeos_bringup also declares '
        'at top level. IncludeLaunchDescription does not scope launch '
        'configurations: name the argument after this package instead.'
    )


@requires_installed_package
def test_the_launch_starts_the_power_node_and_nothing_else():
    """An extra node here is a second thing that can halt the robot."""
    assert set(nodes_by_short_name()) == {EXPECTED_NODE_NAME}


@requires_installed_package
def test_the_node_keeps_its_identity():
    """Renaming the node or pushing it into a namespace breaks the YAML key."""
    node = the_node()
    assert node.node_name.rsplit('/', 1)[-1] == EXPECTED_NODE_NAME
    # No namespace of its own: the node owns absolute topics (/power/state
    # and the two inputs), so a namespace would only hide it.
    assert node.expanded_node_namespace == Node.UNSPECIFIED_NODE_NAMESPACE
    assert node.node_package == EXPECTED_NODE_PACKAGE
    assert node.node_executable == EXPECTED_NODE_EXECUTABLE


@requires_installed_package
def test_the_node_takes_no_remaps():
    """Every topic is a parameter; a remap on top would only hide them."""
    assert not the_node().expanded_remapping_rules


@requires_installed_package
def test_the_node_logs_to_the_screen_line_by_line():
    """The wiring banner and the refusals are read during commissioning."""
    node = the_node()
    _, context = load_launch_description()
    output = node.__dict__.get('_ExecuteLocal__output')
    assert perform_substitutions(context, output) == EXPECTED_NODE_OUTPUT
    assert node.__dict__.get('_ExecuteLocal__emulate_tty') is True


@requires_installed_package
def test_the_respawn_settings_are_unchanged():
    """Whether the node comes back after a crash is a deliberate decision."""
    assert respawn_settings(the_node()) == (EXPECTED_RESPAWN, EXPECTED_RESPAWN_DELAY)


@requires_installed_package
def test_the_node_gets_this_packages_parameter_file_and_nothing_else():
    """Pointed at another YAML the node starts on defaults, silently."""
    entries = parameter_arguments(the_node())
    assert len(entries) == 1, entries
    value, is_file = entries[0]
    assert is_file is True, entries
    path = pathlib.Path(value)
    assert path.parts[-3:] == (PACKAGE, 'config', 'power_manager.yaml'), path
    assert path.is_file(), path


@requires_installed_package
def test_the_launch_sets_no_parameter_of_its_own():
    """One place to configure it, and the value worth not forgetting is dry_run."""
    inline = [entry for entry in parameter_arguments(the_node()) if not entry[1]]
    assert not inline, (
        f'the launch file sets {inline} inline as well as in the YAML: '
        'keep the configuration in config/power_manager.yaml only.'
    )


@requires_installed_package
def test_the_node_name_is_the_parameter_file_root_key():
    """The binding this whole module exists for, asserted on the real names."""
    value = parameter_arguments(the_node())[0][0]
    document = yaml.safe_load(pathlib.Path(value).read_text())
    keys = {k.lstrip('/') for k in document if k not in WILDCARDS}
    assert keys == {the_node().node_name.rsplit('/', 1)[-1]}, keys


@requires_installed_package
def test_the_config_argument_reaches_the_node(tmp_path):
    """The dry-run rehearsal is done through this argument; it must work."""
    override = tmp_path / 'override.yaml'
    override.write_text(
        'power_manager:\n  ros__parameters:\n    dry_run: true\n')
    node = the_node(power_manager_config=str(override))
    assert parameter_arguments(node) == [(str(override), True)]
