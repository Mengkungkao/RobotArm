#!/usr/bin/env python3
# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Expand a Xacro model into a single-line, comment-free URDF.

    compact_xacro.py <model.urdf.xacro> [name:=value ...]

WHY THIS EXISTS
===============
gazebo_ros2_control cannot start the controller manager from a pretty-printed
URDF.  The plugin has no way to set a parameter on the node it creates, so it
builds a command line instead and re-passes the whole robot description as a
single argument (gazebo_ros2_control_plugin.cpp, Humble):

    std::string rb_arg = std::string("robot_description:=") + urdf_string;
    arguments.push_back(RCL_PARAM_FLAG);
    arguments.push_back(rb_arg);
    ...
    rcl_parse_arguments(argv.size(), argv.data(), ..., &rcl_args);

rcl parses that value as a YAML scalar, and an ordinary xacro output is not
one: it spans hundreds of lines, and its XML comments carry ": " sequences
(which end an unquoted scalar) and "#" characters (which start a YAML
comment).  The parse fails, the plugin returns before creating the
controller manager, and every spawner then times out against a manager that
was never created.

The string the plugin re-passes is the one we set on robot_state_publisher,
so the fix belongs here: emit a URDF that is a valid YAML scalar.  None of
the offending content is robot data - it is all comments and indentation, so
nothing is lost.

The output is otherwise identical to `xacro`, and a test asserts that: same
links, same joints, same origins.
"""

import re
import sys

try:
    import xacro
except ImportError:                                          # pragma: no cover
    sys.exit('compact_xacro needs the xacro package from your ROS installation')


def strip_noise(node):
    """Drop comments and whitespace-only text nodes, recursively."""
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
            child.unlink()
        elif child.nodeType == child.TEXT_NODE and not child.data.strip():
            node.removeChild(child)
            child.unlink()
        else:
            strip_noise(child)


def compact(path, mappings):
    """Expand `path` and return it as one line with no comments."""
    document = xacro.process_file(path, mappings=mappings)
    strip_noise(document.documentElement)
    # toxml() does not pretty-print, so this is already one line; the collapse
    # is belt and braces for text nodes that survived.
    return re.sub(r'\s+', ' ', document.documentElement.toxml()).strip()


def main(argv):
    if not argv:
        sys.exit(__doc__.strip().splitlines()[2].strip())

    mappings = {}
    for argument in argv[1:]:
        if ':=' not in argument:
            sys.exit(f"expected name:=value, got '{argument}'")
        name, value = argument.split(':=', 1)
        mappings[name] = value

    urdf = compact(argv[0], mappings)

    # A description that is not a YAML scalar would fail deep inside Gazebo
    # with an unhelpful message; refuse here instead, where it is obvious.
    for bad, why in (('\n', 'a newline'), ('<!--', 'a comment'),
                     (': ', 'a ": " sequence'), ('#', 'a "#" character')):
        if bad in urdf:
            sys.exit(f'refusing to emit a description containing {why}: '
                     f'gazebo_ros2_control would fail to parse it')

    sys.stdout.write(urdf)


if __name__ == '__main__':
    main(sys.argv[1:])
