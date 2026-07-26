"""Setuptools entry point for the vendored ``cvtrack`` package.

This is intentionally a thin shim over :file:`pyproject.toml`.  Modern
setuptools (``>=61``) reads project metadata directly from the
``[project]`` table in ``pyproject.toml``; this file exists so that
``pip install -e .`` keeps working in older toolchains and so that
``python setup.py`` style commands remain callable for debugging.

Vendored source-of-truth policy
--------------------------------
This is the **only** copy of the cvtrack package used by the
Swarm-Control-System ROS2 perception node.  The independent reference
checkout at ``/home/hhh/Downloads/cv_tracking_demo/`` is kept around
for documentation and benchmarking only; runtime imports must resolve
to this vendored tree.  See ``MIGRATION.md`` for context.
"""

from setuptools import setup

if __name__ == "__main__":
    setup()
