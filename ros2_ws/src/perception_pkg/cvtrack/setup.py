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

from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py as _build_py


class _BuildPy(_build_py):
    """Install the repository-level preset YAML files into the package."""

    def run(self):
        super().run()
        source_dir = Path(__file__).parent / 'configs'
        target_dir = Path(self.build_lib) / 'cvtrack' / 'configs'
        self.mkpath(str(target_dir))
        for source_path in source_dir.glob('*.yaml'):
            self.copy_file(str(source_path), str(target_dir / source_path.name))

if __name__ == "__main__":
    setup(cmdclass={'build_py': _BuildPy})
