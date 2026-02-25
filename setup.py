"""Backward-compatible setup.py for pip install -e ."""
from setuptools import setup, find_packages

setup(
    name="refringence-ros2-bridge",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "python-socketio[asyncio]>=5.0",
        "aiohttp>=3.0",
    ],
    entry_points={
        "console_scripts": [
            "refringence-bridge=refringence_bridge.cli:main",
        ],
    },
)
