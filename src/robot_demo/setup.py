from setuptools import setup
from pathlib import Path

data = [
    ("share/ament_index/resource_index/packages", ["resource/robot_demo"]),
    ("share/robot_demo", ["package.xml"]),
]
for directory in ("launch", "config", "assets"):
    parents = {
        p.parent
        for p in Path(directory).rglob("*")
        if p.is_file() and "__pycache__" not in p.parts
    }
    for parent in sorted(parents):
        data.append(
            (
                "share/robot_demo/" + str(parent),
                [
                    str(p)
                    for p in sorted(parent.iterdir())
                    if p.is_file() and p.suffix != ".pyc"
                ],
            )
        )
setup(
    name="robot_demo",
    version="2.0.0",
    packages=["robot_demo"],
    data_files=data,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robot demo",
    maintainer_email="robot-demo@example.com",
    description="Sensor-built mapping and GPS navigation with selectable robots and worlds",
    license="MIT",
    entry_points={
        "console_scripts": [
            "navigator = robot_demo.navigation:main",
            "monitor = robot_demo.monitor:main",
            "localizer = robot_demo.localization:main",
            "mapper = robot_demo.mapping:main",
            "guard = robot_demo.safety:main",
        ]
    },
)
