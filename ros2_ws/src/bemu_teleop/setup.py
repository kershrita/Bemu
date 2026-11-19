from setuptools import find_packages, setup

setup(
    name="bemu_teleop",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_teleop"]),
        ("share/bemu_teleop", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "keyboard = bemu_teleop.keyboard:main",
            "gamepad = bemu_teleop.gamepad:main",
        ]
    },
    zip_safe=True,
)
