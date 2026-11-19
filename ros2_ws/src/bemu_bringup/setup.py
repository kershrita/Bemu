from setuptools import find_packages, setup

setup(
    name="bemu_bringup",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_bringup"]),
        ("share/bemu_bringup", ["package.xml"]),
        ("share/bemu_bringup/launch", ["launch/leg.launch.py", "launch/robot.launch.py"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": ["foot_trace_node = bemu_bringup.foot_trace_node:main"]},
    zip_safe=True,
)
