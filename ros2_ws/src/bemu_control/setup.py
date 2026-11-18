from setuptools import find_packages, setup

setup(
    name="bemu_control",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_control"]),
        ("share/bemu_control", ["package.xml"]),
        ("share/bemu_control/config", ["config/leg_controllers.yaml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
)
