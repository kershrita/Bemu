from setuptools import find_packages, setup

setup(
    name="bemu_sim",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_sim"]),
        ("share/bemu_sim", ["package.xml"]),
        ("share/bemu_sim/config", ["config/actuators.yaml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
)
