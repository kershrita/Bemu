from setuptools import find_packages, setup

setup(
    name="bemu_gait",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_gait"]),
        ("share/bemu_gait", ["package.xml"]),
        ("share/bemu_gait/config", ["config/gait.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
)
