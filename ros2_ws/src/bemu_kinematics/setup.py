from setuptools import find_packages, setup

setup(
    name="bemu_kinematics",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_kinematics"]),
        ("share/bemu_kinematics", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
)
