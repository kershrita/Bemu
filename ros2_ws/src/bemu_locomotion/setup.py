from setuptools import find_packages, setup

setup(
    name="bemu_locomotion",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_locomotion"]),
        ("share/bemu_locomotion", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": ["locomotion_node = bemu_locomotion.node:main"]},
    zip_safe=True,
)
