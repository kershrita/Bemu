from setuptools import find_packages, setup

setup(
    name="bemu_cad",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_cad"]),
        ("share/bemu_cad", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
)
