from setuptools import find_packages, setup

setup(
    name="bemu_description",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_description"]),
        ("share/bemu_description", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    tests_require=["pytest"],
    zip_safe=True,
)
