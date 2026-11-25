from glob import glob

from setuptools import find_packages, setup

setup(
    name="bemu_description",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bemu_description"]),
        ("share/bemu_description", ["package.xml"]),
        (
            "share/bemu_description/meshes",
            glob("meshes/*"),
        ),  # the CAD meshes (tools/generate_cad.py)
    ],
    install_requires=["setuptools", "pyyaml"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
)
