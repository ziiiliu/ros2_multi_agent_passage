from setuptools import setup

package_name = "nn_simulator"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zliu",
    maintainer_email="zl413@cam.ac.uk",
    description="NN simulator to replace the simple_simulator",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robomaster = nn_simulator.nn_robomaster:main",
        ],
    },
)
