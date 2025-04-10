import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gpsr_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "params"), glob("params/*")),
        (os.path.join("share", package_name, "bt_xml"), glob("bt_xml/*.xml")),
        (os.path.join("share", package_name, "test"), [*glob("test/*.txt"), *glob("test/*.json")]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "planning_node = gpsr_planning.gpsr_planning_node:main",
            "generate_json_node = gpsr_planning.generate_json:main"
        ],
    },
)
