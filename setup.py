# Co-authored by Claude Code: registers the node as a sensor processing node in the ament index
from setuptools import find_packages, setup

package_name = 'fiducials_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Registers the node as a sensor processing node for par.launch.py and the setup wizard
        ('share/ament_index/resource_index/par_framework_sensor_nodes',
            ['resource/par_framework_sensor_nodes/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vitus',
    maintainer_email='vitus.maierhoefer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fiducials_detector = fiducials_detector.fiducials_detector_node:main'
        ],
    },
)
