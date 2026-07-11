from setuptools import setup

package_name = 'lite_moveit2'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='StvLi',
    maintainer_email='lipeize@deepcybo.com',
    description='MoveIt 2 motion helpers for DeepCybo Lite.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'move_to_pose = lite_moveit2.move_to_pose:main',
            'translate_right_arm = lite_moveit2.translate_right_arm:main',
        ],
    },
)
