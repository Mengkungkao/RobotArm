from setuptools import find_packages, setup

package_name = 'robot_arm_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_arm maintainers',
    maintainer_email='mengkungkao@gmail.com',
    description='Command-line tools for the 6-DOF robot arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_joint = robot_arm_tools.move_joint:cli',
            'move_pose = robot_arm_tools.move_pose:cli',
            'fk = robot_arm_tools.fk:cli',
            'ik = robot_arm_tools.ik:cli',
            'status = robot_arm_tools.status:cli',
            'stop = robot_arm_tools.stop:cli',
            'e_stop = robot_arm_tools.e_stop:cli',
            'calibrate_joints = robot_arm_tools.calibrate_joints:cli',
        ],
    },
)
