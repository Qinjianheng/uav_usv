from setuptools import find_packages, setup

package_name = 'uav_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['numpy', 'setuptools'],
    zip_safe=True,
    maintainer='qin',
    maintainer_email='email@example.com',
    description='UAV-USV simulation, guidance and offboard control nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_listener = uav_control.position_listener:main',
            'offboard_takeoff = uav_control.offboard_takeoff:main',
            'moving_target = uav_control.moving_target:main',
            'pure_pursuit = uav_control.pure_pursuit:main',
            'predictive_intercept = uav_control.predictive_intercept:main',
            'trajectory_impact_sim = '
            'uav_control.trajectory_impact_sim:main',
            'target_kalman_filter = '
            'uav_control.tracking.target_kalman_filter:main',
            'front_tof_monitor = '
            'uav_control.perception.front_tof_monitor:main',
            'dual_tof_selector = '
            'uav_control.perception.dual_tof_selector:main',
        ],
    },
)
