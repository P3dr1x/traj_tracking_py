from setuptools import find_packages, setup

package_name = 'traj_tracking_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracking.launch.py']),
        ('share/' + package_name + '/plotjuggler', ['plotjuggler/ee_traj_plot.xml']),
        ('share/' + package_name + '/bags/perfect_circle', [
            'bags/perfect_circle/metadata.yaml',
            'bags/perfect_circle/perfect_circle_0.db3'
        ]),
        ('share/' + package_name + '/controllers', ['controllers/modes_quick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattia',
    maintainer_email='mattiapedrocco@libero.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj_tracker_client = traj_tracking_py.traj_tracker_client:main',
            'ee_pose_publisher = traj_tracking_py.ee_pose_publisher:main',
            'traj_tracker_publisher = traj_tracking_py.traj_tracker_publisher:main'
        ],
    },
)
