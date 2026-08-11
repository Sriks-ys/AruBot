from setuptools import find_packages, setup

package_name = 'goal_managers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arubot',
    maintainer_email='yssriker@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "goal_manager = goal_managers.GoalManager:main",
            "aruco_detector = goal_managers.ArucoDetector:main",
        ],
    },
)

