from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ares_reliability'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='priestly',
    maintainer_email='priestly@todo.todo',
    description='Reliability and fault-injection tooling for ARES autonomy.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gnss_fault_injector = '
            'ares_reliability.gnss_fault_injector:main',
            'imu_fault_injector = '
            'ares_reliability.imu_fault_injector:main',
            'wheel_fault_injector = '
            'ares_reliability.wheel_fault_injector:main',
            'consistency_monitor = '
            'ares_reliability.consistency_monitor:main',
            'imu_consistency_monitor = '
            'ares_reliability.imu_consistency_monitor:main',
            'localization_consistency_monitor = '
            'ares_reliability.localization_consistency_monitor:main',
            'trust_engine = ares_reliability.trust_engine:main',
            'recovery_manager = ares_reliability.recovery_manager:main',
            'gnss_trusted_proxy = '
            'ares_reliability.gnss_trusted_proxy:main',
            'experiment_recorder = '
            'ares_reliability.experiment_recorder:main',
            'estimator_health_monitor = '
            'ares_reliability.estimator_health_monitor:main',
            'analyze_week2_results = '
            'ares_reliability.result_analysis:main',
            'analyze_week3_results = '
            'ares_reliability.result_analysis:main',
            'analyze_week4_results = '
            'ares_reliability.week4_analysis:main',
            'analyze_week5_results = '
            'ares_reliability.week5_analysis:main',
            'week6_navigation_mission = '
            'ares_reliability.week6_navigation_mission:main',
            'week6_nav2_startup_barrier = '
            'ares_reliability.week6_nav2_startup_barrier:main',
            'week6_nav2_lifecycle_orchestrator = '
            'ares_reliability.week6_nav2_lifecycle_orchestrator:main',
        ],
    },
)
