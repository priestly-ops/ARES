from setuptools import find_packages, setup

package_name = 'ares_localization'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='priestly',
    maintainer_email='priestly@example.com',

    description='ARES localization and navigation helper nodes',

    license='Apache-2.0',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
            'cmd_vel_adapter = ares_localization.cmd_vel_adapter:main',
        ],
    },
)