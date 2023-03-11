from setuptools import setup

package_name = 'frc_auton'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rosbag'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='nchan18@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reader = frc_auton.runner:main',
        ],
    },
)
