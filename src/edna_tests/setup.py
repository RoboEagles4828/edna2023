from setuptools import setup

package_name = 'edna_tests'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboeagles',
    maintainer_email='roboeagles4828@gmail.com',
    description='Test robot functions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint-arm = edna_tests.publish_joint_arm_command:main',
            'joint-drive = edna_tests.publish_joint_drive_command:main',
            'publish-twist = edna_tests.publish_twist_command:main',
            'run-tests = edna_tests.run_tests_command:main',
            'arm-tests = edna_tests.arm_tests:main'
        ],
    },
)