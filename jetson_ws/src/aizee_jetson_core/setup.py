from setuptools import find_packages, setup

package_name = 'aizee_jetson_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ltr',
    maintainer_email='john@ltr.dev',
    description='The core ROS2 package for Aizee V1.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'center_feetech_servos = aizee_jetson_core.center_feetech_servos:main',
            'center_lx_servos = aizee_jetson_core.center_lx_servos:main',
            'test_move_robstride = aizee_jetson_core.test_move_robstride:main',
            'arm_node = aizee_jetson_core.arm_node:main',
            'scservo_sync_driver = aizee_jetson_core.scservo_sync_driver:main',
        ],
    },
)
