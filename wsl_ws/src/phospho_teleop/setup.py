from setuptools import setup

package_name = 'phospho_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=['phospho_teleop', 'phospho_teleop.hardware'],
    package_dir={'': ''},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@example.com',
    description='Teleoperation pipeline from Phosphobot integrated for Aizee arms.',
    license='MIT',
    entry_points={'console_scripts': ['teleop_server = phospho_teleop.teleop_server:main']},
)
