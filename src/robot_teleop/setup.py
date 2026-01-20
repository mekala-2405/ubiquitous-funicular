from setuptools import setup

package_name = 'robot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harsh',
    maintainer_email='skdhiksdhj',
    description=' ',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            
            'teleopkey = robot_teleop.teleop:main',
            
       
            
        ],
    },
)
#
