from setuptools import find_packages, setup

package_name = 'nirav_assn2'

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
    maintainer='nirav24',
    maintainer_email='niravbhattad24@gmail.com',
    description='Assignment 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'd_rover2 = nirav_assn2.d_rover2:main',
        	'd_rover3 = nirav_assn2.d_rover3:main',
        	'd_rover4 = nirav_assn2.d_rover4:main',
        	'basestation = nirav_assn2.basestation:main',
        ],
    },
)
