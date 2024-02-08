from setuptools import setup

package_name = 'tilt_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Margaria',
    maintainer_email='gabmargaria@gmail.com',
    description='Ros node to control M4 tilt angle',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tilt_control_node = tilt_control_pkg.tilt_control_node:main'
        ],
    },
)
