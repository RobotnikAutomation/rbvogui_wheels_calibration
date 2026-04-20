from setuptools import find_packages, setup

package_name = 'rbvogui_wheels_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/wheel_calibration.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Jose Antonio Mendez',
    maintainer_email='jamendez@robotnik.es',
    description='The rbvogui_wheels_calibration package',
    license='BSD',
    entry_points={
        'console_scripts': [
            'calibration_node = rbvogui_wheels_calibration.calibration_node:main',
            'test_io_publisher = rbvogui_wheels_calibration.test_io_publisher:main',
        ],
    },
)
