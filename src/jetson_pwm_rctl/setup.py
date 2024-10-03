from setuptools import find_packages, setup

package_name = 'jetson_pwm_rctl'

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
    maintainer='dthusian',
    maintainer_email='dthusian@todo.todo',
    description='TODO: Package description',
    license='Unlicensed',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetson_pwm_rctl = jetson_pwm_rctl.jetson_pwm_rctl_node:main'
        ],
    },
)
