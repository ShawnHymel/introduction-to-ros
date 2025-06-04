from setuptools import find_packages, setup

package_name = 'my_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pubsub_example_launch.py',
            'launch/pubsub_config_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/pubsub_debug.yaml',
            'config/pubsub_prod.yaml',
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abc',
    maintainer_email='abc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
