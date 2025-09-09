from setuptools import find_packages, setup

package_name = 'turtle_train_pkg'

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
    maintainer='abc',
    maintainer_email='abc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spawner = turtle_train_pkg.spawner:main",
            "follower = turtle_train_pkg.follower:main",
            "pose_broadcaster = turtle_train_pkg.pose_broadcaster:main",
        ],
    },
)
