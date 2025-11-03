from setuptools import find_packages, setup

package_name = 'learning_action'

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
    maintainer='ethan',
    maintainer_email='2276250774@qq.com',
    description='TODO: Package description',
    license='Apach-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'action_move_client  = learning_action.action_move_client:main',
         'action_move_server  = learning_action.action_move_server:main',
        ],
    },
)
