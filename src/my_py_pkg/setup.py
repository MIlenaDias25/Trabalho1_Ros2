from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='milena',
    maintainer_email='milena@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'py_node = my_py_pkg.my_first_node:main',
        	'publisher = my_py_pkg.publisher:main',
        	'subscriber = my_py_pkg.subscriber:main',
        	'add_two_ints = my_py_pkg.add_two_numbers:main',
        	'add_two_ints_client = my_py_pkg.add_two_numbers_client:main',
        	'number_publisher = my_py_pkg.number_publisher:main',
        	'robot_navigator = my_py_pkg.robot_navigator:main',
        	'robot_navigator_completo = my_py_pkg.robot_navigator_completo:main',
        	'publisher_1 = my_py_pkg.publisher_1:main',
        	'trabalho = my_py_pkg.trabalho:main',
        	'robot_navigator_VFH = my_py_pkg.robot_navigator_VFH:main',
        	'robot_navigator2 = my_py_pkg.robot_navigator2:main',
        	'robot_navigator3 = my_py_pkg.robot_navigator3:main',
        	'robot_navigator_waypoints = my_py_pkg.robot_navigator_waypoints:main',
        ],
    },
)
