from setuptools import find_packages, setup

package_name = 'rx200_moveit_control'

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
    maintainer='master26',
    maintainer_email='master26@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'rx200_gui = rx200_moveit_control.main_window:main',
=======
            "input_manager_node = rx200_moveit_control.input_manager_node:main",
>>>>>>> b7f94f99a0050250b1f7720ff3072e562fc881fa
        ],
    },
)
