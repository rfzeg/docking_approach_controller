from setuptools import setup

package_name = 'docking_approach_controller'

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
    maintainer='Roberto Zegers',
    maintainer_email='user@todo.todo',
    description='Docking approach controller',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'approach_controller_exe = docking_approach_controller.docking_approach_controller:main',
        ],
    }
)
