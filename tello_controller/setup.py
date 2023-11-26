from setuptools import setup

package_name = 'tello_controller'

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
    maintainer='root',
    maintainer_email='jeremiaszwojciak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = tello_controller.controller:main',
            'PID = tello_controller.PID:main',
            'controller_pid = tello_controller.controller_pid:main',
            'controller_spin = tello_controller.controller_spin:main'

        ],
    },
)
