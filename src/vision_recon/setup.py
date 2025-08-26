from setuptools import setup

package_name = 'vision_recon'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_scan_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arushi',
    maintainer_email='arushi@todo.todo',
    description='Vision-Bot multi-view reconstruction package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_saver = vision_recon.camera_saver:main',
            'move_and_capture = vision_recon.move_and_capture:main',
        ],
    },
)

