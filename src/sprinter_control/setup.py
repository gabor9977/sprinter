from setuptools import setup, find_packages

package_name = 'sprinter_control'

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='RViz-only gait, TF motion, and finish-line markers for a two-leg sprinter',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            # 'gait_jointstate = sprinter_control.gait_jointstate:main',
            'bolt_motion = sprinter_control.bolt_motion:main',
            'finish_line_markers = sprinter_control.finish_line_markers:main',
            'sprinter = sprinter_control.sprinter:main',
            # 'sprint_controller = sprinter_control.sprint_controller:main',
            'distance_sensor = sprinter_control.distance_sensor:main',
        ],
    },
)
