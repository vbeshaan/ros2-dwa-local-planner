from setuptools import find_packages, setup

package_name = 'dwa_local_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eshaan',
    maintainer_email='eshaan@example.com',
    description='Custom DWA local planner (smoke test + planner)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smoke_test = dwa_local_planner.smoke_test:main',
            'dwa_planner = dwa_local_planner.dwa_planner:main',
        ],
    },
)
