from setuptools import find_packages, setup

package_name = 'p3_ekf_adr'

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
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_estimation_3d = p3_ekf_adr.ekf_3d_state_estimation:main',
            'ekf_estimation_7d = p3_ekf_adr.ekf_7d_state_estimation:main',
            'ekf_estimation_8d = p3_ekf_adr.ekf_8d_state_estimation:main',
        ],
    },
)
