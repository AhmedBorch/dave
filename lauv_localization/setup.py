# from setuptools import find_packages, setup

# package_name = 'lauv_localization'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     package_data={
#         # Include launch and config files
#         package_name: ['../launch/*.launch.py', '../config/*.yaml'],
#     },
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='root',
#     maintainer_email='borcheni.ahmed99@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )

from setuptools import setup

package_name = 'lauv_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Localization nodes for LAUV',
    license='Apache-2.0',
    entry_points={
        # No executables needed
    },
    data_files=[
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),
        # Install config files
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
)