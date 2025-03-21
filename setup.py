import os
from setuptools import find_packages, setup

package_name = 'usv_slam'

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['launch/', 'config/', 'rviz/', 'urdf/', 'models/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='am567',
    maintainer_email='alan.millard@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require={'test':['pytest'],},
    entry_points={
        'console_scripts': [
            'keyboard_control_node = usv_slam.keyboard_control_node:main',
            'pickup_node = usv_slam.pickup_node:main',
            'lidar_positioning_node = usv_slam.lidar_positioning_node:main',
            'localization_node = usv_slam.localization_node:main',
        ],
    },
)
