import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dirt_cleaner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world') + glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models/dirt_patch'), [
            'models/dirt_patch/model.config',
            'models/dirt_patch/model.sdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lxmix',
    maintainer_email='lamia.koudri@g.enp.edu.dz',
    description='SmartVacuum dirt cleaning robot',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'semantic_detector   = dirt_cleaner.semantic_detector:main',
            'dirt_detector_node  = dirt_cleaner.dirt_detector_node:main',
             #'autonomous_explorer = dirt_cleaner.autonomous_explorer:main',
            'coverage_explorer  = dirt_cleaner.coverage_explorer:main',
        ],
    },
)
