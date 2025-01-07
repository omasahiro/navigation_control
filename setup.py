from setuptools import setup
import os
from glob import glob

package_name = 'navigation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'pyyaml',
        'python-multipart'
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Navigation control package for robot patrol',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_control = navigation_control.navigation_control_node:main',
            'web_config = navigation_control.web_config_node:main'  # 新しいエントリーポイントを追加
        ],
    },
)