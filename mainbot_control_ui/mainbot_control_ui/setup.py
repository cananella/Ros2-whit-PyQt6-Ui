from setuptools import setup
from setuptools import find_packages

package_name = 'mainbot_control_ui'


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
    maintainer='paintingzoo',
    maintainer_email='paintingzoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mainbot_control = mainbot_control_ui.mainbot_control:main',

        ],
    },
)
