from setuptools import setup

package_name = 'battery_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Simple battery model',
    license='MIT',
    entry_points={
        'console_scripts': [
            'battery_node = battery_model.battery_node:main',
        ],
    },
)
