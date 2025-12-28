from setuptools import setup

package_name = 'coverage_mapper'

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
    description='Coverage mapping node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coverage_node = coverage_mapper.coverage_node:main',
        ],
    },
)
