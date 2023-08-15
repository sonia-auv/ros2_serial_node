from setuptools import setup

package_name = 'serial_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='relat',
    maintainer_email='relat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node_stream = serial_node.serial_node_stream:main',
            'serial_node_server = serial_node.serial_node_server:main',
            'serial_node_client = serial_node.serial_node_client:main'
        ],
    },
)
