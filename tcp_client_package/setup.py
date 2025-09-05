from setuptools import find_packages, setup

package_name = 'tcp_client_package'

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
    maintainer='karlo',
    maintainer_email='karlo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'tcp_client = tcp_client_package.tcp_client:main',
            'tcp_client_node = tcp_client_package.tcp_client_node:main'
        ],
    },
)
