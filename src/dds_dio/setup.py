from setuptools import setup

package_name = 'dds_dio'

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
    maintainer='fakhrul',
    maintainer_email='fakhrulazran@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = dds_dio.dds_dio_server:main',
            'client = dds_dio.dds_dio_client:main',
        ],
    },
)
