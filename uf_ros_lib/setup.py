from setuptools import setup, find_packages

package_name = 'uf_ros_lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='vm',
    maintainer_email='vinman.cub@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # extras_require={
    #     'test': ['pytest'],
    # },
    entry_points={
        'console_scripts': [],
    },
)
