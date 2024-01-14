from setuptools import find_packages, setup

package_name = 'reactive_nav'

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
    maintainer='Marcos Hidalgo',
    maintainer_email='marcoshidalgobanos@gmail.com',
    description='Potential Fields implementation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'potential_fields_nav = reactive_nav.potential_fields_nav:main'
        ],
    },
)
