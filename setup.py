from setuptools import find_packages, setup

package_name = 'coverage_planner'

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
    maintainer='yzksy',
    maintainer_email='d22102@numazu.kosen-ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
        'console_scripts': [
            'dummy_area_publisher = coverage_planner.dummy_area_publisher:main',
            'zigzag_generator = coverage_planner.zigzag_generator:main',
            'rectangle_generator = coverage_planner.rectangle_generator:main',
            'two_cone_reciprocate = coverage_planner.two_cone_reciprocate:main',
        ],
    },
)
