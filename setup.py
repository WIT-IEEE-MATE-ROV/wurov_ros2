from setuptools import setup

package_name = 'wurov'

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
    maintainer='Wurov',
    maintainer_email='ieee@wit.edu',
    description='WUROV Control Software Forked and Based off of Enbarr',
    license='PublicLicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nineDof = wurov.simulate_nineDof:main'
        ],
    },
)
