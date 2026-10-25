from glob import glob

from setuptools import setup

package_name = 'rm_flatness_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ziad Aboelkhir',
    maintainer_email='zezomoh951@gmail.com',
    description='AprilTag search, flatness pursuit and motor voltage logging (SCE 672 A2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector = rm_flatness_nav.tag_detector:main',
        ],
    },
)
