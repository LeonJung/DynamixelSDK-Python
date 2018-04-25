
from setuptools import setup, find_packages

setup(
    name='dynamixel_sdk',
    version='3.6.0',
    packages=find_packages('dynamixel_sdk', exclude=['tests*']),
    package_dir={'dynamixel_sdk': 'src/dynamixel_sdk'},
    license='Apache 2.0',
    description='Dynamixel SDK package',
    long_description=open('README.txt').read(),
    url='https://github.com/ROBOTIS-GIT/DynamixelSDK',
    author='Leon Jung',
    author_email='rwjung@robotis.com'
)
