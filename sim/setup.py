from setuptools import setup, find_packages

setup(
    name='ur5e_iff_sim',  # Name of your package/project
    version='0.1',  # Version number
    description='A package for controlling UR5e with ROS and Gazebo',  # Short description
    author='Mohammed Azharudeen Farook Deen',  # Replace with your name
    author_email='fmdazhar@gmail.com',  # Replace with your email
    packages=find_packages(),  # Automatically find all packages (i.e., folders with __init__.py)
    install_requires=[
        'pyserial',
        'pymodbus==2.5.3',
        'numpy==1.19.5',
        'scipy',
        'numpy-quaternion',
        'open3d>=0.8.0',
        'pyspacemouse'
    ],

)