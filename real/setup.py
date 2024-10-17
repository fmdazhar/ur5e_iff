from setuptools import setup, find_packages

setup(
    name='ur5e_iff_real',  # Name of your package/project
    version='0.1',  # Version number
    description='A package for controlling UR5e with cameras and grippers',
    author='Mohammed Azharudeen Farook Deen',  # Replace with your name
    author_email='fmdazhar@gmail.com',  # Replace with your email
    packages=find_packages(),  # Automatically find all packages (i.e., folders with __init__.py)
    install_requires=[
    "gym>=0.26",
    "pymodbus==2.5.3",
    "opencv-python",
    "pyquaternion",
    "pyspacemouse",
    "hidapi",
    "pyyaml",
    "rospkg",
    "scipy",
    "requests",
    "flask",
    "defusedxml",
    "pynput",
    "yacs>=0.1.6",
    "gym>=0.14.0",
    "numpy==1.19.5",
    "colorlog>=4.0.2",
    "pytest==4.6.7;python_version<='2.7'",
    "pytest>=5.3.2;python_version>'2.7'",
    "pytest-cov>=2.6.1",
    "pytest-html==1.22.1;python_version<='2.7'",
    "pytest-html>=2.0.1;python_version>'2.7'",
    "pyassimp>=4.1.4",
    "pytest-lazy-fixture>=0.6.1",
    "absl-py>=0.8.1",
    "sphinx-rtd-theme>=0.4.3",
    "sphinx>=1.8.5;python_version>'2.7'",
    "sphinx==1.8.5;python_version<='2.7'",
    "sphinxcontrib-napoleon>=0.7",
    "recommonmark>=0.6.0",
    "opencv-python==4.2.0.32;python_version<='2.7'",
    "opencv-python>=4.2.0.32;python_version>'2.7'",
    "more_itertools==5.0.0;python_version<='2.7'",
    "pyrsistent==0.14.0;python_version<='2.7'",
    "open3d>=0.8.0"
    ],

)