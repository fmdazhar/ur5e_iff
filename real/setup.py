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
    ],
)