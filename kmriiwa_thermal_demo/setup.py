from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['kmriiwa_thermal_demo'], package_dir={'': 'src'}))
