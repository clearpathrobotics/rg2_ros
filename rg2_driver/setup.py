from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rg2_driver'],
    scripts=['scripts/rg2_ft_node'],
    package_dir={'': 'src'}
)

setup(**d)
