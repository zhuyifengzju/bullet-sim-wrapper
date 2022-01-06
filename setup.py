from setuptools import setup, find_packages

# read the contents of your README file
from os import path
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    lines = f.readlines()

# remove images from README
lines = [x for x in lines if '.png' not in x]
long_description = ''.join(lines)

setup(
        name="bullet_world",
    packages=[
        package for package in find_packages() if package.startswith("bullet_world")
    ],
    install_requires=[
    ],
    eager_resources=['*'],
    include_package_data=True,
    python_requires='>=3',
    description="Simple Bullet Sim Warpper",
    author="Yifeng Zhu",
    author_email="yifeng.zhu@utexas.edu",
    version="0.1.0",
    long_description=long_description,
    long_description_content_type='text/markdown'
)
