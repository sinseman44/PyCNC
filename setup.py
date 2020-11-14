#!/usr/bin/env python

try:
    import pypandoc
    long_description = pypandoc.convert('README.md', 'rst')
except(IOError, ImportError):
    long_description = open('README.md').read()

from setuptools import setup, find_packages
setup(
    name="pycnc",
    version="2.0.1",
    packages=find_packages(),
    scripts=['pycnc'],

    # metadata for upload to PyPI
    author="sinseman44",
    author_email="sinseman44@gmail.com",
    description="PiPlot2D machine controller",
    long_description=long_description,
    license="MIT",
    keywords="CNC 3D printer robot raspberry pi plotter2D",
    url="https://github.com/sinseman44/PyCNC/",
)
