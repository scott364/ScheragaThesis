##### python setup.py build_ext --inplace

from distutils.core import setup, Extension

# define the extension module
nscan = Extension('nscan', sources=['nscan.cpp'])

# run the setup
setup(ext_modules=[nscan])