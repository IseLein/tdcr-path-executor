from setuptools import setup, Extension
import pybind11
import numpy
import os
import sys

# Check if libfranka is available
def check_libfranka():
    """Check if libfranka headers are available"""
    search_paths = [
        '/usr/include/franka',
        '/usr/local/include/franka',
    ]
    for path in search_paths:
        if os.path.exists(os.path.join(path, 'robot_state.h')):
            return True
    return False

# Only build extension if libfranka is available
ext_modules = []
if check_libfranka():
    ext_modules = [
        Extension(
            'csc376_franky',
            [
                'python/bind_franky.cpp',
                'src/franka_joint_trajectory_controller.cpp',
                'src/gripper.cpp',
            ],
            include_dirs=[
                'include/',
                pybind11.get_include(),
                numpy.get_include(),
                '/usr/include/eigen3',
                '/usr/local/include/eigen3',
                '/opt/homebrew/include/eigen3'
            ],
            libraries=['franka'],
            library_dirs=['/usr/lib', '/usr/local/lib'],
            language='c++',
            extra_compile_args=['-std=c++17', '-O3'],
        ),
    ]
    print("Building with libfranka support for robot execution.")
else:
    print("\n" + "="*70)
    print("WARNING: libfranka not found - building WITHOUT robot support")
    print("You can still use simulation mode with --simulate-only")
    print("Robot execution features will not be available.")
    print("="*70 + "\n")

setup(
    name='csc376_franky',
    ext_modules=ext_modules,
    zip_safe=False,
)
