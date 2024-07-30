from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

extensions = [
    Extension("filters", ["filters.pyx"],
              include_dirs=[numpy.get_include()]),
    Extension("arm_tracker_cy", ["arm_tracker_cy.pyx"],
              include_dirs=[numpy.get_include()]),
    Extension("xarm_kinematics", ["xarm_kinematics.pyx"],
              include_dirs=[numpy.get_include()])
]

setup(
    name="XArm6ControlSystem",
    ext_modules=cythonize(extensions, language_level=3),
    install_requires=[
        "numpy",
        "opencv-python",
        "mediapipe",
        "pyrealsense2",
        "xarm-python-sdk",
    ],
)