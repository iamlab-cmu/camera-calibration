"""
FrankaPy camera calibration software.
"""
from setuptools import setup

requirements = [
    'autolab_core',
    'empy',
    'numpy',
    'numpy-quaternion',
    'numba',
    'rospkg',
    'catkin-tools',
    'click',
    'pandas',
]

# TODO: Update to poetry
setup(name='camera_calib',
      version='1.0.0',
      description='Camera Calibration using FrankaPy',
      author='Intelligent Autonomous Manipulation Lab - Carnegie Mellon University',
      author_email='',
      package_dir = {'': '.'},
      packages=['camera_calib'],
      install_requires = requirements,
      extras_require = {}
     )
