from setuptools import setup

requirements = []

setup(name='domino_full_stack',
      version='0.0.0',
      description='Package for Domino Robot',
      author='Liam Parrish',
      author_email='liam_parrish@berkeley.edu',
      package_dir = {'': 'src'},
      packages=['domino_vision_pkg', 'ar_track_alvar', 'game_planner', 'gripper_ctrl', 'lab4_cam', 'robot_ctrl'],
      install_requires=requirements,
      test_suite='test'
     )