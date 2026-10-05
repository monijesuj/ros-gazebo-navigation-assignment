from setuptools import setup
from glob import glob
from pathlib import Path
setup(name='robot_demo',version='1.0.0',packages=['robot_demo'],
 data_files=[('share/ament_index/resource_index/packages',['resource/robot_demo']),('share/robot_demo',['package.xml'])]+[(f'share/robot_demo/{d}',[f for f in glob(f'{d}/*') if Path(f).is_file()]) for d in ['launch','config','urdf','worlds']]+[('share/robot_demo/models/rover',glob('models/rover/*'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='Robot demo',maintainer_email='robot-demo@example.com',description='GPS waypoint navigation with lidar and depth safety',license='MIT',
 entry_points={'console_scripts':['navigator = robot_demo.navigation:main','monitor = robot_demo.monitor:main']})
