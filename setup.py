from setuptools import setup
from setuptools import find_packages

setup(
    name='agri_robot_description',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_cmake_python/cmake/ament_cmake_python',
         ['resource/agri_robot_description']),
        ('share/agri_robot_description', ['package.xml']),
        ('share/agri_robot_description/urdf',
         ['urdf/agri_robot_base.urdf.xacro']),
        ('share/agri_robot_description/meshes',
         ['meshes/wheel.stl', 'meshes/chassis.stl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your_email@example.com',
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    url='https://github.com/your_user/agri_robot_ws',
    description='URDF and USD robot description',
    license='MIT',
)