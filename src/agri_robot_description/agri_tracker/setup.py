# agri_robot_description/setup.py
from setuptools import setup, find_packages

setup(
    name='agri_robot_description',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'rclpy',
        'numpy',
    ],
    maintainer='wilsonchandra',
    maintainer_email='williveon1911@users.noreply.github.com',
    description='Pure Pursuit path tracking for agricultural robot',
    license='MIT',
)