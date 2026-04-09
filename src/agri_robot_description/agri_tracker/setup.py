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
    entry_points={
        'console_scripts': [
            'pure_pursuit_tracker = agri_tracker.tracker:main',
            'fake_odom_publisher = agri_tracker.fake_odom_publisher:main',
            'data_collector = agri_tracker.data_collector:main',
            'tracker_evaluator = agri_tracker.evaluator:main',
            'tracker_tuner = agri_tracker.tuner:main',
            'tracker_visualizer = agri_tracker.visualizer:main',
        ],
    },
    
    maintainer='wilsonchandra',
    maintainer_email='williveon1911@users.noreply.github.com',
    description='Pure Pursuit path tracking for agricultural robot',
    license='MIT',
)