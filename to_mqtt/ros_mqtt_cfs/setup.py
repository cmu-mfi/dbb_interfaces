from setuptools import setup, find_packages

setup(
    name='ros_mqtt_pi',
    version='0.1',
    packages=find_packages(),  # Automatically find all packages (e.g., 'core')
    install_requires=[         # Add dependencies from requirements.txt
        line.strip() for line in open('requirements.txt').readlines()
    ],
    entry_points={
        'console_scripts': [
            'mqtt_publisher=scripts.mqtt_publisher:main',  # Entry point for mqtt_publisher.py
        ]
    }
)
