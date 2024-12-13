from setuptools import setup
import os
from glob import glob


package_name = 'final_project_pkg'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'google-auth', 'google-auth-oauthlib', 'google-auth-httplib2', 'google-api-python-client', 'requests'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

	   'data_uploader = final_project_pkg.data_uploader:main',
            'person_follower = final_project_pkg.person_follower:main',
	    'test_camera = final_project_pkg.test_node:main',
            'hand_detection_node = final_project_pkg.handDetectionCode:main'
         ],
    },
)
