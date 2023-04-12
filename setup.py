from setuptools import setup

package_name = 'ros2_webcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Marins Batista',
    maintainer_email='lucasmarbatista@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts' : [ 'ros2_webcam_node = ros2_webcam.ros2_webcam_pub:main',
        'ros2_video_node = ros2_webcam.ros2_video_pub:main',
        ],
    },
)
