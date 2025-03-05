from setuptools import setup

package_name = 'get_joystick_input'

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
    maintainer='pc',
    maintainer_email='zhupengcheng777@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={# entry_points 用于定义可执行的ROS2节点，让我们可以用ros2 run
        #来启动python的脚本
        'console_scripts': [
        # console_scripts 是一个setuptools 格式： ‘命令名称 = 包名.模块名：函数名‘
        'get_joystick_input = get_joystick_input.get_joystick_input:main'
        ],
    },
)
