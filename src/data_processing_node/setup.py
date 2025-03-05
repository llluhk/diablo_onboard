from setuptools import setup

package_name = 'data_processing_node'

setup(
    name=package_name,
    version='0.1.0',  # 更新版本号
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],  # 添加必要的依赖
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='A ROS2 node for data processing using sliding window technique.',
    license='Apache License 2.0',  # 指定开源许可证
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注册节点入口
            'data_processing_node = data_processing_node.data_processing_node:main',
        ],
    },
)
