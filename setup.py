from setuptools import setup

package_name = 'f1tenth_gym_rl'

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
    maintainer='ingu',
    maintainer_email='cik0418@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'f1tenth_gym_rl_node = f1tenth_gym_rl.f1tenth_gym_rl_node:main',
            'f1tenth_gym_rl_sub_node = f1tenth_gym_rl.f1tenth_gym_rl_sub_node:main',
            'f1tenth_gym_rl_pub_node = f1tenth_gym_rl.f1tenth_gym_rl_pub_node:main'
        ],
    },
)
