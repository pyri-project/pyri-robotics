from setuptools import setup, find_packages, find_namespace_packages

setup(
    name='pyri-robotics',
    version='0.1.0',
    description='PyRI Teach Pendant Robotics Plugin',
    author='John Wason',
    author_email='wason@wasontech.com',
    url='http://pyri.tech',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    zip_safe=False,
    install_requires=[
        'pyri-common'
    ],
    tests_require=['pytest','pytest-asyncio'],
    extras_require={
        'test': ['pytest','pytest-asyncio']
    },
    entry_points = {
        'pyri.plugins.sandbox_functions': ['pyri-robotics-sandbox-functions=pyri.robotics.sandbox_functions:get_sandbox_functions_factory'],
        'pyri.plugins.device_type_adapter': ['pyri-robotics-type-adapter = pyri.robotics.device_type_adapter:get_device_type_adapter_factory'],
        'console_scripts': ['pyri-robotics-jog-joint-service = pyri.robotics.jog_joint_service.__main__:main',
            'pyri-robotics-jog-cartesian-service = pyri.robotics.jog_cartesian_service.__main__:main']
    }
)