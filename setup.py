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
    package_data = {
        'pyri.robotics.robotics_jog_service': ['*.robdef','*.yml'],
        'pyri.robotics.robotics_motion_service': ['*.robdef','*.yml']
    },
    zip_safe=False,
    install_requires=[
        'pyri-common',
        'qpsolvers',
        'scipy',
        'toppra'
    ],
    tests_require=['pytest','pytest-asyncio'],
    extras_require={
        'test': ['pytest','pytest-asyncio']
    },
    entry_points = {
        'pyri.plugins.sandbox_functions': ['pyri-robotics-sandbox-functions=pyri.robotics.sandbox_functions:get_sandbox_functions_factory'],
        'pyri.plugins.device_type_adapter': ['pyri-robotics-type-adapter = pyri.robotics.device_type_adapter:get_device_type_adapter_factory'],
        'pyri.plugins.blockly': ['pyri-robotics-plugin-blockly=pyri.robotics.blockly:get_blockly_factory'],
        'console_scripts': ['pyri-robotics-jog-service = pyri.robotics.robotics_jog_service.__main__:main']
    }
)