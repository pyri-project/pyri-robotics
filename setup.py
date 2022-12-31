from setuptools import setup, find_packages, find_namespace_packages

setup(
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    package_data = {
        'pyri.robotics.robotics_jog_service': ['*.robdef','*.yml'],
        'pyri.robotics.robotics_motion_service': ['*.robdef','*.yml']
    },
    zip_safe=False
)