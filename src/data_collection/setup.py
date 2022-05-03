from setuptools import setup

package_name = 'data_collection'

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
    maintainer='ziyi',
    maintainer_email='zl413@cam.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publish_reference_state = data_collection.publish_reference_state:main",
            "publish_reference_state_inverse = data_collection.inverse_sampling_publish:main",
            "publish_referenec_state_active = data_collection.active_sampling_publish:main",
        ],
    },
)
