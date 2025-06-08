from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/data_files', [
            'controller/index.html',
            'controller/certificate.pem',
            'controller/key.pem',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yukichi6105',
    maintainer_email='107849799+YUKICHI6105@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = controller.controller:main',
        ],
    },
)
