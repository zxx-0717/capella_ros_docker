from glob import glob
from setuptools import setup

package_name = 'capella_nav_and_carto_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_manager = capella_nav_and_carto_server.node_manager:main',
            'nav_and_carto_service_node = capella_nav_and_carto_server.nav_and_carto_service:main',
            'nav_client_node = capella_nav_and_carto_server.nav_client:main',
            'carto_client_node = capella_nav_and_carto_server.carto_client:main',
            'missionstate_node = capella_nav_and_carto_server.missionstate_node:main',
            'map_test_node = capella_nav_and_carto_server.map_test_node:main',
            'mapupdates_test_node = capella_nav_and_carto_server.mapupdates_test_node:main',
            'map_monitor_node = capella_nav_and_carto_server.map_monitor_node:main',
            'stop_client_node = capella_nav_and_carto_server.stop_client:main',
            'test_node = capella_nav_and_carto_server.test_node:main',
            'wifi_test_node = capella_nav_and_carto_server.wifi_test_node:main'
            'irdetection_node = capella_nav_and_carto_server.infrared_detection_service:main'
            'wifi_test_node = capella_nav_and_carto_server.wifi_test_node:main'
        ],
    },
)
