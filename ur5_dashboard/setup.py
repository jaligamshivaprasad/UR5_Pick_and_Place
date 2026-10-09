from pathlib import Path
from setuptools import setup

PACKAGE = 'ur5_dashboard'
setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/web', list(map(str, Path('web').glob('*')))),
        ('share/' + PACKAGE + '/scripts', ['scripts/start_dashboard.sh']),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn'],
    zip_safe=True,
    entry_points={'console_scripts': ['ur5_dashboard = ur5_dashboard.server:main']},
)
