from setuptools import setup

setup(
    name='robom1_sim', version='0.1.0', packages=['robom1_sim'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/robom1_sim']),
                ('share/robom1_sim', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='daisuke', maintainer_email='daisuke@example.com',
    description='Lightweight badminton sensor simulator', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['scene_sim = robom1_sim.scene:main']},
)
