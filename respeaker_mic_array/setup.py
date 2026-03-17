from setuptools import setup, find_packages

package_name = 'respeaker_mic_array'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/respeaker_mic_array']),
        ('share/respeaker_mic_array', ['package.xml']),
        ('share/respeaker_mic_array/launch', ['launch/respeaker.launch.py']),
    ],
    package_data={
        'respeaker_mic_array': [
            'usb_4_mic_array/*.bin',
            'usb_4_mic_array/*.cfg',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thawatchai',
    maintainer_email='thawatchai@todo.todo',
    description='ReSpeaker USB 4-Mic Array ROS2 driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'doa_node = respeaker_mic_array.doa_node:main',
            'vad_node = respeaker_mic_array.vad_node:main',
            'audio_node = respeaker_mic_array.audio_node:main',
            'full_dsp_tuning_node = respeaker_mic_array.full_dsp_tuning_node:main',
            'graph_node = respeaker_mic_array.graph_node:main',
            'audio_listener_node = respeaker_mic_array.audio_listener_node:main',
            'google_stt = respeaker_mic_array.google_stt_node:main',



            
        ],
    },
)
