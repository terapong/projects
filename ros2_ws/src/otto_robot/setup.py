from setuptools import find_packages, setup

package_name = 'otto_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toto',
    maintainer_email='toto@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'button_tester = otto_robot.button_tester:main',
            'led_control_gui = otto_robot.led_control_gui:main',
            'relay_control_gui = otto_robot.relay_control_gui:main',
            'segment7_control_gui = otto_robot.segment7_control_gui:main',
            'segment7_4_control_gui = otto_robot.segment7_4_control_gui:main',
            'dht_control_gui = otto_robot.dht_control_gui:main',
            'lcd_control_gui = otto_robot.lcd_control_gui:main',
            'rtc_control_gui = otto_robot.rtc_control_gui:main',
            'rfid_control_gui = otto_robot.rfid_control_gui:main',
            'water_sensor_control_gui = otto_robot.water_sensor_control_gui:main',
            'rgba_control_gui = otto_robot.rgba_control_gui:main',
            'servo_control_gui = otto_robot.servo_control_gui:main',
            'buzzer_control_gui = otto_robot.buzzer_control_gui:main',
            'stepper_control_gui = otto_robot.stepper_control_gui:main',
            'ldr_control_gui = otto_robot.ldr_control_gui:main',
            'potentiometer_control_gui = otto_robot.potentiometer_control_gui:main',
            'ultrasonic_control_gui = otto_robot.ultrasonic_control_gui:main',           
            'joystick_control_gui = otto_robot.joystick_control_gui:main',
        ],
    },
)
