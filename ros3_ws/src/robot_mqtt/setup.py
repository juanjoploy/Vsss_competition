from setuptools import setup

package_name = "robot_mqtt"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools", "paho-mqtt"],
    zip_safe=True,
    maintainer="tu_usuario",
    maintainer_email="tu_email@example.com",
    description="Nodo que comunica ROS 2 con ESP32 usando MQTT",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_mqtt = robot_mqtt.mqtt_publisher:main",
            "movement_node = robot_mqtt.movement_node:main",
        ],
    },
)
