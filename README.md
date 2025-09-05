# Gémelo digital Robot Eddie
Este repositorio da continuidad al Trabajo de Fin de Grado, en el que se diseñó e implementó un gemelo digital funcional del robot móvil Eddie utilizando ROS 1 (Noetic Ninjemys) sobre Ubuntu 20.04. En aquella primera fase, el desarrollo se centró en la simulación y validación de las capacidades básicas de movilidad del gemelo digital.

El presente repositorio recoge la migración y adaptación de dicho proyecto a ROS 2, ampliando así su alcance y compatibilidad con las versiones más recientes del ecosistema robótico.

## Compilación del proyecto con `colcon`

1. Clonar el repositorio en el workspace de ROS 2:  

   ```bash
   cd ~/eddiebot_ros2_ws/src
   git clone https://github.com/Ignix98/eddiebot_ws_ROS2.git
   
   cd ~/eddiebot_ros2_ws
   colcon build

   source install/setup.bash

## Avances del proyecto
En esta versión basada en ROS 2 se han logrado los siguientes avances:
 - Funcionamiento básico en RVIZ2
    ```bash
    ros2 launch eddiebot_gazebo check_urdf.launch.py
 -
## Objetivos a futuro
La evolución de este proyecto contempla las siguientes etapas de desarrollo:  

- **Compatibilidad con Gazebo (Ignition)**: migrar la simulación del robot al nuevo entorno de Gazebo.
- **Integración de sensores**: añadir un sensor LiDAR y una cámara Kinect. 
- **Diseño de un controlador propio**: implementar un sistema de control que permita validar diferentes estrategias de movimiento y maniobrabilidad del robot.  
- **Localización y mapeo (SLAM)**: dotar al robot de capacidad de construir mapas del entorno y ubicarse en ellos en tiempo real.  
- **Path planning y motion planning**: incorporar técnicas de planificación de rutas y trayectorias, facilitando la navegación autónoma en entornos complejos.  
   
