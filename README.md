# Proyecto de Integración: FreeRTOS, Micro-ROS y C++ para Control de Motor y Comunicación con PC

![Logo del Proyecto](/images/logo.png)

Bienvenido al repositorio del Proyecto de Integración entre FreeRTOS, Micro-ROS y C++ para el control de motores y la comunicación con ROS2. Este proyecto proporciona una solución completa y escalable para la gestión de tareas en tiempo real, la comunicación con dispositivos periféricos y la transmisión de datos a través de una red ROS 2.

## Descripción

El objetivo de este proyecto es demostrar la sinergia entre tres poderosas tecnologías:

- **FreeRTOS**: Un sistema operativo en tiempo real altamente eficiente que facilita la gestión de tareas concurrentes, ideal para sistemas embebidos.

- **Micro-ROS**: Una implementación de ROS 2 específicamente diseñada para dispositivos con recursos limitados, permitiendo la comunicación con otros nodos ROS de manera eficaz.

- **C++**: El lenguaje de programación elegido para aprovechar su capacidad de abstracción y modularidad, facilitando el desarrollo y mantenimiento del código.

## Características Destacadas

- Implementación de un sistema de tareas en tiempo real utilizando FreeRTOS para el control preciso del motor.

- Integración de Micro-ROS para establecer una comunicación bidireccional entre el dispositivo embebido y un PC a través de una red ROS 2.

- Desarrollo en C++ para aprovechar las ventajas del paradigma de programación orientada a objetos y garantizar un código limpio y modular.

- Documentación detallada, incluyendo guías de instalación, configuración y uso, para facilitar la adopción y contribución de la comunidad.

## Estructura del Repositorio
![Estructura](/images/Estructura.png)
- `/Motor_uROS_FREERTOS`: Carpeta principal donde se pueden crear proyectos.
   - `/Motorp_FRTOS`: Carpeta del proyecto para controlar el Motor.
      - `/Libreria microRos`: Carpeta que contiene la librería microROS
      - `/pico_micro_ros_SCARA.cpp`: Archivo principal (main)
      - `/CmakeLists.txt`: Archivo de configuración CMake para integrar la librería microROS en el archivo principal

   - `/FreeRTOS`: Libreria del sistema operativo FreeRTOS

   - `/CmakeLists.txt`: Archivo de configuración CMake para integrar la librería FreeRTOS en todos los proyectos.

   - `/pico_sdk_import.cmake`:  Este archivo es utilizado para importar el SDK de Raspberry Pi Pico en el proyecto.

## Contribuciones

¡Tus contribuciones son bienvenidas! Si deseas mejorar o expandir este proyecto, por favor, sigue nuestras pautas de contribución detalladas en [CONTRIBUTING.md](enlace_a_contributing.md).

## Licencia

Este proyecto está bajo la Licencia [Nombre_de_la_Licencia]. Consulta el archivo [LICENSE.md](enlace_a_license.md) para más detalles.
