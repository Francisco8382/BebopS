[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# BebopS

Este proyecto es una bifurcación de [BebopS](https://github.com/gsilano/BebopS), del cual se hicieron los siguientes cambios:

* Añadido nodo para guardar mensajes de odometría en archivos CSV
* Añadido nodo para guardar las entradas virtuales en archivos CSV
* Añadido nodo para generación de trayectorias
* Modificación del controlador PD
* Añadido controlador PID
* Añadido controlador CITC
* Añadido lanzador para la sintonización de ganancias del controlador PD
* Añadido lanzador para la sintonización de ganancias del controlador PID
* Añadido lanzador para la sintonización de ganancias del controlador CITC

Los siguientes archivos fueron añadidos:
  * /include/bebop\_simulator/pos\_controller.h
  * /msg/AttitudeController.msg
  * /msg/Control.msg
  * /msg/PosController.msg
  * /msg/ReferenceAngles.msg
  * /launch/tarea\_1.launch
  * /launch/tarea\_2.launch
  * /launch/tarea\_3.launch
  * /launch/sintonizacion\_1.launch
  * /launch/sintonizacion\_2.launch
  * /launch/sintonizacion\_3.launch
  * /resource/controller\_bebop.yaml
  * /resource/controller\_bebop\_launcher.yaml
  * /resource/controller\_bebop\_des.yaml
  * /resource/controller\_bebop\_des\_launcher.yaml
  * /resource/controller\_bebop\_stc.yaml
  * /resource/controller\_bebop\_stc\_launcher.yaml
  * /resource/controller\_bebop\_default.yaml
  * /scripts/plot.py
  * /scripts/find\_MSE.py
  * /src/library/pos\_controller.cpp
  * /src/library/pos\_controller\_2.cpp
  * /src/library/pos\_controller\_3.cpp
  * /src/nodes/csv\_odometry.cpp
  * /src/nodes/csv\_control.cpp
  * /src/nodes/launcher.cpp
  * /src/nodes/launcher\_2.cpp
  * /src/nodes/launcher\_3.cpp
  * /src/nodes/pos\_controller\_node.cpp
  * /src/nodes/pos\_controller\_node.h
  * /src/nodes/pos\_controller\_node\_2.cpp
  * /src/nodes/pos\_controller\_node\_2.h
  * /src/nodes/pos\_controller\_node\_3.cpp
  * /src/nodes/pos\_controller\_node\_3.h
  * /src/nodes/waypoint\_gen.cpp
