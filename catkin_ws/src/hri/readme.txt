#Documentación y recursos necesarios para usar la interfaz

#Recursos a instalar

1) Tener instalado los siguientes recursos de ROS
usb_cam
web_video_server - sudo apt-get install ros-DISTRO-web-video-server

2) Ejecutar por el script de shell ubicado en carpeta FinderV3/toInstall/ 
- Sirve para modificar permisos de la cámara al momento de iniciar la NUC

3) Ejecutar roslaunch usb_cam usb_cam-test.launch 
- es para lanzar las cámaras

NOTA: Hay un launch que fue modificado para ejecutar todas las cámaras 
el link de esta documentación es:
https://answers.ros.org/question/218573/running-2-different-cameras-using-usb_cam/

NOTA2: dentro del link lo único distinto fue 
<param name="camera_frame_id" value="yuyv" />
    <param name="io_method" value="yuyv"/>




