# ROS 2 Jazzy para AttaBot.
#
# Arch no tiene binarios de ROS y va adelante en Python (3.14 vs los 3.12 que
# espera Jazzy), asi que ROS vive aca adentro y habla con la Base por UDP.
# Con --network=host el contenedor ve el :6060 del host sin configurar nada,
# venga de la camara real o de Webots.
FROM osrf/ros:jazzy-desktop

# Drivers DRI de Mesa para la iGPU Intel (UHD 620) + glxinfo para diagnostico.
# La NVIDIA MX250 de esta maquina no tiene modulo cargado; se ignora a proposito.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1-mesa-dri \
        libglx-mesa0 \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# La base noble ya trae el usuario 'ubuntu' con uid/gid 1000, igual que el host,
# asi que lo que se escriba en los volumenes no queda de root.
USER ubuntu
WORKDIR /ws

# El entrypoint de la imagen ya hace source de /opt/ros/jazzy/setup.bash.
CMD ["bash"]
