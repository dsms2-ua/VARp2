#!/bin/bash

# Colores para output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

CONTAINER_NAME=var_container_cpu
VNC_PORT=5900
VNC_PASS=davidet

echo -e "${YELLOW}Arrancando contenedor ROS Humble...${NC}"

# Levanta el contenedor en background
docker compose up -d

# Espera a que x11vnc esté listo
echo -e "${YELLOW}Esperando a que VNC esté listo...${NC}"
until docker exec $CONTAINER_NAME ps aux | grep -q "[x]11vnc"; do
    sleep 2
done

sleep 5

echo -e "${GREEN}Contenedor arrancado.${NC}"
echo -e "${GREEN}Conectando a VNC en localhost:${VNC_PORT} (contraseña: ${VNC_PASS})${NC}"

# Abre VNC con el cliente nativo del Mac
open vnc://localhost:$VNC_PORT

# Entra al contenedor
docker exec -it $CONTAINER_NAME bash