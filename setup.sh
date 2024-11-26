#!/bin/bash

# Función para mostrar mensajes con colores
log() {
  echo -e "\033[1;32m$1\033[0m"
}

error() {
  echo -e "\033[1;31m$1\033[0m"
}

log "Iniciando configuración del workspace..."

# Obtener el directorio donde está ubicado este script
WORKSPACE_DIR=$(pwd)
SRC_DIR="$WORKSPACE_DIR/src"
INSTALL_DIR="$WORKSPACE_DIR/install"

# 1. Regresar al directorio principal del workspace y compilar con catkin_make
log "Ejecutando catkin_make en $WORKSPACE_DIR..."
cd "$WORKSPACE_DIR" || { error "No se pudo acceder al directorio $WORKSPACE_DIR"; exit 1; }
catkin_make || { error "Error durante catkin_make"; exit 1; }

# 2. Clonar el repositorio rbdl dentro de src/
log "Clonando el repositorio rbdl en $SRC_DIR..."
cd "$SRC_DIR" || { error "No se pudo acceder al directorio $SRC_DIR"; exit 1; }
if [ ! -d "rbdl" ]; then
  git clone https://github.com/alexwbots/rbdl.git || { error "Error al clonar el repositorio rbdl"; exit 1; }
else
  log "El repositorio rbdl ya existe."
fi

# 3. Configurar y construir rbdl
log "Configurando y construyendo rbdl..."
cd rbdl || { error "No se pudo acceder al directorio rbdl"; exit 1; }
mkdir -p build
cd build || { error "No se pudo acceder al directorio build"; exit 1; }
cmake .. -DRBDL_BUILD_PYTHON_WRAPPER=ON -DRBDL_BUILD_ADDON_URDFREADER=ON -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" || { error "Error durante cmake"; exit 1; }
make || { error "Error durante make"; exit 1; }
make install || { error "Error durante make install"; exit 1; }

# 4. Configurar el archivo ~/.bashrc
log "Configurando ~/.bashrc..."
BASHRC_ENTRY="source $WORKSPACE_DIR/devel/setup.bash"
PYTHONPATH_ENTRY="export PYTHONPATH=\$PYTHONPATH:$SRC_DIR/rbdl/build/python"

if ! grep -Fxq "$BASHRC_ENTRY" ~/.bashrc; then
  echo "$BASHRC_ENTRY" >> ~/.bashrc
  log "Se agregó la configuración del workspace a ~/.bashrc"
else
  log "El workspace ya estaba configurado en ~/.bashrc"
fi

if ! grep -Fxq "$PYTHONPATH_ENTRY" ~/.bashrc; then
  echo "$PYTHONPATH_ENTRY" >> ~/.bashrc
  log "Se agregó PYTHONPATH a ~/.bashrc"
else
  log "PYTHONPATH ya estaba configurado en ~/.bashrc"
fi

# 5. Aplicar los cambios en la sesión actual
log "Aplicando configuración en la sesión actual..."
source ~/.bashrc || { error "Error al cargar ~/.bashrc"; exit 1; }

log "¡Configuración completada con éxito!"
