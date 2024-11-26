# Robot Grua

Este repositorio contiene la implementación de un modelo de robot grúa en ROS 1, incluyendo configuraciones, nodos y lanzadores necesarios para simular y visualizar el robot en RViz y Gazebo.

## Instalación

### Creación del workspace
Para comenzar, sigue estos pasos para crear el workspace y configurar correctamente el proyecto:

1. **Crea el workspace y la carpeta `src`**:
   ```bash
   mkdir -p <nombre_workspace>_ws/src
   cd <nombre_workspace>_ws/src
   ```
   Es **muy importante** que el directorio donde se clonará el repositorio se llame `src` (y no otro nombre), ya que es el estándar para los workspaces de ROS.

2. **Clona el repositorio e instala las dependencias**:
   ```bash
   git clone https://github.com/JamCor365/robot-grua.git . --depth=1 --single-branch && chmod +x setup.sh && ./setup.sh
   ```

3. **Configuración automática**:
   El script `setup.sh` realiza las siguientes acciones:
   - Compila el workspace utilizando `catkin_make`.
   - Clona y configura el repositorio `rbdl` dentro del directorio `src`.
   - Agrega las rutas necesarias al archivo `~/.bashrc`:
     - `source` del workspace.
     - Configuración de `PYTHONPATH` para RBDL.
   - Aplica automáticamente las configuraciones a tu sesión actual.

---

## Estructura del proyecto

El repositorio está organizado en las siguientes carpetas principales:

### 1. `config/`
   - Contiene los archivos de configuración necesarios para el correcto funcionamiento del robot.
   - Configuración de RViz para visualizar el robot.
   - Archivos `.yaml` que especifican los controladores de Gazebo para el robot.

### 2. `launch/`
   - Incluye los archivos de lanzamiento (`.launch`) para ejecutar el robot en diferentes entornos.
   - Lanzadores para iniciar Gazebo con el modelo del robot.
   - Lanzadores para abrir RViz con la configuración adecuada del robot grúa.

### 3. `urdf/`
   - Contiene el modelo del robot grúa, descrito en formato URDF.
   - Incluye la estructura del robot, enlaces, articulaciones y propiedades físicas.

### 4. `src/`
   - En este directorio se encuentran los nodos implementados para controlar y operar el robot.
   - Aquí se añadirán futuros desarrollos y funcionalidades relacionadas con la lógica del robot grúa.

---

## Uso del proyecto

### Visualización en RViz
Para visualizar el robot en RViz, utiliza el siguiente comando (desde el directorio del workspace):

```bash
roslaunch <nombre_paquete> <archivo_rviz>.launch
```

### Simulación en Gazebo
Para simular el robot en Gazebo, ejecuta:

```bash
roslaunch <nombre_paquete> <archivo_gazebo>.launch
```

---

## Notas adicionales

- Asegúrate de tener instaladas las dependencias de ROS 1 necesarias para trabajar con RViz, Gazebo y controladores.
- Este proyecto está en desarrollo y se expandirá con nuevos nodos y funcionalidades en futuras actualizaciones.
