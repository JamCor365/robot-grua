import numpy as np
import matplotlib.pyplot as plt

# Leer datos de los archivos
def read_data(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split()))
            data.append(values)
    return np.array(data)

# Archivos de datos
file_current = "./xcurrent.txt"
file_desired = "./xdesired.txt"

# Leer posiciones actuales y deseadas
xcurrent = read_data(file_current)
xdesired = read_data(file_desired)

# Graficar resultados
time = np.arange(0, len(xcurrent)) * (1 / 50)  # Tiempo basado en la frecuencia (50 Hz)

plt.figure(figsize=(10, 6))
plt.plot(time, xcurrent[:, 0], label="Posición actual (x)", linestyle="-")
plt.plot(time, xdesired[:, 0], label="Posición deseada (x)", linestyle="--")
plt.plot(time, xcurrent[:, 1], label="Posición actual (y)", linestyle="-")
plt.plot(time, xdesired[:, 1], label="Posición deseada (y)", linestyle="--")
plt.plot(time, xcurrent[:, 2], label="Posición actual (z)", linestyle="-")
plt.plot(time, xdesired[:, 2], label="Posición deseada (z)", linestyle="--")

plt.title("Control Diferencial: Trayectorias de Posición")
plt.xlabel("Tiempo (s)")
plt.ylabel("Posición (m)")
plt.legend()
plt.grid(True)
plt.show()

