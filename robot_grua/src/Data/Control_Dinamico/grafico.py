import matplotlib.pyplot as plt
import numpy as np

# Cargar datos desde los archivos
data_actual = np.loadtxt("./xactual.txt")
data_deseado = np.loadtxt("./xdeseado.txt")

# Graficar las posiciones x, y, z
plt.figure()
plt.plot(data_actual[:, 0], data_actual[:, 1], label="x actual")
plt.plot(data_deseado[:, 0], data_deseado[:, 1], label="x deseado", linestyle="--")
plt.plot(data_actual[:, 0], data_actual[:, 2], label="y actual")
plt.plot(data_deseado[:, 0], data_deseado[:, 2], label="y deseado", linestyle="--")
plt.plot(data_actual[:, 0], data_actual[:, 3], label="z actual")
plt.plot(data_deseado[:, 0], data_deseado[:, 3], label="z deseado", linestyle="--")

# Configuración del gráfico
plt.legend()
plt.title("Evolución temporal de las coordenadas x, y, z del efector final")
plt.xlabel("Tiempo [s]")
plt.ylabel("Posición [m]")
plt.grid()
plt.show()
