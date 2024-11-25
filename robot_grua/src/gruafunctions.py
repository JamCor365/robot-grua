import numpy as np
from copy import copy
import rbdl

cos=np.cos; sin=np.sin; pi=np.pi

class Robot(object):
 def __init__(self, q0, dq0, ndof, dt):
  self.q = q0    # numpy array (ndof x 1)
  self.dq = dq0  # numpy array (ndof x 1)
  self.M = np.zeros([ndof, ndof])
  self.b = np.zeros(ndof)
  self.dt = dt
  self.robot = rbdl.loadModel('./urdf/robot_grua.urdf')

 def send_command(self, tau):
  rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
  rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
  ddq = np.linalg.inv(self.M).dot(tau-self.b)
  self.q = self.q + self.dt*self.dq
  self.dq = self.dq + self.dt*ddq

 def read_joint_positions(self):
  return self.q

 def read_joint_velocities(self):
  return self.dq

def dh(d, theta, a, alpha):
 """
 Calcular la matriz de transformacion homogenea asociada con los parametros
 de Denavit-Hartenberg.
 Los valores d, theta, a, alpha son escalares.
 """
 # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
 T = np.array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])
 return T
 
def fkine(q):
 """
 Calcular la cinematica directa del robot grua dados sus valores articulares. 
 q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
 """
 # Longitudes (en metros)
 # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion

 # Parámetros DH del UR5 (valores ejemplo, ajusta según el modelo del UR5)
 d = [4, q[1], 0, q[3]+2.2, 0, 0.56]
 theta = [q[0], pi/2, q[2]+pi/2, pi, q[4]+pi/2, -q[5]]
 a = [0, 0, 0, 0, 0, 0]
 alpha = [0, pi/2, pi/2, pi/2, pi/2, 0]

 # Matrices de transformación para cada articulación
 T1 = dh(d[0], theta[0], a[0], alpha[0])
 T2 = dh(d[1], theta[1], a[1], alpha[1])
 T3 = dh(d[2], theta[2], a[2], alpha[2])
 T4 = dh(d[3], theta[3], a[3], alpha[3])
 T5 = dh(d[4], theta[4], a[4], alpha[4])
 T6 = dh(d[5], theta[5], a[5], alpha[5])

 # Multiplicación de las matrices de transformación para obtener la posición y orientación del efector final
 T = T1 @ T2 @ T3 @ T4 @ T5 @ T6

 return T
 
def jacobian(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad. 
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular 
 q=[q1, q2, q3, ..., qn]
 """
 # Crear una matriz 3xn
 n = q.size
 J = np.zeros((3,n))
 # Calcular la transformacion homogenea inicial (usando q)
 T = fkine(q)
    
 # Iteracion para la derivada de cada articulacion (columna)
 for i in range(n):
  # Copiar la configuracion articular inicial
  dq = copy(q)
  # Calcular nuevamenta la transformacion homogenea e
  # Incrementar la articulacion i-esima usando un delta
  dq[i] += delta
  # Transformacion homogenea luego del incremento (q+delta)
  T_inc = fkine(dq)
  # Aproximacion del Jacobiano de posicion usando diferencias finitas
  J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
 return J
 
def ikine(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir 
 de la configuracion articular inicial de q0. Emplear el metodo de newton.
 """
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001

 q  = copy(q0)
 for i in range(max_iter):
  # Transformación homogenea actual
  T = fkine(q)
  # Posición actual del efector final
  xcurr = T[0:3, 3]
  # Error entre la posición deseada y la posición actual
  error = xdes - xcurr
        
  # Condición de convergencia
  if np.linalg.norm(error) < epsilon:
   print(f"Converged in {i} iterations.")
   return q

  # Calcular el Jacobiano
  J = jacobian(q, delta)
  # Actualizar las articulaciones usando el método de Newton
  dq = np.linalg.pinv(J).dot(error)  # Pseudo-inversa del Jacobiano
  q += dq
        
 print("Max iterations reached without convergence.")  
 return q
 
def diffkine(xd, q0, k, freq):
 """
 Control diferencial para alcanzar una posición deseada, con registro de datos.
 Guarda los valores de posición actual, posición deseada y configuración articular en archivos.
 """
    
 epsilon  = 0.001
 max_iter = 1000
 # Configuración inicial
 q = copy(q0)
 dt = 1.0 / freq  # Paso de tiempo (derivado de la frecuencia)

 # Abrir archivos para guardar datos
 fxcurrent = open("./Data/Control_Diferencial/xcurrent.txt", "w")
 fxdesired = open("./Data/Control_Diferencial/xdesired.txt", "w")
 fq = open("./Data/Control_Diferencial/q.txt", "w")

 # Inicializar bucle de control
 for _ in range(max_iter):
  # Transformación homogénea actual
  T = fkine(q)
  x = T[0:3, 3]  # Posición actual del efector final

  # Error
  e = xd - x
  if np.linalg.norm(e) < epsilon:
   print("Error suficientemente pequeño, convergencia alcanzada.")
   break

  # Velocidad cartesiana deseada
  dx_star = -k * e

  # Jacobiano y su pseudo-inversa
  J = jacobian(q)
  J_pinv = np.linalg.pinv(J)

  # Calcular velocidades articulares
  dq = J_pinv @ dx_star

  # Actualizar configuración articular usando delta_t
  q += dt * dq

  # Guardar datos en los archivos
  fxcurrent.write(f"{x[0]} {x[1]} {x[2]}\n")
  fxdesired.write(f"{xd[0]} {xd[1]} {xd[2]}\n")
  fq.write(" ".join(map(str, q)) + "\n")

 # Cerrar los archivos
 fxcurrent.close()
 fxdesired.close()
 fq.close()

 return q

def dynamic_control(q0, dq0, qdes, Kp, Kd, modelo, freq):
 """
 Implementa un control dinámico para alcanzar una configuración articular deseada.
 Guarda datos de la simulación en archivos.
 """
 epsilon=0.001
 dt = 1.0 / freq  # Paso de tiempo
 ndof = len(q0)   # Grados de libertad
 q = copy(q0)  # Configuración inicial
 dq =copy(dq0)  # Velocidad inicial

 # Archivos para guardar datos
 fqact = open("./Data/Control_Dinamico/qactual.txt", "w")
 fqdes = open("./Data/Control_Dinamico/qdeseado.txt", "w")
 fxact = open("./Data/Control_Dinamico/xactual.txt", "w")
 fxdes = open("./Data/Control_Dinamico/xdeseado.txt", "w")
 ftau = open("./Data/Control_Dinamico/torques.txt", "w")

 # Posición deseada del efector final (x, y, z)
 xdes = fkine(qdes)[0:3, 3]

 t = 0.0  # Tiempo inicial
 while np.linalg.norm(qdes - q) > epsilon:
  # Gravedad
  g = np.zeros(ndof)
  rbdl.InverseDynamics(modelo, q, np.zeros(ndof), np.zeros(ndof), g)

  # Control de torque
  u = np.dot(Kp, (qdes - q)) - np.dot(Kd, dq) + g

  # Actualizar velocidades y posiciones articulares
  dq += dt * u
  q += dt * dq

  # Calcular posición actual del efector final
  x = fkine(q)[0:3, 3]

  # Guardar datos
  fxact.write(f"{t} {x[0]} {x[1]} {x[2]}\n")
  fxdes.write(f"{t} {xdes[0]} {xdes[1]} {xdes[2]}\n")
  fqact.write(f"{t} {' '.join(map(str, q))}\n")
  fqdes.write(f"{t} {' '.join(map(str, qdes))}\n")
  ftau.write(f"{t} {' '.join(map(str, u))}\n")

  # Actualizar tiempo
  t += dt

  # Limitar iteraciones (opcional)
  if t > 100:  # Máximo 100 segundos de simulación
   print("Límite de tiempo alcanzado.")
   break

 # Cerrar archivos
 fqact.close()
 fqdes.close()
 fxact.close()
 fxdes.close()
 ftau.close()

 print("Control dinámico completado y datos guardados.")
 return q, dq
 
def rot2quat(R):
 """
 Convertir una matriz de rotacion en un cuaternion

 Entrada:
  R -- Matriz de rotacion
 Salida:
  Q -- Cuaternion [ew, ex, ey, ez]

 """
 dEpsilon = 1e-6
 quat = 4*[0.,]

 quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
 if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
  quat[1] = 0.0
 else:
  quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
 if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
  quat[2] = 0.0
 else:
  quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
 if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
  quat[3] = 0.0
 else:
  quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

 return np.array(quat)


def TF2xyzquat(T):
 """
 Convert a homogeneous transformation matrix into the a vector containing the
 pose of the robot.

 Input:
  T -- A homogeneous transformation
 Output:
  X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
       is Cartesian coordinates and the last part is a quaternion
 """
 quat = rot2quat(T[0:3,0:3])
 res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
 return np.array(res)
