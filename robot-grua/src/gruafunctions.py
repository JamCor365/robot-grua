import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi


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
