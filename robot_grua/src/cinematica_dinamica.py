from gruafunctions import *

def menu():
 print("\n--- Plataforma de Control del Robot ---")
 print("Seleccione una opcion:")
 print("1. Cinematica Directa")
 print("2. Cinematica Inversa")
 print("3. Control Diferencial")
 print("4. Dinamica")
 print("5. Control Dinamico")
 print("6. Salir")
 opcion = input("Ingrese su eleccion: ")
 return opcion

if __name__ == '__main__':

 # Lectura del modelo del robot a partir de URDF (parsing)
 modelo = rbdl.loadModel('../urdf/robot_grua.urdf')
 # Grados de libertad
 ndof = modelo.q_size

 # Initial Joint Configuration
 q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
 # Velocidad inicial
 dq0 = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
 
 
 # Loop rate (in Hz)
 freq = 10

 # Simulador dinamico del robot
 #robot = Robot(q0, dq0, ndof, dt)
 
 # Se definen las ganancias del controlador
 Kp = np.diag([300.0, 400.0, 400.0, 500.0, 100.0, 100.0])  # Ajusta según la respuesta deseada
 Kd = np.diag([30.0, 10.0, 20.0, 10.0, 5.0, 5.0])  # Ajusta según la respuesta deseada
 
 
 while True:
 
  opcion = menu()
  
  if opcion == "1":  # Cinemática Directa
   print("\n--- CINEMATICA DIRECTA ---")
   try:
    print("Ingrese las 6 articulaciones actuales separados por espacio:")
    print("Rango: [-3.1, 0, -0.87, 0, -0.25, -3.1] to [3.1, 0.7, 0.52, 1, 0.25, 3.1]")
    q_input = input()  # Lee la entrada del usuario
    q = np.array([float(val) for val in q_input.split()])
    if len(q) != 6:
     raise ValueError("Las articulaciones son seis valores")

    print(f"Configuracion actual de las articulaciones: {np.round(q, 3)}")
    
    # End effector with respect to the base
    T = fkine(q)
    print( "Posicion (x,y,z): ",np.round(T[0:3,3], 3) )
    break

   except ValueError as e:
    print(f"Error: {e}")
    
  elif opcion == "2":  # Cinemática Inversa
   print("\n--- CINEMATICA INVERSA ---")
   try:
    print("Ingrese la posicion deseada (x, y, z) separados por espacio:")
    xd_input = input()  # Lee la entrada del usuario
    xd = np.array([float(val) for val in xd_input.split()])
    if len(xd) != 3:
     raise ValueError("La posicion deseada debe tener exactamente tres valores: x, y, z.")
     
    print(f"Posicion deseado del efector final: {np.round(xd, 3)}")

    # Inverse kinematics
    q = ikine(xd, q0)

    # Resulting position (end effector with respect to the base link)
    T = fkine(q)
    print( "Posicion actual del efector final: ",np.round(T[0:3,3], 3) )
    print(f"Configuracion actual de las articulaciones: {np.round(q, 3)}")
    break

   except ValueError as e:
    print(f"Error: {e}")
    
  elif opcion == "3":  # Control Diferencial
   print("\n--- CONTROL DIFERENCIAL ---")
   try:
    print("Ingrese la posicion deseada (x, y, z) y las ganancias k_pos, k_ori  separados por espacio:")
    v_input = input()  # Lee la entrada del usuario
    v = np.array([float(val) for val in v_input.split()])
    xd = v[0:3]
    k_pos = v[3]
    k_ori = v[4]
    q = ikine(xd, q0)
    T = fkine(q)
    xd = TF2xyzquat(T)
    if len(v) != 5:
     raise ValueError("Deben ser cinco valores: x, y, z, k_pos, k_ori.")
    
    # Control kinematics 
    q = diffkine(xd, q0, k_pos, k_ori, freq)

    print('Data cargada')
    break

   except ValueError as e:
    print(f"Error: {e}")
  
  elif opcion == "4":  # Dinamica
   print("\n--- DINAMICA ---")
   
   
   # Configuracion articular
   q = np.array([0.5, 0.2, 0.3, 0.8, 0.1, 0.6])
   print('Ejemplo de configuracion articular:\n', np.round(q,3))
   # Velocidad articular
   dq = np.array([0.8, 0.7, 0.8, 0.6, 0.1, 1.0])
   print('Ejemplo de velocidad articular:\n', np.round(dq,3))
   # Aceleracion articular
   ddq = np.array([0.2, 0.5, 0.4, 0.3, 0.1, 0.5])
   print('Ejemplo de aceleracion articular:\n', np.round(ddq,3))
   
   # Arrays numpy
   zeros = np.zeros(ndof)          # Vector de ceros
   tau   = np.zeros(ndof)          # Para torque
   g     = np.zeros(ndof)          # Para la gravedad
   c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
   M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
   e     = np.eye(6)               # Vector identidad
   
   # Torque dada la configuracion del robot
   rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
  
   # Calcular vector de gravedad
   rbdl.InverseDynamics(modelo, q, zeros, zeros, g)

   # Calcular vector de Coriolis y fuerzas centrífugas
   temp_tau = np.zeros(ndof)  # Vector temporal para cálculos intermedios
   rbdl.InverseDynamics(modelo, q, dq, zeros, temp_tau)
   c = temp_tau - g
  
   # Calcular matriz de inercia M
   for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i], temp_tau)
    M[:, i] = temp_tau - g
   
   print("\nVector de gravedad g(q):\n", np.round(g, 3))
   print("\nVector de Coriolis y fuerzas centrífugas c(q, dq):\n", np.round(c, 3))
   print("\nMatriz de inercia M(q):\n", np.round(M, 3))
   break
   
  elif opcion == "5":  # Control Dinamico
   print("\n--- CONTROL DINAMICO ---")
   try:
    print("Ingrese las 6 articulaciones deseadas separadas por espacio:")
    print("Rango: [-3.1, 0, -0.87, 0, -0.25, -3.1] to [3.1, 0.7, 0.52, 1, 0.25, 3.1]")
    qdes_input = input()  # Lee la entrada del usuario
    qdes = np.array([float(val) for val in qdes_input.split()])
    if len(qdes) != 6:
     raise ValueError("Las articulaciones son seis valores")

    print(f"Configuracion deseada de las articulaciones: {np.round(qdes, 3)}")
    
    # Ejecutar el control dinámico
    q, dq = dynamic_control(q0, dq0, qdes, Kp, Kd, modelo, freq)
    # Mostrar la configuración articular final
    print("Control dinámico completado.")
    print("Configuración articular final:", np.round(q, 3))
    print("Velocidades articulares finales:", np.round(dq, 3))   
    break

   except ValueError as e:
    print(f"Error: {e}")   
   
  elif opcion == "6":  # Salir
   print("Saliendo del programa...")
   break
   
  else:
   print("Opción inválida. Por favor, seleccione una opción válida.")
