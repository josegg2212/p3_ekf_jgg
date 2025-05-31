# p3\_ekf\_jgg

## Ejecución del proyecto

1. **Clonar el repositorio**
   Colocar el repositorio dentro de la carpeta `src` de tu workspace ROS 2. Por ejemplo:

   ```bash
   mkdir -p ~/AdR/p3_ws/src
   cd ~/AdR/p3_ws/src
   git clone https://github.com/josegg2212/p3_ekf_jgg
   ```

2. **Compilar e inicializar el entorno**

   ```bash
   cd ~/AdR/p3_ws
   colcon build
   source install/setup.zsh
   ```

3. **Lanzar el simulador de TurtleBot3**

   ```bash
   # Si no tienes configuradas las variables de TurtleBot3
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.zshrc
   source ~/.zshrc

   # Instalar dependencias necesarias
   sudo apt update
   sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-turtlebot3-gazebo

   # Lanzar mundo de Gazebo con TurtleBot3
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

4. **Ejecutar los nodos de estimación EKF**
   Existen tres nodos distintos, uno por cada modelo de estado. Su sintaxis es similar:

   * **Modelo 3D** (`[x, y, θ]`):

     ```bash
     ros2 run p3_ekf_adr ekf_estimation_3d
     ```
   * **Modelo 7D** (`[x, y, θ, vx, vy, ω, b_gyro]` u otro orden interno):

     ```bash
     ros2 run p3_ekf_adr ekf_estimation_7d
     ```
   * **Modelo 8D** (`[x, y, θ, vx, vy, ω, bias_acc, bias_gyro]`, por ejemplo):

     ```bash
     ros2 run p3_ekf_adr ekf_estimation_8d
     ```

---

## Descripción del proyecto

Este proyecto implementa un **Filtro de Kalman Extendido (EKF)** en ROS 2 para estimar el estado de un robot móvil (TurtleBot3) usando odometría y datos de IMU simulada. Se proponen tres modelos de estado de complejidad creciente:

1. **Modelo 3D**

   * Estado: $x, y, θ$.
   * Únicamente se usa odometría como observación ruidosa.
   * Implementado en `ekf_estimation_3d.py` dentro de la raíz del paquete.

2. **Modelo 7D**

   * Estado: $x, y, θ, vx, vy, ω, b_imu$ (por ejemplo, incluyéndose las velocidades lineales y angular y un sesgo de IMU).
   * Se fusionan odometría y lecturas de la IMU.
   * Implementado en `ekf_estimation_7d.py`.

3. **Modelo 8D**

   * Estado: $x, y, θ, vx, vy, ω, bias_acc, bias_gyro$.
   * Fusiona odometría + IMU + corrección de sesgos tanto de acelerómetro como de giroscopio.
   * Implementado en `ekf_estimation_8d.py`.

El nodo genérico de EKF (`kf_node.py`) define la lógica base de suscripción y publicación en ROS 2, así como los pasos de predicción y corrección del EKF. Cada script `ekf_estimation_<mod>.py` extiende esa clase base para configurar:

* **Modelos de transición (g() y sus jacobianos)** en `motion_models`.
* **Modelos de observación (h() y sus jacobianos)** en `observation_models`.
* **Parámetros de ruido de proceso (R)** y **ruido de observación (Q)**.

En carpetas auxiliares:

* `filters/ekf.py` contiene la clase `ExtendedKalmanFilter` donde se implementan los métodos:

  * `predict(self, u, dt)`: predicción no lineal con Jacobiano del modelo de movimiento.
  * `update(self, z, dt)`: corrección no lineal con Jacobiano del modelo de observación.
* `motion_models/velocity_motion_models.py` (u otros archivos en esa carpeta) definen `g(μ,u,dt)` y las funciones para calcular jacobianos **G** (respecto al estado) y **V** (respecto al control).
* `observation_models` define las funciones `h(μ)` y Jacobiano **H(μ)** para cada modelo (solo pose, pose+velocidades, etc.).
* `common_utils` incluye utilidades genéricas:

  * Conversión de mensajes ROS a vectores de estado.
  * Funciones para añadir ruido gaussiano a odometría/IMU.

---

1. **Inicialización de cada nodo**

   * Se suscribe a `/odom` e `/imu` (solo IMU en 7D/8D, el 3D solo usa /odom como “medida”).
   * Se define el estado inicial (μ₀) y covarianza (Σ₀).
   * Se configuran los parámetros de ruido (desviaciones estándar) para proceso (**R**) y observación (**Q**).
   * Se crea la instancia de `ExtendedKalmanFilter` con los modelos correspondientes (g, G, V, h, H).

2. **Callback de odometría** (`odom_callback`)

   * Se convierte el mensaje ROS (`nav_msgs/Odometry`) a un vector de pose 2D $x,y,θ$.
   * Para 7D/8D, se extrae la velocidad lineal y angular estimada por odometría.
   * Se añade **ruido** a la observación.
   * Se construye el vector de control u (velocidad lineal y angular).
   * Se invoca `kf.predict(u, dt)` y luego `kf.update(z, dt)`.

3. **Callback de IMU** (`imu_callback`) *(solo 7D y 8D)*

   * Se procesa `sensor_msgs/Imu` para obtener lecturas de aceleración y giroscopio.
   * Se filtran/trasforman según el modelo.
   * Se añade ruido de IMU según configuración.
   * Se usan estas lecturas dentro de la corrección del EKF.

4. **Predict**

   * Llamada interna a `g(μ_{k-1}, u_k, dt)` para obtener μₖ|ₖ₋₁.
   * Jacobiano G = ∂g/∂μ evaluado en μₖ₋₁.
   * Actualización de covarianza: Σₖ|ₖ₋₁ = G\,Σₖ₋₁\,Gᵀ + V\,R\,Vᵀ donde V = ∂g/∂u y R es la covarianza de ruido de proceso.

5. **Update**

   * Se calcula la predicción de medida h(μₖ|ₖ₋₁).
   * Jacobiano H = ∂h/∂μ evaluado en μₖ|ₖ₋₁.
   * Estado corregido: μₖ = μₖ|ₖ₋₁ + K(zₖ − h(μₖ|ₖ₋₁)).
   * Covarianza actualizada: Σₖ = (I − K,H)Σₖ|ₖ₋₁.

6. **Visualización y registro**

   * Cada paso proyecta la trayectoria real (odometría ruidosa) en RViz y la estimación (posición de μₖ).
   * Se guardan las trayectorias en arrays para generar, al finalizar, gráficos que muestran:

     * **Ground truth** (trayectoria limpia de odometría original sin ruido).
     * **Observaciones** (odometría ruidosa/IMU ruidosa).
     * **Estimación EKF** (estimación).

---

## Experimentos y resultados

Para evaluar el comportamiento de los tres modelos (3D, 7D y 8D) se realizaron **nueve** experimentos en total, abarcando tres configuraciones de ruido para cada modelo:

1. **Caso base (ruido por defecto)**

   * Se emplean las desviaciones estándar de ruido inicializadas en el código (valores por defecto).
   * **Observación principal**: Las gráficas resultantes aparecen “desalineadas” y con muchas oscilaciones, pues no hay un tuning de los parámetros de R y Q.
   * Gráficas obtenidas:

     * `caso_base_3D.png`
     * `caso_base_7D.png`
     * `caso_base_8D.png`

2. **Ruido alto en la observación**

   * Se incrementa notablemente la desviación estándar del **ruido de observación** (Q grande) para simular sensores poco fiables.
   * El filtro EKF “cree” menos en la medida y confía más en el modelo de movimiento no lineal.
   * **Observación principal**:

     * La estimación se vuelve muy “suavizada”, pues el EKF minimiza las correcciones vía sensores poco fiables.
     * Sin embargo, dado que nuestros modelos (especialmente el 7D y el 8D) no están tan bien ajustados, la trayectoria predicha por el modelo de movimiento domina y se aleja de la verdad cuando ocurre un giro brusco.
   * Gráficas obtenidas:

     * `alto_obs_3D.png`
     * `alto_obs_7D.png`
     * `alto_obs_8D.png`

3. **Ruido alto en el proceso**

   * Se aumenta la desviación estándar del **ruido de proceso** (R grande), haciendo que el filtro reconozca mayor incertidumbre en el modelo de movimiento.
   * Ahora el EKF “confía” más en los sensores (odometría/IMU).
   * **Observación principal**:

     * La estimación tiende a “pegarse” a la observación (incluso si ésta es ruidosa), generando trayectorias con más fluctuaciones similares a los datos medidos.
     * El modelo 3D, al ser más simple, sigue mejor la medición de odometría con ruido .
     * En los casos 7D/8D, al confiar en IMU + odometría, la trayectoria estimada ronda de cerca a la observada.
   * Gráficas obtenidas:

     * `alto_proc_3D.png`
     * `alto_proc_7D.png`
     * `alto_proc_8D.png`







