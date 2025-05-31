# 🤖 Navigation Controllers

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-green?style=flat-square&logo=python)](https://www.python.org/)

Un paquete completo de ROS2 que implementa múltiples algoritmos de control de navegación para robots móviles, con soporte tanto para simulación (Turtlesim) como robots físicos (Puzzlebot).

---

## ✨ Características

- 🎮 **4 Algoritmos de Control Diferentes**: Turn & Go, Turn While Go, Fuzzy Logic, Pure Pursuit
- 🔄 **Sistema Modular**: Intercambia controladores sin modificar el resto del sistema
- 🐢 **Dual Mode**: Soporte para simulación (Turtlesim) y robot físico (Puzzlebot)
- 📊 **Monitoreo Avanzado**: Comparación automática de precisión (REAL vs ODOM vs TARGET)
- 🎯 **Generador de Trayectorias**: Patrones geométricos predefinidos

---

## 🏗️ Arquitectura del Sistema

```
┌─────────────────┐
│   🐢 Turtlesim  │ ←→ /turtle1/pose
│   Simulator     │ ←→ /turtle1/cmd_vel
└─────────────────┘
         ↕
┌─────────────────┐
│ 📍 Odometry     │ → /odom
│    Node         │
└─────────────────┘
         ↓
┌─────────────────┐
│ 🗺️ Path         │ → next_point
│   Generator     │ ← point_reached
└─────────────────┘
         ↓
┌─────────────────┐
│ 🎮 Controllers  │ → /turtle1/cmd_vel
│ (Turn&Go/Fuzzy/ │ ← /odom
│ TurnWhileGo/PP) │ ← next_point
└─────────────────┘
```

---

## 🛠 Instalación

### Prerrequisitos

- **ROS2 Humble** 
- **Python 3.8+**
- **Turtlesim package**
- **Bibliotecas adicionales** para el controlador Fuzzy:

```bash
pip install scikit-fuzzy numpy
```

### Instalación del Paquete

**1. Clona el repositorio:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/Pezcadoo31/navigation_controllers.git
```

**2. Compila el workspace:**
```bash
cd ~/ros2_ws
colcon build --packages-select navigation_controllers
source install/setup.bash
```

---

## 🚀 Uso

### 🐢 Configuración Básica (Simulación)

**1. Lanzar Turtlesim:**
```bash
ros2 run turtlesim turtlesim_node
```

**2. Iniciar el nodo de odometría:**
```bash
ros2 run navigation_controllers odometry --ros-args -p robot_mode:=turtlesim
```

**3. Lanzar el generador de trayectoria:**
```bash
ros2 run navigation_controllers trayectory
```

**4. Ejecutar un controlador (elige uno):**
```bash
# Turn and Go Controller
ros2 run navigation_controllers turn_and_go

# Turn While Go Controller  
ros2 run navigation_controllers turn_while_go

# Fuzzy Logic Controller
ros2 run navigation_controllers fuzzy_controller

# Pure Pursuit Controller
ros2 run navigation_controllers pure_pursuit_controller
```

### 🤖 Configuración para Robot Físico (Puzzlebot)

```bash
# Nodo de odometría para robot real
ros2 run navigation_controllers odometry --ros-args -p robot_mode:=puzzlebot

# Nodo generador de trayectorias
ros2 run navigation_controllers trayectory

# Controlador con coordenadas invertidas (si es necesario)
ros2 run navigation_controllers fuzzy_controller --ros-args -p invert_coordinates:=true
```

---

## 🎮 Controladores Disponibles

### 1. 🔄 Turn and Go Controller
**Estrategia:** Girar primero, luego avanzar

| ✅ **Pros** | ❌ **Contras** |
|-------------|----------------|
| Simple y predecible | Movimiento discontinuo |
| Alta precisión | Mayor tiempo de ejecución |

### 2. ⚡ Turn While Go Controller  
**Estrategia:** Movimiento y rotación simultáneos

| ✅ **Pros** | ❌ **Contras** |
|-------------|----------------|
| Movimiento fluido | Requiere ajuste fino |
| Eficiente en tiempo | Mayor complejidad |

### 3. 🧠 Fuzzy Logic Controller
**Estrategia:** Control inteligente basado en lógica difusa

| ✅ **Pros** | ❌ **Contras** |
|-------------|----------------|
| Robusto ante incertidumbre | Más complejo de implementar |
| Comportamiento natural | Requiere conocimiento de fuzzy logic |

**Configuración implementada:**
- **15 reglas de inferencia**
- **Variables de entrada:** error_posición, error_orientación
- **Variables de salida:** velocidad_lineal, velocidad_angular

### 4. 🎯 Pure Pursuit Controller
**Estrategia:** Seguimiento de trayectoria con punto de anticipación

| ✅ **Pros** | ❌ **Contras** |
|-------------|----------------|
| Trayectorias suaves | Puede desviarse en curvas cerradas |
| Ampliamente utilizado | Requiere ajuste de lookahead |

---

## ⚙️ Configuración

### Parámetros Principales

| Parámetro | Nodo | Valores | Descripción |
|-----------|------|---------|-------------|
| `robot_mode` | Odometry | `turtlesim` \| `puzzlebot` | Modo de operación |
| `invert_coordinates` | Controllers | `true` \| `false` | Intercambia X e Y |
| `k_linear` | Controllers | `0.1 - 0.3` | Ganancia velocidad lineal |
| `k_angular` | Controllers | `0.2 - 0.5` | Ganancia velocidad angular |
| `distance_threshold` | Controllers | `0.01 - 0.1` | Umbral de llegada |

### 🗺️ Configuración de Trayectorias

Edita el archivo `path_generator_node.py` para personalizar los puntos:

```python
self.point_list = [
    # Cuadrado
    [1.0, 0.0],
    [1.0, 1.0], 
    [0.0, 1.0],
    [0.0, 0.0],
    # Agrega tus propios puntos aquí
]
```

**Figuras Geométricas Predefinidas:**

| Figura | Descripción |
|--------|-------------|
| 🔷 **Cuadrado** | Figura básica de 4 lados |
| 🔶 **Rombo** | Paralelogramo con lados iguales |
| 🔺 **Triángulo** | Polígono de 3 vértices |
| 🔸 **Trapecio** | Cuadrilátero con lados paralelos |
| 💎 **Diamante** | Forma de diamante compleja |

---

## 🔍 Monitoreo y Debugging

### Visualización de Topics

```bash
# Ver todos los topics activos
ros2 topic list

# Monitorear odometría
ros2 topic echo /odom

# Ver comandos de velocidad
ros2 topic echo /turtle1/cmd_vel

# Verificar puntos objetivo
ros2 topic echo next_point
```

### Herramientas de Análisis

```bash
# Frecuencia de publicación
ros2 topic hz /odom

# Gráfico de nodos y topics
rqt_graph
```

### 📊 Métricas Automáticas

El sistema registra automáticamente:

| Métrica | Descripción |
|---------|-------------|
| 📊 **Comparación de precisión** | REAL vs ODOM vs TARGET |
| ⏱️ **Tiempos de llegada** | Para cada punto objetivo |
| 📍 **Errores de posición** | Diferencias en X, Y, θ |
| 🎯 **Puntos de seguimiento** | Inicio, medio, fin de trayectoria |

---

## 📊 Comparación de Rendimiento

| Controlador | Complejidad | Suavidad | Eficiencia | Robustez | Tiempo Promedio |
|-------------|:-----------:|:--------:|:----------:|:--------:|:---------------:|
| **Turn & Go** | ⭐⭐ | ⭐ | ⭐⭐ | ⭐⭐⭐ | `8.5s` |
| **Turn While Go** | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | `6.2s` |
| **Fuzzy Logic** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | `7.1s` |
| **Pure Pursuit** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | `6.8s` |

---

## 🛠 Troubleshooting

### Problemas Comunes

<details>
<summary><strong>🚫 El robot no se mueve</strong></summary>

```bash
# Verificar que los topics estén publicando
ros2 topic hz /turtle1/cmd_vel
ros2 topic echo next_point
```
</details>

<details>
<summary><strong>⚠️ Errores de odometría</strong></summary>

```bash
# Verificar modo de robot
ros2 param get /odometry robot_mode
```
</details>

<details>
<summary><strong>🧠 Controlador Fuzzy no funciona</strong></summary>

```bash
# Instalar dependencias
pip install scikit-fuzzy numpy
```
</details>

<details>
<summary><strong>🔄 Coordenadas invertidas</strong></summary>

```bash
# Usar parámetro de inversión
ros2 run navigation_controllers fuzzy_controller --ros-args -p invert_coordinates:=true
```
</details>

---

## 📞 Contacto

**👨‍🎓 Autor:** [Abdiel Vicencio Antonio A01750922](https://github.com/Pezcadoo31)  
**📚 Materia:** Implementación de robótica inteligente  
**🏫 Institución:** Tecnológico de Monterrey CEM

---


