# Robotica Line Follower 🤖

Questo progetto universitario rappresenta lo sviluppo di un algoritmo di computer vision e di controllo per un turtlebot, che aveva il compito di rimanere in carreggiata e fermarsi ai semafori. 
<p align="center">
    <img src="material/out.gif" alt="Demo GIF" width=10%/>
</p>

## Struttura del progetto 🗂️

```
.
├── launch/
│   └── line_follower.launch
├── src/
│   ├── control.cpp
│   ├── extrinsic_calibration_gui.cpp
│   ├── hsv_tuner.cpp
│   ├── odometry.cpp
│   ├── traffic_light_detector.cpp
│   └── vision_control.cpp
└── ReadME.md
```

## Componenti principali 🛠️

- **odometry.cpp**: Stima la posizione del robot usando i dati degli encoder delle ruote.
- **control.cpp**: Implementa il controllo del robot sulla base della posizione desiderata e dell'odometria.
- **vision_control.cpp**: Elabora le immagini della telecamera per seguire la linea, rilevare semafori e cartelli di stop.
- **traffic_light_detector.cpp**: Rileva il colore dei semafori e pubblica un fattore di velocità.
- **hsv_tuner.cpp**: GUI per la calibrazione dei parametri HSV per la segmentazione dei colori.
- **extrinsic_calibration_gui.cpp**: GUI per la calibrazione della trasformazione prospettica della telecamera.

## Come eseguire 🚀

1. **Compilazione**
   
   Assicurati di avere un workspace ROS configurato. Copia i file nella cartella `src` del tuo workspace e compila con:

   ```sh
   catkin_make
   ```

2. **Lancio dei nodi**

   Avvia tutti i nodi necessari con il file di launch:

   ```sh
   roslaunch line_follower line_follower.launch
   ```

3. **Calibrazione**

   - Usa `hsv_tuner.cpp` per calibrare i parametri HSV.
   - Usa `extrinsic_calibration_gui.cpp` per calibrare la prospettiva della telecamera.

## Dipendenze 📦

- ROS (Robot Operating System)
- OpenCV
- cv_bridge
- image_transport
- geometry_msgs, sensor_msgs, std_msgs

## Equazioni
L'odometria differenziale è stata calcolata con le formule di eulero in avanti:

$$
\begin{align*}
v &= \frac{(\Delta \theta_R + \Delta \theta_L)r}{2T} \\
\omega &= \frac{(\Delta \theta_R - \Delta \theta_L)b}{2T} \\
x_{k+1} &= x_k + v \cdot \cos(\theta_k ) \\
y_{k+1} &= y_k + \omega \cdot \sin(\theta_k ) \\
\theta_{k+1} &= \theta_k + \omega T
\end{align*}
$$

dove:
- $\Delta \theta_R$, $\Delta \theta_L$: spostamenti delle ruote destra e sinistra
- $b$: distanza tra le ruote
- $(x_k, y_k, \theta_k)$: posizione e orientamento attuali del robot
- $(x_{k+1}, y_{k+1}, \theta_{k+1})$: nuova posizione e orientamento

Mentre per il controllo è stato scelto il seguente controllo:
$$
\begin{align*}
k_1 = k_2 = 2 \zeta \alpha \\
k_3 = \frac{\alpha^2 - \omega_d^2}{v_d}\\
u_1 = -k1 e_x\\
u_2 = -k2 e_y - k_3 e_\theta\\
v = v_d cos(e_\theta) - u_1\\
\omega = \omega_d - u_2
\end{align*}
$$

dove:
- $v_{d}$, $\omega_{d}$: velocità lineare e angolare di riferimento
- $e_y, e_x$: errore laterale rispetto alla traiettoria
- $e_\theta$: errore di orientamento
- $k_1$, $k_2, k_3$: guadagni del controllore
- $\zeta, \alpha$: parametri dinamici

## Note 📝

- I parametri di calibrazione vengono caricati da un file YAML (`config/calibration_params.yaml`).
- Modifica i topic nel codice se la tua configurazione hardware è diversa.

## Autori 👥

- [Giada Pietrocola](https://github.com/GiadaPietrocola)
- [Achille Cannavale](https://github.com/Achille1912)



