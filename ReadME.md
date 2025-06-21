# Robotica Line Follower 🤖

Questo progetto universitario rappresenta lo sviluppo di un'algoritmo di computer vision

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

## Note 📝

- I parametri di calibrazione vengono caricati da un file YAML (`config/calibration_params.yaml`).
- Modifica i topic nel codice se la tua configurazione hardware è diversa.

## Autori 👥

- [Giada Pietrocola](https://github.com/GiadaPietrocola)
- [Achille Cannavale](https://github.com/Achille1912)