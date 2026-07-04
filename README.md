# stage-carano


Questo documento descrive le attività svolte durante lo stage, articolate in diverse fasi operative e di analisi. L'obiettivo è stato l'ottenimento, l'elaborazione e la valutazione dell'odometria e delle traiettorie a partire da dati LiDAR, utilizzando strumenti ROS 2 e tecniche SLAM.

## Fasi dello stage

--------------------------------------------------------
### Fase 1 - Odometria e Generazione dei Keyframe
--------------------------------------------------------

Il robot utilizzato è equipaggiato con sensori 3D (LiDAR o RGB-D), i quali acquisiscono rappresentazioni dell’ambiente sotto forma di nuvole di punti (point cloud), ovvero insiemi di coordinate 3D (x, y, z) che descrivono la geometria circostante.

Tra due scansioni successive, è stimata una trasformazione rigida (rotazione + traslazione) che descrive lo spostamento relativo del robot. Per ottimizzare l’elaborazione, vengono selezionati dei keyframe, cioè scansioni significative, tipicamente ogni N metri o in presenza di cambiamenti rilevanti nella scena.

--------------------------------------------------------
### Fase 2 - Place Recognition
--------------------------------------------------------

Il riconoscimento dei luoghi consente al sistema di rilevare quando il robot rientra in una zona già esplorata, permettendo la chiusura del loop (loop closure).

Ogni keyframe viene convertito in un embedding, un vettore o una rappresentazione sintetica della struttura osservata, che viene salvato in un database con la rispettiva posa.

Quando viene acquisito un nuovo keyframe, il sistema confronta il suo embedding con quelli esistenti per individuare somiglianze. Se il match è buono, viene ipotizzata una loop closure.

--------------------------------------------------------
### Fase 3 - Point Cloud Registration
--------------------------------------------------------

Una possibile loop closure viene verificata tramite registrazione di nuvole di punti, ovvero l’allineamento tra due scansioni 3D in un sistema di riferimento comune.

Viene ricercata la trasformazione rigida che meglio allinea le due point cloud (quella nuova e quella corrispondente a un keyframe passato). Se l’errore di allineamento è basso, la loop closure è confermata.




## Implementazione delle fasi dello stage

--------------------------------------------------------
### Fase 1 - Ottenimento traiettoria odometria
--------------------------------------------------------

1. Conversione del Dataset KITTI in ROS 2

   È stato utilizzato uno script Python (`kitti_publisher.py`) che legge i file `velodyne/*.bin` del dataset KITTI e li converte in messaggi `sensor_msgs/msg/PointCloud2`, pubblicandoli su un topic ROS 2.

2. Ottenimento dell'Odometria

   L'odometria è stata stimata utilizzando il pacchetto KISS-ICP con ROS 2:

   ros2 launch kiss_icp odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>

   (modalità di riproduzione a 5HZ, metà del rate)

   terminale 1:
   ros2 bag play <path_to_rosbag> --rate 0.5

   terminale 2:   
   ros2 launch kiss_icp odometry.launch.py topic:=/topic


   I dati di odometria sono stati registrati con:

   ros2 bag record /kiss/odometry


   Il risultato è una cartella ROS 2 bag (formato `rosbag2_YYYY_MM_DD_HH_MM_SS`).

3. Conversione dell'Odometria in Formato .tum

   Utilizzando [evo](https://github.com/MichaelGrupp/evo/wiki/Formats#saving--exporting-to-other-formats), l'odometria è stata convertita in `.tum`, un       formato compatibile con gli strumenti di valutazione.

4. Allineamento del Sistema di Riferimento

   Utilizzando la matrice di calibrazione del dataset KITTI, è stato applicato uno shift spaziale all’odometria tramite `transform_odometry.py`, per           portarla nello stesso sistema di riferimento della ground truth.
   
5. Conversione della Ground Truth

   La ground truth KITTI è stata convertita in `.tum` usando lo script `kitti_to_tum.py`.

6. Valutazione delle Traiettorie

   Le traiettorie (odometria stimata vs ground truth) sono state confrontate singolarmente usando evo:

   comandi evo_traj / evo_ape / evo_rpe

---------------------------------------------------------------
### Fase 2 - Implementazione nodo ros per place recognition
---------------------------------------------------------------

Per la fase di place recognition è stato sviluppato un nodo ros2 in c++, basato sulla libreria `ScanContext` per il rilevamente delle loop closure.

Il nodo si sottoscrive a:
- odometria `/kiss/odometry`
- point clouds `/pointcloud_topic`

Quando il robot si sposta oltre la soglia `min_displacemente`, viene estratto un descrittori della nuova scansione e confrontato con i precedenti per individuare possibili loop closure.


#### Valutazione dei risultati

Per valutare le prestazioni del sistema di place recognition, è stato sviluppato uno script python che:
- legge tutti i log generati dal nodo ros corrispondendi alla rilevazione o meno di un loop closure
- verifica che il loop closure rilavto sia corretto
- classifica di ogni caso come: `TP`, `TN`, `FP`, `FN`
- calcola `precision` e `recall`

--------------------------------------------------------
## Conclusione
--------------------------------------------------------

Il lavoro ha consentito di:

- Simulare e acquisire dati LiDAR da un dataset reale.
- Stimare l’odometria con un sistema SLAM moderno.
- Allineare, convertire e confrontare i dati di odometria con la ground truth.
- Comprendere in dettaglio le fasi di un sistema SLAM: dalla generazione dei keyframe, al riconoscimento dei luoghi, fino alla validazione tramite registrazione di nuvole di punti.

Il codice sviluppato è documentato e include gli script Python per le fasi di conversione, allineamento e valutazione.
