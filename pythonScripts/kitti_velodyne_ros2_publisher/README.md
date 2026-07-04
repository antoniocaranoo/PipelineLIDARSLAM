Publisher ROS 2 per PointCloud2 da dataset KITTI

Scopo:
Questo script ha lo scopo di leggere i dati LiDAR in formato binario da un dataset KITTI e pubblicarli come messaggi PointCloud2 su un topic ROS 2. Viene simulata la trasmissione temporale dei dati, usando i timestamp originali forniti dal dataset.

Funzionamento:
- Lo script è costruito come un nodo ROS 2 (`KittiPublisher`) che pubblica dati LiDAR letti da file `.bin`.
- I file binari si trovano nella cartella `velodyne/` e vengono letti uno alla volta a intervalli regolari (10 Hz).
- Ogni point cloud è associato a un timestamp letto da `times.txt`.
- I dati vengono convertiti nel formato `PointCloud2`, con i campi `x`, `y`, `z` e `t` (timestamp per punto).
- Il nodo pubblica i dati sul topic `/pointcloud_topic` e termina automaticamente alla fine della sequenza.

Requisiti:
- ROS 2 
- Pacchetti Python: `numpy`, `rclpy`, `sensor_msgs`, `std_msgs`, `builtin_interfaces`
- Dataset KITTI con cartelle:
    - `velodyne/` contenente file `.bin`
    - `times.txt` contenente timestamp (uno per riga)
- Percorso modificabile in `dataset_path` nel `main()`.

Utilizzo:
1. Assicurarsi che ROS 2 sia attivo in un ambiente sorgente.
2. Modificare il percorso `dataset_path` nel `main()` se necessario.
3. Eseguire lo script: python3 kitti_publisher.py



Note:
- Lo script arresta automaticamente ROS al termine dei file.
- Se il numero di timestamp non corrisponde ai file, il nodo stampa un errore e si chiude.


