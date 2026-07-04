Descrizione
-----------
Questo script Python converte una traiettoria registrata nel formato KITTI (pose file + timestamp file) 
in una traiettoria nel formato TUM. È utile per utilizzare traiettorie KITTI con strumenti che supportano 
il formato TUM, come evo, una libreria per la valutazione di traiettorie in ambito SLAM/odometria.

Nel caso dello stage da me svolto, tale script viene utilizzato per convertire la groud trurh da formato KITTI a formato TUM.

Requisiti
---------
- Python 3
- evo installato (pip install evo)

Input
-----
Lo script richiede tre file come input:

1. poses_file: file di pose nel formato KITTI (es. poses.txt)
   - Ogni riga rappresenta una trasformazione 3D 4x4 in formato compatto (12 valori per riga).
2. timestamp_file: file dei timestamp associati alle pose (es. timestamps.txt)
   - Deve contenere una sola colonna con lo stesso numero di righe del file delle pose.
3. trajectory_out: percorso di destinazione del file risultante in formato TUM.

Uso
---
python3 script.py poses.txt timestamps.txt output_tum.txt

Dove:
- poses.txt è il file delle pose in formato KITTI.
- timestamps.txt è il file dei timestamp.
- output_tum.txt è il file di output in formato TUM.

Formato di output (TUM)
-----------------------
Il file generato (output_tum.txt) sarà nel formato TUM, ovvero:

timestamp tx ty tz qx qy qz qw

Dove:
- tx, ty, tz sono le traslazioni.
- qx, qy, qz, qw sono i quaternioni che rappresentano l'orientamento.



