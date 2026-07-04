Trasformazione di pose TUM con matrice di calibrazione

Scopo:
Questo script carica una sequenza di pose 3D in formato TUM (timestamp + posizione + quaternione), applica una trasformazione rigida 4x4 (matrice di calibrazione tra sensori, ad esempio da LiDAR a camera) e salva le nuove pose trasformate nello stesso formato TUM. Questo per far si che ground truth e la traiettoria ottenuta attraverso l'odometria siano nello stesso sistmea di riferiemto.

Funzionamento:
- Le pose vengono caricate da un file `.tum` specificato nel path `input_file`.
- Ogni riga del file contiene: `timestamp tx ty tz qx qy qz qw`.
- Le pose vengono convertite in matrici omogenee 4x4.
- Viene applicata la matrice di calibrazione `Tr` (trasformazione da un frame all'altro).
- Le pose trasformate vengono convertite nuovamente nel formato TUM e salvate su `output_file`.

Requisiti:
- Python 3.x
- Pacchetti: `numpy`, `scipy` (in particolare `scipy.spatial.transform`)

Utilizzo:
1. Posizionare il file `.tum` in input nel percorso corretto (modificabile nel codice).
2. Impostare correttamente la matrice `Tr` (es. da LiDAR a camera).
3. Eseguire lo script: python3 ytansform_odometry.py


Output:
- File `.tum` con le pose trasformate, salvato come specificato in `output_file`.


