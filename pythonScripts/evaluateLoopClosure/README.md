# Valutazione dei Loop Closure tramite Ground Truth

Questo script Python consente di verificare se i loop closure rilevati da un nodo ROS sono corretti, confrontandoli con la ground truth delle pose nel tempo.

## File in ingresso

### loop_closures.txt
Contiene le righe di loop closure rilevati dal nodo SLAM. Ogni riga indica un loop trovato o non trovato, sono dunque presenti i timestamps delle pointcloud prese in considerazione.

### ground_truth.txt
Contiene la ground truth delle pose nel tempo. Ogni riga inizia con un timestamp (in secondi, in notazione scientifica) seguito da valori che rappresentano la pose del robot. 

## Obiettivo dello script

Per ogni coppia di timestamp nel file `loop_closures.txt`, lo script:

- Trova nella ground truth le pose più vicine temporalmente.
- Calcola la distanza euclidea tra le due posizioni.
- Determina se il loop closure è corretto in base a una soglia (threshold, default 4.0 m).

## Metodo di valutazione

Lo script utilizza la distanza euclidea per confrontare le due posizioni nella ground truth:

**Formula della distanza euclidea:**

d = √[(x₁ - x₂)² + (y₁ - y₂)² + (z₁ - z₂)²]


Se `d < threshold` → il loop è considerato corretto.  
Se `d ≥ threshold` → il loop è considerato errato.

Lo script calcola anche le metriche di classificazione:

- True Positive (TP) → loop previsto e confermato dalla ground truth
- True Negative (TN) → assenza di loop prevista e confermata
- False Positive (FP) → loop previsto ma errato
- False Negative (FN) → loop non previsto ma presente in ground trut

Tali metriche vengono utilizzate per calcolare precision e recall.
- Recall: misura la capacità del sistema di trovare tuttii veri loop closure presenti
- Precision: misura la capacità del sistema di riconoscere solo veri loop closure

     $$
     Precision = \frac{TP}{TP + FP}
     $$
     
     
     $$
     Recall = \frac{TP}{TP + FN}
     $$

## Struttura dello script

Lo script è suddiviso in più funzioni:

- `parse_loop_closures(file_path)`: legge il file dei loop closure ed estrae le coppie di timestamp.
- `parse_ground_truth(file_path)`: legge il file della ground truth ed estrae timestamp e posizioni (x, y, z).
- `find_closest(gt_data, target_time)`: trova la pose nella ground truth il cui timestamp è più vicino a un tempo target.
- `evaluate_loops(loops, gt_data, threshold)`: valuta ciascun loop closure, calcola la distanza e determina se è valido.

## Output

Lo script restituisce un file .txt nel quale, per ogni loop trovato o non, scrive un record contenente:
- I due timestamp coinvolti (`t1`, `t2`)
- La distanza calcolata `d`
- Il risultato della valutazione (`TP`/ `TN` / `FP` / `FN`)

Inoltre, alla fine del file si trova:
- Numero di `TP`/ `TN` / `FP` / `FN` trovati
- Valori di precision e recall

Questi ultimi citati vengono stampati anche a terminale




