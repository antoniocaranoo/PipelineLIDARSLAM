import re
import numpy as np
import datetime

def parse_loop_closures(file_path):
    loops = []
    with open(file_path, 'r') as f:
        for line in f:
            
            if "[loop found]" in line.lower():
                predicted_loop = True
            elif "[not loop]" in line.lower():
                predicted_loop = False
            else:
                continue  

            # Estraggo timestamp
            match = re.search(
                r"timestamp:\s*([\d.]+)\s*s\s*and\s*(\d+)\s*\(timestamp:\s*([\d.]+)\s*s\)",
                line
            )
            if match:
                t1 = float(match.group(1))
                t2 = float(match.group(3))
                loops.append((t1, t2, predicted_loop))
    return loops

def parse_ground_truth(file_path):
    gt = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 4:
                timestamp = float(parts[0])
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                gt.append((timestamp, np.array([x, y, z])))
    return gt

def find_closest(gt_data, target_time):
    times = np.array([t for t, _ in gt_data])
    idx = (np.abs(times - target_time)).argmin()
    return gt_data[idx][1]

def evaluate_loops(loops, gt_data, threshold=4.0):
    results = []
    for t1, t2, predicted_loop in loops:
        try:
            pos1 = find_closest(gt_data, t1)
            pos2 = find_closest(gt_data, t2)
            distance = np.linalg.norm(pos1 - pos2)
            ground_truth_loop = distance < threshold
            results.append({
                "t1": t1,
                "t2": t2,
                "distance": distance,
                "predicted": predicted_loop,
                "ground_truth": ground_truth_loop
            })
        except Exception as e:
            print(f"Errore con loop ({t1}, {t2}): {e}")
    return results



loop_file = '/home/otto/ros2_ws/src/log_2025-8-11_8-54-49.txt'
gt_file = '/home/otto/Desktop/NewFolder/gt_00.tum'

loops = parse_loop_closures(loop_file)
gt_data = parse_ground_truth(gt_file)

print(f"Loop trovati: {len(loops)}")  
results = evaluate_loops(loops, gt_data, threshold=4.0)

# File output
now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f"loop_results_{now}.txt"

true_positive_loops = 0
true_negative_loops = 0
false_positive_loops = 0
false_negative_loops = 0

with open(log_filename, 'w') as f:
    for res in results:
        predicted = res["predicted"]
        ground_truth = res["ground_truth"]
        
        loop_text = "[loop found]" if predicted else "[not loop]"

        if predicted and ground_truth:
            status = "TRUE POSITIVE"
            true_positive_loops += 1
        elif not predicted and not ground_truth:
            status = "TRUE NEGATIVE"
            true_negative_loops += 1
        elif predicted and not ground_truth:
            status = "FALSE POSITIVE"
            false_positive_loops += 1
        else:
            status = "FALSE NEGATIVE"
            false_negative_loops += 1
         
        f.write(f"{loop_text} tra {res['t1']:.3f}s e {res['t2']:.3f}s → "
                f"distanza = {res['distance']:.2f} m → {status}\n")


    f.write("\n RISULTATI FINALI \n")
    f.write(f"True Positive Loops: {true_positive_loops}\n")
    f.write(f"True Negative Loops: {true_negative_loops}\n")
    f.write(f"False Positive Loops: {false_positive_loops}\n")
    f.write(f"False Negative Loops: {false_negative_loops}\n")

    precision = true_positive_loops / (true_positive_loops + false_positive_loops) if (true_positive_loops + false_positive_loops) > 0 else 0
    recall = true_positive_loops / (true_positive_loops + false_negative_loops) if (true_positive_loops + false_negative_loops) > 0 else 0

    f.write(f"Precision: {precision:.4f}\n")
    f.write(f"Recall: {recall:.4f}\n")


print(f"TP: {true_positive_loops} | TN: {true_negative_loops} | FP: {false_positive_loops} | FN: {false_negative_loops}")
print(f"Precision: {precision:.4f} | Recall: {recall:.4f}")
print(f"Risultati salvati in: {log_filename}")

