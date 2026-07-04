import re
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import copy
import os


def load_groundtruth(gt_file):
    poses = []
    timestamps = []
    with open(gt_file, 'r') as f:
        for line in f:
            if not line.strip():
                continue
            vals = list(map(float, line.strip().split()))

            if len(vals) == 8:
                ts, x, y, z, qx, qy, qz, qw = vals
            elif len(vals) == 7:
                x, y, z, qw, qx, qy, qz = vals
                ts = None
            else:
                raise ValueError(f"Riga GT con formato non valido: {line}")

            rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rot
            T[:3, 3] = [x, y, z]
            poses.append(T)
            timestamps.append(ts)
    return np.array(timestamps), poses


def load_loops(loop_file):
    loops = []
    with open(loop_file, 'r') as f:
        lines = [l.strip() for l in f if l.strip()]

    i = 0
    while i < len(lines):
        header = lines[i]
        if not header.startswith("SRC_TIME="):
            raise ValueError(f"Formato log non valido alla riga {i}: {header}")

        parts = header.split()
        src_time = float(parts[0].split("=")[1])
        trg_time = float(parts[1].split("=")[1])

        T_rows = []
        for j in range(1, 5):
            vals = list(map(float, lines[i + j].strip().split()))
            if len(vals) != 4:
                raise ValueError(f"Riga matrice T con formato non valido: {lines[i + j]}")
            T_rows.append(vals)
        T_est = np.array(T_rows)

        loops.append((src_time, trg_time, T_est))
        i += 5
    return loops


def load_labels(label_file):
    """
    Carica le etichette dal file e restituisce una lista di coppie (t1, t2)
    corrispondenti alle righe marcate come TRUE POSITIVE.
    Usa regex per estrarre i tempi e non dipende dall'ordine della frase.
    """
    tp_pairs = []

    with open(label_file, 'r') as f:
        for line in f:
            if not line.strip():
                continue
            # consideriamo solo le righe etichettate come TRUE POSITIVE (case-insensitive)
            if "TRUE POSITIVE" not in line.upper():
                continue

            times = re.findall(r'(\d+\.\d+)s', line)
            if len(times) >= 2:
                t1 = float(times[0])
                t2 = float(times[1])
                tp_pairs.append((t1, t2))
            else:
                print(f"[WARN] Impossibile parsare i tempi nella linea: {line.strip()}")

    return tp_pairs


def is_tp_match(src_time, trg_time, tp_pairs, tol=0.02):
    """
    Verifica se la coppia (src_time, trg_time) corrisponde ad una coppia TP nel file di etichette.
    Il matching è non ordinato e usa una tolleranza (in secondi).
    """
    for a, b in tp_pairs:
        if (abs(a - src_time) <= tol and abs(b - trg_time) <= tol) or \
           (abs(a - trg_time) <= tol and abs(b - src_time) <= tol):
            return True
    return False


def find_closest_pose(time, gt_timestamps):
    if gt_timestamps[0] is None:
        raise ValueError("GT senza timestamp, impossibile fare il matching.")
    diffs = np.abs(gt_timestamps - time)
    return np.argmin(diffs)


def load_kitti_bin_as_pcd(bin_path):
    """Carica un file .bin KITTI come PointCloud Open3D"""
    pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)[:, :3]  
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


# Matrice di calibrazione velo->cam
Tr_velo_to_cam = np.array([
    [4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02],
    [-7.210626507497e-03,  8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02],
    [9.999738645903e-01,  4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01],
    [0.0,                 0.0,                 0.0,                 1.0]
])
Tr_cam_to_velo = np.linalg.inv(Tr_velo_to_cam)


def verify_loop_with_gt(gt_timestamps, gt_poses, loops, kitti_velo_dir, tp_pairs, tol=0.02):
    """
    Verifica i loop stimati confrontandoli con la ground truth, ma processa
    solamente i loop che risultano essere TRUE POSITIVE secondo il file di etichette.
    tol = tolleranza per matching temporale (secondi).
    """
    log_filename = "pcRegistration.txt"
    count = 0

    good_loops = 0
    total_loops = 0
    trans_errors = []
    rot_errors = []

    matched_labels_found = 0
    skipped_loops = 0

    with open(log_filename, 'w') as f:
        f.write(f"Matching dei loop: tolleranza temporale = {tol} s\n")
        f.write(f"TP labels caricate: {len(tp_pairs)}\n\n")

        for src_time, trg_time, T_est in loops:
            # Verifica se la coppia corrente è segnala come TP  
            if not is_tp_match(src_time, trg_time, tp_pairs, tol=tol):
                skipped_loops += 1
                continue

            matched_labels_found += 1

            src_idx = find_closest_pose(src_time, gt_timestamps)
            trg_idx = find_closest_pose(trg_time, gt_timestamps)

            T_src = gt_poses[src_idx]
            T_trg = gt_poses[trg_idx]

            # Calcola la trasformazione relativa ground truth (camera frame)
            T_gt_rel_cam = np.linalg.inv(T_trg) @ T_src

            # Porta la GT nel frame LiDAR
            T_gt_rel_velo = Tr_cam_to_velo @ T_gt_rel_cam @ Tr_velo_to_cam

            # Stima errore
            T_err = np.linalg.inv(T_est) @ T_gt_rel_velo
            rot_err_mat = T_err[:3, :3]
            trans_err_vec = T_err[:3, 3]

            trans_norm = np.linalg.norm(trans_err_vec)
            angle_err = np.rad2deg(
                np.arccos(np.clip((np.trace(rot_err_mat) - 1) / 2, -1.0, 1.0))
            )

            trans_errors.append(trans_norm)
            rot_errors.append(angle_err)

            total_loops += 1
            is_good = (trans_norm <= 1.0) and (angle_err <= 10.0)
            if is_good:
                good_loops += 1

            f.write("=" * 80 + "\n")
            f.write(f"Loop closure check: SRC_TIME={src_time:.6f} → TRG_TIME={trg_time:.6f}\n")
            f.write(f"Matched GT indices: SRC={src_idx} (time={gt_timestamps[src_idx]:.6f}) → TRG={trg_idx} (time={gt_timestamps[trg_idx]:.6f})\n")
            f.write(f"Estimated relative T (KISS, LiDAR frame): \n{T_est}\n")
            f.write(f"Ground truth relative T (LiDAR frame): \n{T_gt_rel_velo}\n")
            f.write(f"Translation error (m): {trans_norm:.6f}\n")
            f.write(f"Rotation error (deg): {angle_err:.6f}\n")
            f.write(f"Good registration: {is_good}\n")

            if not is_good:
                src_file = os.path.join(kitti_velo_dir, f"{src_idx:06d}.bin")
                trg_file = os.path.join(kitti_velo_dir, f"{trg_idx:06d}.bin")

                pcd_src = load_kitti_bin_as_pcd(src_file)
                pcd_trg = load_kitti_bin_as_pcd(trg_file)

                pcd_src_kiss = copy.deepcopy(pcd_src)
                pcd_src_gt = copy.deepcopy(pcd_src)

                pcd_src_kiss.transform(T_est)
                pcd_src_gt.transform(T_gt_rel_velo)

                pcd_trg.paint_uniform_color([0, 1, 0])   # verde = target
                pcd_src_kiss.paint_uniform_color([1, 0, 0])  # rosso = KISS
                pcd_src_gt.paint_uniform_color([0, 0, 1])    # blu = GT

                print(f"Estimated relative T (KISS, LiDAR frame): \n{T_est}\n")
                print(f"Ground truth relative T (LiDAR frame): \n{T_gt_rel_velo}\n")
                print(f"Translation error (m): {trans_norm:.6f}\n")
                print(f"Rotation error (deg): {angle_err:.6f}\n")
                print(f"SRC idx={src_idx} time={gt_timestamps[src_idx]:.6f} → TRG idx={trg_idx} time={gt_timestamps[trg_idx]:.6f}\n")
                print(f"[Visualizzazione loop] Premere 'q' per chiudere la finestra")

                o3d.visualization.draw_geometries([pcd_trg, pcd_src_kiss, pcd_src_gt])
                count += 1

        # Risultati finali
        percent_good = 100.0 * good_loops / total_loops if total_loops > 0 else 0.0
        mean_trans = float(np.mean(trans_errors)) if trans_errors else 0.0
        mean_rot = float(np.mean(rot_errors)) if rot_errors else 0.0

        summary = (
            f"\nTP labels caricate: {len(tp_pairs)}\n"
            f"TP effettivamente trovate nei loop (matching): {matched_labels_found}\n"
            f"Totale loop verificati (solo TP matched): {total_loops}\n"
            f"Loop buoni: {good_loops}\n"
            f"Percentuale buoni: {percent_good:.2f}%\n"
            f"Errore medio traslazione: {mean_trans:.6f} m\n"
            f"Errore medio rotazione: {mean_rot:.6f} deg\n"
            f"Loop saltati (non TP): {skipped_loops}\n"
        )
        f.write(summary)
        print(summary)


if __name__ == "__main__":
    gt_file = "/home/otto/Desktop/NewFolder/gt_00.tum"
    loop_file = "/home/otto/ros2_ws/loop_closures.txt"
    label_file = "/home/otto/Desktop/pythonScripts/evaluateLoopClosure/loop_results_2025-09-03_09-14-57.txt"
    kitti_velo_dir = "/home/otto/Desktop/kitti_data/sequences/00/velodyne"

    gt_timestamps, gt_poses = load_groundtruth(gt_file)
    loops = load_loops(loop_file)
    tp_pairs = load_labels(label_file)
    
    verify_loop_with_gt(gt_timestamps, gt_poses, loops, kitti_velo_dir, tp_pairs, tol=0.02)
