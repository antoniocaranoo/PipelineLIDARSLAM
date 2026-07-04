import numpy as np
from scipy.spatial.transform import Rotation as R

def load_tum_poses(file_path):
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.strip() == '':
                continue
            parts = line.strip().split()
            timestamp = float(parts[0])
            tx, ty, tz = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])
            T = np.eye(4)
            T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
            T[:3, 3] = [tx, ty, tz]
            poses.append((timestamp, T))
    return poses

def save_tum_poses(file_path, poses):
    with open(file_path, 'w') as f:
        for timestamp, T in poses:
            t = T[:3, 3]
            q = R.from_matrix(T[:3, :3]).as_quat()  # qx, qy, qz, qw
            line = f"{timestamp:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
            f.write(line + "\n")

def main():
    input_file = '/home/otto/Desktop/NewFolder/kiss_odometry.tum'
    output_file = '/home/otto/Desktop/NewFolder/kiss_odometry_calib.tum'

    # La matrice Tr da LiDAR a Camera in formato 4x4
    Tr = np.array([
        [4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02],
        [-7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02],
        [9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01],
        [0.0, 0.0, 0.0, 1.0]
           
    ])

    print("Caricamento pose da file .tum...")
    tum_poses = load_tum_poses(input_file)

    print("Applicazione della trasformazione Tr a tutte le pose...")
    transformed_poses = [(ts, Tr @ T) for ts, T in tum_poses]

    print("Salvataggio delle pose trasformate in formato .tum...")
    save_tum_poses(output_file, transformed_poses)

    print(f"Fatto. Nuovo file salvato come: {output_file}")

if __name__ == '__main__':
    main()

