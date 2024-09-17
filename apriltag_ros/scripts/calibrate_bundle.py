#!/usr/bin/env python3
import argparse
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation as R


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate AprilTag bundle relative poses to a "master" tag.'
    )
    parser.add_argument(
        "calibration_file", type=str, help="Relative path of calibration bagfile"
    )
    parser.add_argument("bundle_name", type=str, help="Bundle name")
    parser.add_argument("master_id", type=int, help="Master tag ID")
    args = parser.parse_args()

    calibration_file = args.calibration_file
    bundle_name = args.bundle_name
    master_id = args.master_id

    # Load the tag detections bagfile
    bag = rosbag.Bag(calibration_file)
    tag_msgs = [msg for topic, msg, t in bag.read_messages(topics=["/tag_detections"])]
    bag.close()

    if not tag_msgs:
        print("No messages found on topic '/tag_detections'")
        return

    t0 = tag_msgs[0].header.stamp.to_sec()

    # Initialize tag_data
    tag_data = []
    N = len(tag_msgs)
    for i in range(N):
        msg = tag_msgs[i]
        t = msg.header.stamp.to_sec() - t0
        detections = msg.detections  # detections is a list of AprilTagDetection
        detection_data = {"t": t, "id": [], "size": [], "p": [], "q": []}
        for detection in detections:
            if len(detection.id) > 1:
                # Skip bundle detections
                warning_str = "Skipping tag bundle detection with IDs {}".format(
                    detection.id
                )
                print(warning_str)
                continue
            # Extract data
            detection_data["id"].append(detection.id[0])
            detection_data["size"].append(detection.size[0])

            # Position
            position = detection.pose.pose.pose.position
            p = np.array([position.x, position.y, position.z])
            detection_data["p"].append(p)

            # Orientation (quaternion)
            orientation = detection.pose.pose.pose.orientation
            # Rearrange quaternion from (x, y, z, w) to (w, x, y, z)
            q = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
            detection_data["q"].append(q)
        tag_data.append(detection_data)

    # Proceed to compute measured poses of each tag relative to the master tag
    master_size = None  # Size of the master tag

    # IDs, sizes, relative positions and orientations of detected tags other than master
    other_ids = []
    other_sizes = []
    rel_p = {}  # Dictionary with key=tag_id, value=list of positions
    rel_q = {}  # Dictionary with key=tag_id, value=list of quaternions

    def create_T(p, q):
        # q is (w, x, y, z)
        # Rearrange to (x, y, z, w)
        q_xyzw = np.hstack((q[1:], q[0]))
        R_mat = R.from_quat(q_xyzw).as_matrix()
        T = np.eye(4)
        T[0:3, 0:3] = R_mat
        T[0:3, 3] = p
        return T

    def invert_T(T):
        # Invert a homogeneous transformation matrix
        R_inv = T[0:3, 0:3].T
        p_inv = -R_inv @ T[0:3, 3]
        T_inv = np.eye(4)
        T_inv[0:3, 0:3] = R_inv
        T_inv[0:3, 3] = p_inv
        return T_inv

    for detection_data in tag_data:
        ids = detection_data["id"]
        if master_id not in ids:
            # Master not detected in this detection
            continue

        mi = ids.index(master_id)
        # Get the master tag's rigid body transform to the camera frame
        p_m = detection_data["p"][mi]
        q_m = detection_data["q"][mi]
        T_cm = create_T(p_m, q_m)

        if master_size is None:
            master_size = detection_data["size"][mi]

        # Process other tags detected in the same message
        for j in range(len(ids)):
            if j == mi:
                continue  # Skip the master tag
            tag_id = ids[j]
            if tag_id not in other_ids:
                other_ids.append(tag_id)
                other_sizes.append(detection_data["size"][j])
                rel_p[tag_id] = []
                rel_q[tag_id] = []
            # Get this tag's rigid body transform to the camera frame
            p_j = detection_data["p"][j]
            q_j = detection_data["q"][j]
            T_cj = create_T(p_j, q_j)
            # Deduce this tag's rigid body transform to the master tag's frame
            T_mj = invert_T(T_cm) @ T_cj
            # Save the relative position and orientation
            rel_p[tag_id].append(T_mj[0:3, 3])
            R_mj = T_mj[0:3, 0:3]
            rel_q_rot = R.from_matrix(R_mj)
            rel_q_quat = rel_q_rot.as_quat()  # Returns in (x, y, z, w)
            # Rearrange to (w, x, y, z)
            rel_q_quat = np.hstack((rel_q_quat[3], rel_q_quat[0:3]))
            rel_q[tag_id].append(rel_q_quat)

    if master_size is None:
        print("Master tag with ID {} not found in detections".format(master_id))
        return

    # Compute geometric median position of each tag in master tag frame

    def geometric_median(X, eps=1e-5):
        # Compute the geometric median of a set of points
        # https://stackoverflow.com/questions/30299267/geometric-median-of-multidimensional-points
        y = np.mean(X, axis=0)
        while True:
            D = np.sqrt(((X - y) ** 2).sum(axis=1))
            nonzeros = D != 0
            Dinv = 1 / D[nonzeros]
            W = Dinv / Dinv.sum()
            T = (W[:, np.newaxis] * X[nonzeros]).sum(axis=0)

            num_zeros = len(X) - np.sum(nonzeros)
            if num_zeros == 0:
                y1 = T
            elif num_zeros == len(X):
                return y
            else:
                R = (T - y) * Dinv.sum() / num_zeros
                y1 = y + R

            if np.linalg.norm(y - y1) < eps:
                return y1
            y = y1

    rel_p_median = {}
    for tag_id in other_ids:
        positions = np.array(rel_p[tag_id])
        # Compute geometric median
        p_median = geometric_median(positions)
        rel_p_median[tag_id] = p_median

    # Compute the average orientation of each tag with respect to the master tag
    rel_q_mean = {}
    for tag_id in other_ids:
        quaternions = np.array(rel_q[tag_id])  # Each quaternion is in (w, x, y, z)
        # Stack quaternions into a matrix
        Q = quaternions.T  # Each column is a quaternion
        # Compute the eigenvalues and eigenvectors of Q*Q^T
        eigvals, eigvecs = np.linalg.eig(Q @ Q.T)
        max_index = np.argmax(eigvals)
        avg_quat = eigvecs[:, max_index]
        # Ensure positive scalar component
        if avg_quat[0] < 0:
            avg_quat = -avg_quat
        rel_q_mean[tag_id] = avg_quat

    # Print output to paste in tags.yaml
    print("tag_bundles:")
    print("  [")
    print("    {")
    print("      name: '%s'," % bundle_name)
    print("      layout:")
    print("        [")

    # First, print the master tag
    print(
        "          {id: %d, size: %.2f, x: %.4f, y: %.4f, z: %.4f, qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f},"
        % (master_id, master_size, 0, 0, 0, 1, 0, 0, 0)
    )

    # Now, print other tags
    for i, tag_id in enumerate(other_ids):
        size = other_sizes[i]
        p = rel_p_median[tag_id]
        q = rel_q_mean[tag_id]
        newline = "," if i < len(other_ids) - 1 else ""
        print(
            "          {id: %d, size: %.2f, x: %.4f, y: %.4f, z: %.4f, qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f}%s"
            % (tag_id, size, p[0], p[1], p[2], q[0], q[1], q[2], q[3], newline)
        )

    print("        ]")
    print("    }")
    print("  ]")


if __name__ == "__main__":
    main()
