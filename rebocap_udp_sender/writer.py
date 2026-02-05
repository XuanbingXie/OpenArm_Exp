import csv
import os

class Writer:
    """Debug writer for recording joint angles before and after interpolation"""

    def __init__(self, filename='debug_joints.csv', debug=True):
        self.debug = debug
        self.filename = filename
        if self.debug:
            # Ensure the file exists and write header if not
            if not os.path.exists(self.filename):
                with open(self.filename, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Header: timestamp, pre_j1 to pre_j14, post_j1 to post_j14
                    header = ['timestamp'] + [f'pre_j{i+1}' for i in range(14)] + [f'post_j{i+1}' for i in range(14)]
                    writer.writerow(header)

    def record(self, timestamp, pre_joints, post_joints):
        """Record joint angles before and after interpolation"""
        if not self.debug:
            return
        if len(pre_joints) != 14 or len(post_joints) != 14:
            print(f"Warning: Expected 14 joints, got pre: {len(pre_joints)}, post: {len(post_joints)}")
            return
        row = [timestamp] + pre_joints + post_joints
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)