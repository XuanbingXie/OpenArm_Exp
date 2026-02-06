import csv
import os
import threading
import queue
import time


class Writer:
    """Debug writer for recording joint angles before and after interpolation (non-blocking).

    - `record(timestamp, pre_joints, post_joints)` signature is unchanged and returns immediately.
    - Writes are enqueued and flushed by a background daemon thread.
    - Call `stop()` or use as a context manager to flush before exit.
    """

    def __init__(self, filename='debug_joints.csv', debug=True, _queue_maxsize=0):
        self.debug = debug
        self.filename = filename
        if self.debug:
            # Ensure the file exists and write header (overwrite existing)
            with open(self.filename, 'w', newline='') as f:
                writer = csv.writer(f)
                header = ['timestamp'] + [f'pre_j{i+1}' for i in range(14)] + [f'post_j{i+1}' for i in range(14)]
                writer.writerow(header)

            # Setup async queue and worker
            self._queue = queue.Queue(maxsize=_queue_maxsize) if _queue_maxsize > 0 else queue.Queue()
            self._stop_event = threading.Event()
            self._worker_thread = threading.Thread(target=self._worker, daemon=True)
            self._worker_thread.start()
        else:
            # When debug is False, avoid creating background resources
            self._queue = None
            self._stop_event = None
            self._worker_thread = None

    def record(self, timestamp, pre_joints, post_joints):
        """Enqueue a row for non-blocking write. Signature unchanged."""
        if not self.debug:
            return
        if len(pre_joints) != 14 or len(post_joints) != 14:
            print(f"Warning: Expected 14 joints, got pre: {len(pre_joints)}, post: {len(post_joints)}")
            return
        row = [timestamp] + list(pre_joints) + list(post_joints)
        try:
            # put_nowait to avoid blocking the caller
            self._queue.put_nowait(row)
        except queue.Full:
            # If the queue is bounded and full, drop the row to avoid blocking
            print("Warning: write queue full, dropping debug row")

    def _worker(self):
        """Background worker that consumes the queue and writes to CSV."""
        try:
            with open(self.filename, 'a', newline='') as f:
                writer = csv.writer(f)
                while not (self._stop_event.is_set() and self._queue.empty()):
                    try:
                        row = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    try:
                        writer.writerow(row)
                        f.flush()
                    finally:
                        try:
                            self._queue.task_done()
                        except Exception:
                            pass
        except Exception as e:
            print(f"Writer worker encountered exception: {e}")

    def stop(self, wait=True, timeout=None):
        """Stop the background worker and flush remaining rows. Call before exit if needed."""
        if not self.debug:
            return
        self._stop_event.set()
        if wait and self._worker_thread is not None:
            start = time.time()
            # wait for queue to drain or timeout
            while not self._queue.empty():
                if timeout and (time.time() - start) > timeout:
                    break
                time.sleep(0.05)
            self._worker_thread.join(timeout=timeout)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop(wait=True)

    def __del__(self):
        try:
            if getattr(self, '_stop_event', None):
                self.stop(wait=False)
        except Exception:
            pass