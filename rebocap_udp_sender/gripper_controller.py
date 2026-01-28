#!/usr/bin/env python3
"""
Gripper Controller with GUI for torque control
"""

import tkinter as tk
from tkinter import ttk
import threading

class GripperController:
    """Controller for gripper torque values with GUI interface"""

    def __init__(self, left_torque=0.0, right_torque=0.0, min_torque=0.0, max_torque=10.0):
        self.left_torque = left_torque
        self.right_torque = right_torque
        self.min_torque = min_torque
        self.max_torque = max_torque

        # Thread-safe access
        self.lock = threading.Lock()

        # GUI
        self.root = None
        self.gui_thread = None
        self.running = False

        # Selection state
        self.selected_gripper = 'left'  # 'left' or 'right'

        # GUI elements for highlighting
        self.left_frame = None
        self.right_frame = None

        # Open gripper functionality
        self.open_duration = 0.5  # seconds to hold -1 torque when opening

    def start_gui(self):
        """Start the GUI in a separate thread"""
        if self.gui_thread is None:
            self.running = True
            self.gui_thread = threading.Thread(target=self._run_gui, daemon=True)
            self.gui_thread.start()

    def stop_gui(self):
        """Stop the GUI"""
        self.running = False
        if self.root is not None:
            self.root.quit()

    def _run_gui(self):
        """Run the GUI main loop"""
        self.root = tk.Tk()
        self.root.title("Gripper Torque Controller - Keyboard Control")
        self.root.geometry("800x250")

        # Define custom styles for highlighting
        style = ttk.Style()
        style.configure('Selected.TLabelframe', borderwidth=3, relief='solid',
                       background='#e6f7ff', bordercolor='#1890ff')
        style.configure('Selected.TLabelframe.Label', font=('Arial', 10, 'bold'),
                       foreground='#1890ff')

        # Left gripper frame (top)
        self.left_frame = ttk.LabelFrame(self.root, text="LEFT GRIPPER", padding=10)
        self.left_frame.pack(pady=5, padx=20, fill=tk.X)

        # Left gripper progress bar and value
        left_progress_frame = ttk.Frame(self.left_frame)
        left_progress_frame.pack(fill=tk.X)

        ttk.Label(left_progress_frame, text="Torque:", width=8).pack(side=tk.LEFT)
        self.left_progress = ttk.Progressbar(left_progress_frame, length=500,
                                           maximum=self.max_torque - self.min_torque,
                                           value=self.left_torque - self.min_torque)
        self.left_progress.pack(side=tk.LEFT, padx=10, expand=True, fill=tk.X)

        self.left_value_label = ttk.Label(left_progress_frame, text=f"{self.left_torque:.1f}",
                                        font=('Arial', 12, 'bold'), width=8)
        self.left_value_label.pack(side=tk.RIGHT)

        # Right gripper frame (bottom)
        self.right_frame = ttk.LabelFrame(self.root, text="RIGHT GRIPPER", padding=10)
        self.right_frame.pack(pady=5, padx=20, fill=tk.X)

        # Right gripper progress bar and value
        right_progress_frame = ttk.Frame(self.right_frame)
        right_progress_frame.pack(fill=tk.X)

        ttk.Label(right_progress_frame, text="Torque:", width=8).pack(side=tk.LEFT)
        self.right_progress = ttk.Progressbar(right_progress_frame, length=500,
                                            maximum=self.max_torque - self.min_torque,
                                            value=self.right_torque - self.min_torque)
        self.right_progress.pack(side=tk.LEFT, padx=10, expand=True, fill=tk.X)

        self.right_value_label = ttk.Label(right_progress_frame, text=f"{self.right_torque:.1f}",
                                         font=('Arial', 12, 'bold'), width=8)
        self.right_value_label.pack(side=tk.RIGHT)

        # Bind keyboard events
        self.root.bind('<Key>', self._on_key_press)
        self.root.focus_set()  # Make sure window has focus for key events

        # Update display periodically and highlight selection
        self._update_display()

        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.root.mainloop()

    def _on_key_press(self, event):
        """Handle keyboard events"""
        key = event.keysym

        if key == 'Up':
            # Select left gripper
            self.selected_gripper = 'left'
            self._update_display()
        elif key == 'Down':
            # Select right gripper
            self.selected_gripper = 'right'
            self._update_display()
        elif key == 'Left':
            # Decrease torque of selected gripper
            self._adjust_torque(-0.1)
        elif key == 'Right':
            # Increase torque of selected gripper
            self._adjust_torque(0.1)
        elif key == 'space':
            # Open selected gripper
            self._open_selected_gripper()

    def _adjust_torque(self, delta):
        """Adjust torque of selected gripper by delta"""
        with self.lock:
            if self.selected_gripper == 'left':
                self.left_torque = max(self.min_torque, min(self.max_torque, self.left_torque + delta))
            elif self.selected_gripper == 'right':
                self.right_torque = max(self.min_torque, min(self.max_torque, self.right_torque + delta))
        self._update_display()

    def _open_selected_gripper(self):
        """Open the currently selected gripper"""
        if self.selected_gripper == 'left':
            self._open_left_gripper()
        elif self.selected_gripper == 'right':
            self._open_right_gripper()
        self._update_display()

    def _update_display(self):
        """Update GUI display with current values and highlighting"""
        if self.root is not None and self.running:
            try:
                # Update progress bars and labels (only show non-negative values)
                left_display_value = max(0.0, self.left_torque)
                self.left_progress['value'] = left_display_value - self.min_torque
                self.left_value_label.config(text=f"{left_display_value:.1f}")

                right_display_value = max(0.0, self.right_torque)
                self.right_progress['value'] = right_display_value - self.min_torque
                self.right_value_label.config(text=f"{right_display_value:.1f}")

                # Highlight selected gripper
                if self.selected_gripper == 'left':
                    self.left_frame.config(style='Selected.TLabelframe')
                    self.right_frame.config(style='TLabelframe')
                else:
                    self.left_frame.config(style='TLabelframe')
                    self.right_frame.config(style='Selected.TLabelframe')

                # Schedule next update
                self.root.after(50, self._update_display)  # Update every 50ms
            except tk.TclError:
                pass  # GUI might be closing

    def _on_closing(self):
        """Handle window closing"""
        self.running = False
        self.root.destroy()

    def get_torques(self):
        """Get current torque values (thread-safe)"""
        with self.lock:
            return self.left_torque, self.right_torque

    def set_torques(self, left, right):
        """Set torque values (thread-safe)"""
        with self.lock:
            self.left_torque = max(self.min_torque, min(self.max_torque, left))
            self.right_torque = max(self.min_torque, min(self.max_torque, right))

    def _open_left_gripper(self):
        """Open left gripper by setting torque to -1 for a duration, then reset to 0"""
        with self.lock:
            self.left_torque = -1.0

        # Schedule reset to 0 after duration
        def reset_torque():
            with self.lock:
                self.left_torque = 0.0

        threading.Timer(self.open_duration, reset_torque).start()

    def _open_right_gripper(self):
        """Open right gripper by setting torque to -1 for a duration, then reset to 0"""
        with self.lock:
            self.right_torque = -1.0

        # Schedule reset to 0 after duration
        def reset_torque():
            with self.lock:
                self.right_torque = 0.0

        threading.Timer(self.open_duration, reset_torque).start()
