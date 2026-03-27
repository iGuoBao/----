import tkinter as tk
from tkinter import messagebox, filedialog

class PathGenerator:
    def __init__(self, root):
        self.root = root
        self.root.title("10x8 Grid Path Generator")
        self.rows = 10
        self.cols = 8
        self.cell_size = 50
        self.waypoints = []  # List of (row, col)
        self.current_direction = 0  # 0: up, 1: right, 2: down, 3: left

        # Canvas for grid
        self.canvas = tk.Canvas(root, width=self.cols * self.cell_size, height=self.rows * self.cell_size, bg='black')
        self.canvas.pack()
        self.draw_grid()

        # Bind click events
        self.canvas.bind("<Button-1>", self.on_left_click)  # Add waypoint
        self.canvas.bind("<Button-3>", self.on_right_click)  # Remove last waypoint

        # UI elements
        self.frame = tk.Frame(root)
        self.frame.pack()

        self.generate_btn = tk.Button(self.frame, text="Generate Path", command=self.generate_path)
        self.generate_btn.pack(side=tk.LEFT)

        self.clear_btn = tk.Button(self.frame, text="Clear", command=self.clear_waypoints)
        self.clear_btn.pack(side=tk.LEFT)

        self.save_btn = tk.Button(self.frame, text="Save Path", command=self.save_path)
        self.save_btn.pack(side=tk.LEFT)

        self.text_area = tk.Text(root, height=10, width=80)
        self.text_area.pack()

    def draw_grid(self):
        for i in range(self.rows + 1):
            self.canvas.create_line(0, i * self.cell_size, self.cols * self.cell_size, i * self.cell_size, fill='white')
        for j in range(self.cols + 1):
            self.canvas.create_line(j * self.cell_size, 0, j * self.cell_size, self.rows * self.cell_size, fill='white')

        # Draw waypoints
        for wp in self.waypoints:
            x = wp[1] * self.cell_size + self.cell_size // 2
            y = (self.rows - 1 - wp[0]) * self.cell_size + self.cell_size // 2  # Origin at bottom-left
            self.canvas.create_oval(x-10, y-10, x+10, y+10, fill='red')

    def on_left_click(self, event):
        col = event.x // self.cell_size
        row = self.rows - 1 - (event.y // self.cell_size)  # Flip y for bottom-left origin
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.waypoints.append((row, col))
            self.draw_grid()

    def on_right_click(self, event):
        if self.waypoints:
            self.waypoints.pop()
            self.draw_grid()

    def clear_waypoints(self):
        self.waypoints = []
        self.draw_grid()
        self.text_area.delete(1.0, tk.END)

    def generate_path(self):
        if not self.waypoints:
            messagebox.showwarning("Warning", "No waypoints defined.")
            return

        path = []
        prev_row, prev_col = self.waypoints[0]
        direction = 0  # Start facing up

        for wp in self.waypoints[1:]:
            row, col = wp
            dr = row - prev_row
            dc = col - prev_col

            # Determine turns
            if dr > 0:  # Up
                target_dir = 0
            elif dr < 0:  # Down
                target_dir = 2
            elif dc > 0:  # Right
                target_dir = 1
            elif dc < 0:  # Left
                target_dir = 3
            else:
                continue  # Same cell

            # Calculate turns
            turn_diff = (target_dir - direction) % 4
            if turn_diff == 1:
                path.append('R')
            elif turn_diff == 3:
                path.append('L')
            elif turn_diff == 2:
                path.append('R')
                path.append('R')

            # Move forward
            distance = abs(dr) + abs(dc)
            if distance == 1:
                path.append('1')
            elif distance == 2:
                path.append('2')
            else:
                # For larger, repeat '1'
                for _ in range(distance):
                    path.append('1')

            direction = target_dir
            prev_row, prev_col = row, col

        # Add stop
        path.append('S')
        path.append('\0')

        # Format as C array
        path_str = "static char path[100] = {\n    " + ", ".join(f"'{c}'" for c in path) + "\n};"
        self.text_area.delete(1.0, tk.END)
        self.text_area.insert(tk.END, path_str)

    def save_path(self):
        content = self.text_area.get(1.0, tk.END).strip()
        if not content:
            messagebox.showwarning("Warning", "No path to save.")
            return
        file_path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if file_path:
            with open(file_path, 'w') as f:
                f.write(content)
            messagebox.showinfo("Saved", f"Path saved to {file_path}")

if __name__ == "__main__":
    root = tk.Tk()
    app = PathGenerator(root)
    root.mainloop()