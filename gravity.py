import tkinter as tk
import math
import random
import numpy as np


class DraggableBall:
    def __init__(self, canvas, x, y, r, color):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.r = r
        self.color = color
        self.mass = 4/3 * math.pi * r ** 3
        self.item = canvas.create_oval(x - r, y - r, x + r, y + r,
                                       fill=color, outline="")

    def move_to(self, newx, newy):
        dx = newx - self.x
        dy = newy - self.y
        self.canvas.move(self.item, dx, dy)
        self.x = newx
        self.y = newy


def on_button_press(event):
    clicked = canvas.find_closest(event.x, event.y)
    for ball in main_balls:
        if ball.item == clicked[0]:
            drag_data["ball"] = ball
            drag_data["offset_x"] = ball.x - event.x
            drag_data["offset_y"] = ball.y - event.y
            break


def on_mouse_drag(event):
    if drag_data["ball"] is not None:
        newx = event.x + drag_data["offset_x"]
        newy = event.y + drag_data["offset_y"]
        drag_data["ball"].move_to(newx, newy)


def on_button_release(event):
    drag_data["ball"] = None


def update_simulation():
    dt = 0.05  # time step in seconds
    sim_speed = 400
    G = 3      # gravitational constant
    max_speed = 100
    max_speed_color = max_speed

    # Initialize acceleration array for all orbiting balls.
    acc = np.zeros_like(positions)

    # For each main ball, add its gravitational effect.
    for main in main_balls:
        # Compute differences from orbit balls to the main ball.
        diff = np.empty_like(positions)
        diff[:, 0] = main.x - positions[:, 0]
        diff[:, 1] = main.y - positions[:, 1]
        # Squared distances.
        dist_sq = diff[:, 0]**2 + diff[:, 1]**2
        # Compute distances; add a small epsilon to avoid division-by-zero.
        dist = np.sqrt(dist_sq) + 1e-8
        # Ensure that the distance is never below the sum of radii (to avoid huge forces)
        min_dist = main.r + radii
        dist = np.maximum(dist, min_dist)
        # Compute the factor (G * mass / distance^3) for each orbit ball.
        factor = G * main.mass / (dist**3)
        # Update acceleration contributions.
        acc[:, 0] += factor * diff[:, 0]
        acc[:, 1] += factor * diff[:, 1]

    # Update velocities and positions.
    velocities[:] += acc * dt

    velocities[velocities > max_speed] = max_speed
    velocities[velocities < -max_speed] = -max_speed

    positions[:] += velocities * dt

    # Collision detection with each main ball.
    for main in main_balls:
        diff = positions - np.array([main.x, main.y])
        dist = np.sqrt(diff[:, 0]**2 + diff[:, 1]**2)
        min_dist = main.r + radii
        # Find indices of orbit balls that are too close.
        colliding = dist < min_dist
        if np.any(colliding):
            idx = np.where(colliding)[0]
            # Avoid division by zero by replacing zeros with ones temporarily.
            safe_dist = np.where(dist[idx] == 0, 1, dist[idx])
            # Normalized collision vectors.
            norm = diff[idx] / safe_dist[:, None]
            # Reset velocity for collided balls.
            velocities[idx] = 0
            # Place the orbit ball exactly at the collision boundary.
            positions[idx] = np.array(
                [main.x, main.y]) + norm * min_dist[idx, None]

    # Update the canvas items.
    for i, cid in enumerate(canvas_ids):
        r = radii[i]
        x, y = positions[i]
        canvas.coords(cid, x - r, y - r, x + r, y + r)
        # Update color based on speed.
        speed = math.hypot(velocities[i, 0], velocities[i, 1])

        norm_val = min(speed / max_speed_color, 1)
        col_val = int(255 * norm_val)
        color = f"#{col_val:02x}{col_val:02x}{col_val:02x}"
        canvas.itemconfig(cid, fill=color)

    root.after(int(dt * sim_speed), update_simulation)


if __name__ == '__main__':
    # -------------------------------
    # Setup main Tkinter window and canvas
    # -------------------------------
    WIDTH, HEIGHT = 1000, 600

    root = tk.Tk()
    root.title("Vectorized Orbiting Balls Simulation")
    # root.attributes('-fullscreen', True)

    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT,
                       bg="black", cursor='dot', bd=0)
    canvas.pack()

    # Create a single draggable main ball.
    main_ball = DraggableBall(canvas, x=200, y=300, r=30, color="white")
    # main_ball2 = DraggableBall(canvas, x=100, y=100, r=30, color="grey")

    main_balls = [main_ball]  # in case you want to add more later

    # -------------------------------
    # Setup orbiting balls in vectorized arrays.
    # -------------------------------
    N_ORBIT = 1500  # You can increase this number

    # Each orbiting ball has:
    #   position: (x, y)
    #   velocity: (vx, vy)
    #   radius: r
    #   canvas id: used for drawing
    positions = np.zeros((N_ORBIT, 2), dtype=np.float64)
    velocities = np.zeros((N_ORBIT, 2), dtype=np.float64)
    radii = np.full(N_ORBIT, 1.5, dtype=np.float64)
    canvas_ids = np.empty(N_ORBIT, dtype=np.int32)

    # Initialize orbiting balls: position them around a randomly chosen main ball.
    for i in range(N_ORBIT):
        main = random.choice(main_balls)
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(50, 100)
        x = main.x + distance * math.cos(angle)
        y = main.y + distance * math.sin(angle)
        positions[i] = (x, y)
        # Give a random initial velocity
        velocities[i] = (random.uniform(-50, 50), random.uniform(-50, 50))
        # Create the ball on the canvas.
        canvas_ids[i] = canvas.create_oval(x - radii[i], y - radii[i],
                                           x + radii[i], y + radii[i],
                                           fill="grey", outline="")

    drag_data = {"ball": None, "offset_x": 0, "offset_y": 0}
    canvas.bind("<ButtonPress-1>", on_button_press)
    canvas.bind("<B1-Motion>", on_mouse_drag)
    canvas.bind("<ButtonRelease-1>", on_button_release)

    # Start the simulation.
    update_simulation()
    root.mainloop()
