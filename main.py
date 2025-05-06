import numpy as np

# Parameters
g = 9.8  # m/s^2
rho_air = 1.225  # kg/m^3
Cd = 0.47
r = 0.001  # m
rho_water = 1000  # kg/m^3
m = (4/3) * np.pi * r**3 * rho_water
A = np.pi * r**2
dt = 0.001  # Time step
y = 0  # Initial position
v = 0  # Initial velocity
t = 0  # Initial time

# Simulation
while y > -250:
    Fd = 0.5 * rho_air * Cd * A * v**2
    a = g - Fd / m
    v += a * dt
    y -= v * dt  # Corrected line: subtract v * dt from y
    t += dt

print(f"Time to fall 250m with drag: {t:.2f} seconds")
