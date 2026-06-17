#!/usr/bin/env python3
import math
import sys

import casadi as ca
import numpy as np


def read_input(path):
    tokens = open(path, "r", encoding="utf-8").read().split()
    i = 0

    def expect(name):
        nonlocal i
        if i >= len(tokens) or tokens[i] != name:
            raise RuntimeError(f"expected {name}, got {tokens[i] if i < len(tokens) else '<eof>'}")
        i += 1

    expect("CONFIG")
    cfg_names = [
        "wheelbase", "max_steer", "max_velocity", "max_acceleration",
        "comfort_weight", "constraint_penalty", "ipopt_max_iterations", "ipopt_tol",
        "infeasibility_tolerance", "robot_length",
    ]
    cfg = {}
    for name in cfg_names:
        cfg[name] = float(tokens[i])
        i += 1
    cfg["ipopt_max_iterations"] = int(cfg["ipopt_max_iterations"])

    expect("POINTS")
    n = int(tokens[i])
    i += 1
    points = []
    for _ in range(n):
        x = float(tokens[i]); y = float(tokens[i + 1]); yaw = float(tokens[i + 2])
        i += 3
        points.append((x, y, yaw))

    def read_corridors(name):
        nonlocal i
        expect(name)
        count = int(tokens[i]); i += 1
        values = []
        for _ in range(count):
            row = tuple(float(tokens[i + j]) for j in range(10))
            i += 10
            values.append(row)
        return values

    center = read_corridors("CENTER_CORRIDORS")
    front = read_corridors("FRONT_CORRIDORS")
    rear = read_corridors("REAR_CORRIDORS")
    return cfg, points, center, front, rear


def norm_angle_np(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def corridor_constraints(opti, px, py, corridor):
    cx, cy, tx, ty, nx, ny, backward, forward, right, left = corridor
    dx = px - cx
    dy = py - cy
    s = dx * tx + dy * ty
    l = dx * nx + dy * ny
    opti.subject_to(s >= -backward)
    opti.subject_to(s <= forward)
    opti.subject_to(l >= -right)
    opti.subject_to(l <= left)


def make_initial_guess(cfg, points):
    n_points = len(points)
    n = n_points - 1
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    th = np.unwrap(np.array([p[2] for p in points]))
    seg_len = np.hypot(np.diff(xs), np.diff(ys))
    total_len = float(seg_len.sum())
    t_guess = max(total_len / max(cfg["max_velocity"], 1e-3), 5.0)
    dt = t_guess / max(n, 1)

    v_interval = np.zeros(n)
    for k in range(n):
        dx = xs[k + 1] - xs[k]
        dy = ys[k + 1] - ys[k]
        speed = math.hypot(dx, dy) / max(dt, 1e-6)
        sign = 1.0 if dx * math.cos(th[k]) + dy * math.sin(th[k]) >= 0.0 else -1.0
        v_interval[k] = min(max(sign * speed, -cfg["max_velocity"]), cfg["max_velocity"])

    v_guess = np.zeros(n_points)
    if n > 1:
        v_guess[1:-1] = 0.5 * (v_interval[:-1] + v_interval[1:])

    phi_init = np.zeros(n_points)
    for k in range(1, n_points - 1):
        if abs(v_guess[k]) < 1e-4:
            continue
        yaw_rate = norm_angle_np(th[k + 1] - th[k - 1]) / max(2.0 * dt, 1e-6)
        phi_init[k] = math.atan(yaw_rate * cfg["wheelbase"] / (2.0 * v_guess[k]))
    phi_init = np.clip(phi_init, -cfg["max_steer"], cfg["max_steer"])
    phi_init[0] = 0.0
    phi_init[-1] = 0.0

    u_guess = np.zeros((2, n))
    if n > 0:
        u_guess[0, :] = np.clip(
            np.diff(v_guess) / max(dt, 1e-6),
            -cfg["max_acceleration"], cfg["max_acceleration"])
        dphi = np.array([norm_angle_np(phi_init[k + 1] - phi_init[k]) for k in range(n)])
        u_guess[1, :] = dphi / max(dt, 1e-6)
    return t_guess, v_guess, phi_init, u_guess


def solve_all(cfg, points, front_corridors, rear_corridors):
    n_points = len(points)
    if n_points < 3:
        length = sum(math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
                     for i in range(n_points - 1))
        return points, length / max(cfg["max_velocity"], 1e-3), True, 0

    n = n_points - 1
    nx = 9
    nu = 2
    alpha = 0.25 * cfg["robot_length"]
    beta = 0.25 * cfg["robot_length"]
    t_guess, v_guess, phi_guess, u_guess = make_initial_guess(cfg, points)

    opti = ca.Opti()
    X = opti.variable(nx, n_points)
    U = opti.variable(nu, n)
    T = opti.variable()

    x = X[0, :]
    y = X[1, :]
    theta = X[2, :]
    v = X[3, :]
    phi = X[4, :]
    xf = X[5, :]
    yf = X[6, :]
    xr = X[7, :]
    yr = X[8, :]
    a = U[0, :]
    omega = U[1, :]
    h = T / n

    def dyn(z, u):
        th = z[2]
        vel = z[3]
        ph = z[4]
        acc = u[0]
        om = u[1]
        dx = vel * ca.cos(th)
        dy = vel * ca.sin(th)
        dth = 2.0 * vel * ca.tan(ph) / max(cfg["wheelbase"], 1e-3)
        return ca.vertcat(dx, dy, dth, acc, om)

    objective = T
    for k in range(n):
        objective += cfg["comfort_weight"] * (v[k] * omega[k]) ** 2 * h
        opti.subject_to(X[0:5, k + 1] == X[0:5, k] + dyn(X[0:5, k], U[:, k]) * h)

    geom = 0
    for k in range(n_points):
        xf_nom = x[k] + alpha * ca.cos(theta[k])
        yf_nom = y[k] + alpha * ca.sin(theta[k])
        xr_nom = x[k] - beta * ca.cos(theta[k])
        yr_nom = y[k] - beta * ca.sin(theta[k])
        geom += ((xf[k] - xf_nom) ** 2 + (yf[k] - yf_nom) ** 2 +
                 (xr[k] - xr_nom) ** 2 + (yr[k] - yr_nom) ** 2) * h
        corridor_constraints(opti, xf[k], yf[k], front_corridors[k])
        corridor_constraints(opti, xr[k], yr[k], rear_corridors[k])

    theta_goal = points[-1][2]
    terminal = (ca.sin(theta[-1]) - math.sin(theta_goal)) ** 2 + (
        ca.cos(theta[-1]) - math.cos(theta_goal)) ** 2
    objective += cfg["constraint_penalty"] * (geom + terminal)

    for k in range(n_points):
        opti.subject_to(v[k] >= -cfg["max_velocity"])
        opti.subject_to(v[k] <= cfg["max_velocity"])
        opti.subject_to(phi[k] >= -cfg["max_steer"])
        opti.subject_to(phi[k] <= cfg["max_steer"])
    for k in range(n):
        opti.subject_to(a[k] >= -cfg["max_acceleration"])
        opti.subject_to(a[k] <= cfg["max_acceleration"])

    opti.subject_to(x[0] == points[0][0])
    opti.subject_to(y[0] == points[0][1])
    opti.subject_to(theta[0] == points[0][2])
    opti.subject_to(v[0] == 0.0)
    opti.subject_to(phi[0] == 0.0)
    opti.subject_to(x[-1] == points[-1][0])
    opti.subject_to(y[-1] == points[-1][1])
    opti.subject_to(v[-1] == 0.0)
    opti.subject_to(phi[-1] == 0.0)
    opti.subject_to(T >= 1e-3)

    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    th = np.unwrap(np.array([p[2] for p in points]))
    xf_guess = xs + alpha * np.cos(th)
    yf_guess = ys + alpha * np.sin(th)
    xr_guess = xs - beta * np.cos(th)
    yr_guess = ys - beta * np.sin(th)
    opti.set_initial(X, np.vstack([
        xs, ys, th, v_guess, phi_guess,
        xf_guess, yf_guess, xr_guess, yr_guess,
    ]))
    opti.set_initial(U, u_guess)
    opti.set_initial(T, t_guess)
    opti.minimize(objective)
    opti.solver(
        "ipopt",
        {"print_time": False},
        {
            "print_level": 0,
            "max_iter": int(cfg["ipopt_max_iterations"]),
            "tol": max(cfg["ipopt_tol"], 1e-12),
            "acceptable_tol": max(cfg["infeasibility_tolerance"], cfg["ipopt_tol"], 1e-12),
        },
    )

    try:
        sol = opti.solve()
        X_sol = sol.value(X)
        T_sol = float(sol.value(T))
        ok = True
        status = 0
    except RuntimeError:
        X_sol = opti.debug.value(X)
        T_sol = float(opti.debug.value(T))
        ok = False
        status = -1

    out = []
    for k in range(n_points):
        out.append((float(X_sol[0, k]), float(X_sol[1, k]), norm_angle_np(float(X_sol[2, k]))))
    return out, T_sol, ok, status


def write_output(path, states, total_time, ok, status):
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"STATUS {1 if ok else 0} {status} {total_time:.17g}\n")
        f.write(f"STATES {len(states)}\n")
        for x, y, yaw in states:
            f.write(f"{x:.17g} {y:.17g} {yaw:.17g}\n")


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: casadi_backend.py input output")
    cfg, points, _center, front, rear = read_input(sys.argv[1])
    states, total_time, ok, status = solve_all(cfg, points, front, rear)
    write_output(sys.argv[2], states, total_time, ok, status)


if __name__ == "__main__":
    main()
