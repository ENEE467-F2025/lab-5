#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser(
        description="Generate and plot a cubic joint-space trajectory for a 3R robot."
    )
    parser.add_argument(
        "--numpoints", type=int, default=100,
        help="Number of interpolation points along the trajectory (default: 100)"
    )
    parser.add_argument(
        "--tf", type=float, default=8.0,
        help="Final time for the trajectory in seconds (default: 8.0s)"
    )
    args = parser.parse_args()
    tf_arg = args.tf

    # Parameters
    q0 = np.array([0.0, 0.5, 1.0])      # start configuration
    qf = np.array([1.0, 1.0, 0.5])      # goal configuration
    t0, tf = 0.0, tf_arg                # start and end times
    num_points = args.numpoints

    # Compute cubic coefficients
    A = np.array([
        [1, t0, t0**2, t0**3],
        [0, 1, 2*t0, 3*t0**2],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*tf**2]
    ])

    v0 = np.zeros_like(q0)
    vf = np.zeros_like(qf)
    b = np.vstack([q0, v0, qf, vf])

    coeffs = np.linalg.solve(A, b)

    # Evaluate trajectory
    times = np.linspace(t0, tf, num_points)
    T = times[:, None]

    positions = coeffs[0] + coeffs[1]*T + coeffs[2]*T**2 + coeffs[3]*T**3
    velocities = coeffs[1] + 2*coeffs[2]*T + 3*coeffs[3]*T**2

    # Plot results
    fig, axs = plt.subplots(2, 1, figsize=(6, 4), sharex=True)
    joint_labels = [r"$q_1$", r"$q_2$", r"$q_3$"]

    for i in range(3):
        axs[0].plot(times, positions[:, i], label=joint_labels[i])
        axs[1].plot(times, velocities[:, i], label=joint_labels[i])

    axs[0].set_ylabel(r"Position, $q$ (rad)")
    axs[1].set_ylabel(r"Velocity, $v$ (rad/s)")
    axs[1].set_xlabel("Time (s)")

    axs[0].set_title("Cubic Polynomial Joint Trajectory for a 3R Robot")
    for ax in axs:
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
