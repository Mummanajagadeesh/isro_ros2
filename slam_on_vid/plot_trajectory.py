import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_camera_trajectory(file_path):
    """
    Plot the camera trajectory from a trajectory file.
    
    The file should be in the format:
    timestamp tx ty tz qx qy qz qw
    
    Args:
        file_path: Path to the trajectory file
    """
    # Load trajectory data
    try:
        data = np.loadtxt(file_path)
        print(f"Loaded trajectory with {len(data)} poses")
    except Exception as e:
        print(f"Error loading trajectory file: {e}")
        return
    
    # Extract positions (we only need tx, ty, tz for position plotting)
    if data.shape[1] >= 4:  # Ensure we have at least timestamp, tx, ty, tz
        timestamps = data[:, 0]
        positions = data[:, 1:4]  # tx, ty, tz
    else:
        print(f"Invalid data format. Expected at least 4 columns, got {data.shape[1]}")
        return
    
    # Create 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the trajectory
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=1)
    
    # Mark start and end points
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='green', marker='o', s=100, label='Start')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='red', marker='o', s=100, label='End')
    
    # Set labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Camera Trajectory')
    
    # Add legend
    ax.legend()
    
    # Equal aspect ratio
    max_range = np.max([
        np.max(positions[:, 0]) - np.min(positions[:, 0]),
        np.max(positions[:, 1]) - np.min(positions[:, 1]),
        np.max(positions[:, 2]) - np.min(positions[:, 2])
    ])
    mid_x = (np.max(positions[:, 0]) + np.min(positions[:, 0])) / 2
    mid_y = (np.max(positions[:, 1]) + np.min(positions[:, 1])) / 2
    mid_z = (np.max(positions[:, 2]) + np.min(positions[:, 2])) / 2
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
    
    # Add a colormap to show progression over time
    scatter = ax.scatter(
        positions[:, 0], positions[:, 1], positions[:, 2],
        c=timestamps, cmap='viridis', s=10
    )
    
    # Add colorbar to show time progression
    cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
    cbar.set_label('Timestamp')
    
    # Set an initial view angle
    ax.view_init(elev=30, azim=45)
    
    plt.tight_layout()
    plt.show()
    
    # Also create a 2D top-down view (X-Z plane)
    plt.figure(figsize=(10, 8))
    plt.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=1)
    plt.scatter(positions[0, 0], positions[0, 2], c='green', marker='o', s=100, label='Start')
    plt.scatter(positions[-1, 0], positions[-1, 2], c='red', marker='o', s=100, label='End')
    plt.scatter(positions[:, 0], positions[:, 2], c=timestamps, cmap='viridis', s=10)
    plt.colorbar(label='Timestamp')
    plt.xlabel('X (m)')
    plt.ylabel('Z (m)')
    plt.title('Camera Trajectory (Top-Down View)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Replace with your trajectory file path
    trajectory_file = "trajectory.txt"
    plot_camera_trajectory(trajectory_file)
