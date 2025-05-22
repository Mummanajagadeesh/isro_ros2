import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_camera_trajectories(file_paths, labels, colors):
    """
    Plot multiple camera trajectories from trajectory files.
    
    Args:
        file_paths: List of paths to trajectory files
        labels: List of labels for each trajectory
        colors: List of colors for each trajectory
    """
    # Create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Also create a 2D top-down view (X-Y plane)
    fig2d = plt.figure(figsize=(12, 10))
    ax2d = fig2d.add_subplot(111)
    
    # Track min/max values for axis scaling
    all_positions = []
    all_timestamps = []
    
    # Process each trajectory file
    for i, file_path in enumerate(file_paths):
        # Load trajectory data
        try:
            data = np.loadtxt(file_path)
            print(f"Loaded trajectory {labels[i]} with {len(data)} poses")
        except Exception as e:
            print(f"Error loading trajectory file {file_path}: {e}")
            continue
        
        # Extract positions
        if data.shape[1] >= 4:  # Ensure we have at least timestamp, tx, ty, tz
            timestamps = data[:, 0]
            positions = data[:, 1:4]  # tx, ty, tz
            
            # Store for later scaling
            all_positions.append(positions)
            all_timestamps.append(timestamps)
        else:
            print(f"Invalid data format in {file_path}. Expected at least 4 columns, got {data.shape[1]}")
            continue
        
        # Plot the trajectory in 3D
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                color=colors[i], linewidth=2, label=f"{labels[i]} Path")
        
        # Mark start and end points in 3D
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                   color=colors[i], marker='o', s=100, label=f"{labels[i]} Start")
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                   color=colors[i], marker='s', s=100, label=f"{labels[i]} End")
        
        # Add colormap scatter for time progression in 3D
        # Use valid colormaps based on color names
        if colors[i] == "blue":
            cmap_name = "Blues_r"
        elif colors[i] == "red":
            cmap_name = "Reds_r"
        else:
            cmap_name = "viridis_r"  # Default fallback

        scatter = ax.scatter(
            positions[:, 0], positions[:, 1], positions[:, 2],
            c=timestamps, cmap=cmap_name, s=15, alpha=0.6
        )

        # Plot in 2D (top-down view)
        ax2d.plot(positions[:, 0], positions[:, 1], 
                  color=colors[i], linewidth=2, label=f"{labels[i]} Path")
        
        # Mark start and end points in 2D
        ax2d.scatter(positions[0, 0], positions[0, 1], 
                     color=colors[i], marker='o', s=100, label=f"{labels[i]} Start")
        ax2d.scatter(positions[-1, 0], positions[-1, 1], 
                     color=colors[i], marker='s', s=100, label=f"{labels[i]} End")
        
        if colors[i] == "blue":
            cmap_name = "Blues_r"
        elif colors[i] == "red":
            cmap_name = "Reds_r"
        else:
            cmap_name = "viridis_r"  # Default fallback
            
        scatter2d = ax2d.scatter(
            positions[:, 0], positions[:, 1],
            c=timestamps, cmap=cmap_name, s=15, alpha=0.6
        )

    # Combine all positions for scaling
    if all_positions:
        all_pos = np.vstack(all_positions)
        
        # Set equal aspect ratio for 3D plot
        max_range = np.max([
            np.max(all_pos[:, 0]) - np.min(all_pos[:, 0]),
            np.max(all_pos[:, 1]) - np.min(all_pos[:, 1]),
            np.max(all_pos[:, 2]) - np.min(all_pos[:, 2])
        ])
        mid_x = (np.max(all_pos[:, 0]) + np.min(all_pos[:, 0])) / 2
        mid_y = (np.max(all_pos[:, 1]) + np.min(all_pos[:, 1])) / 2
        mid_z = (np.max(all_pos[:, 2]) + np.min(all_pos[:, 2])) / 2
        ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
        
        # Add a single colorbar for the 3D plot
        if len(all_timestamps) > 0:
            # Use the first trajectory's timestamps for the colorbar
            norm = plt.Normalize(min([min(ts) for ts in all_timestamps]), 
                                max([max(ts) for ts in all_timestamps]))
            sm = plt.cm.ScalarMappable(cmap='viridis', norm=norm)
            sm.set_array([])
            cbar = fig.colorbar(sm, ax=ax, pad=0.1)
            cbar.set_label('Timestamp')
    
    # Set labels and title for 3D plot
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Camera Trajectories Comparison (3D View)')
    
    # Set an initial view angle
    ax.view_init(elev=30, azim=45)
    
    # Add legend with a smaller font size and outside the plot
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=8)
    
    # Set labels and title for 2D plot
    ax2d.set_xlabel('X (m)')
    ax2d.set_ylabel('Y (m)')
    ax2d.set_title('Camera Trajectories Comparison (Top-Down View)')
    ax2d.grid(True)
    ax2d.axis('equal')
    
    # Add legend to 2D plot
    ax2d.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=8)
    
    # Add a colorbar for the 2D plot
    if len(all_timestamps) > 0:
        norm = plt.Normalize(min([min(ts) for ts in all_timestamps]), 
                            max([max(ts) for ts in all_timestamps]))
        sm = plt.cm.ScalarMappable(cmap='viridis', norm=norm)
        sm.set_array([])
        cbar2d = fig2d.colorbar(sm, ax=ax2d, pad=0.1)
        cbar2d.set_label('Timestamp')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Trajectory file paths
    trajectory_files = [
        "full_drone_vid_traj.txt",
        "hovering_part_drone_vid_traj.txt"
    ]
    
    # Labels for each trajectory
    labels = ["Full Trajectory", "Hovering Part"]
    
    # Colors for each trajectory
    colors = ["blue", "red"]
    
    # Plot both trajectories
    plot_camera_trajectories(trajectory_files, labels, colors)

