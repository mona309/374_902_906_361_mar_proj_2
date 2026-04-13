#!/usr/bin/env python3
"""
True 3D Voxel Mapping for Drone Motion Planning
Creates a 3D voxel grid where each cell is marked as obstacle (1) or free (0)
"""

import numpy as np
import matplotlib.pyplot as plt
from planning_utils import create_grid, a_star, heuristic, path_prune, collinear_points

def create_voxmap(data, voxel_size=5):
    """Create a 3D voxel map from obstacle data."""
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    
    alt_max = np.ceil(np.max(data[:, 2] + data[:, 5]))
    
    north_size = int(np.ceil(north_max - north_min)) // voxel_size + 1
    east_size = int(np.ceil(east_max - east_min)) // voxel_size + 1
    alt_size = int(alt_max) // voxel_size + 1
    
    print(f"Voxel map size: {north_size} x {east_size} x {alt_size}")
    
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=bool)
    
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        n_start = max(0, int((north - d_north - north_min) // voxel_size))
        n_end = min(north_size - 1, int((north + d_north - north_min) // voxel_size) + 1)
        e_start = max(0, int((east - d_east - east_min) // voxel_size))
        e_end = min(east_size - 1, int((east + d_east - east_min) // voxel_size) + 1)
        a_end = min(alt_size - 1, int((alt + d_alt) // voxel_size) + 1)
        
        voxmap[n_start:n_end, e_start:e_end, 0:a_end] = True
    
    return voxmap, int(north_min), int(east_min), voxel_size

def visualize_3d_volume(voxmap, north_min, east_min, voxel_size):
    """Create slices visualization"""
    num_slices = min(15, voxmap.shape[2])
    rows = (num_slices + 4) // 5
    cols = 5
    
    fig, axes = plt.subplots(rows, cols, figsize=(20, 4*rows))
    axes = axes.flatten() if num_slices > 1 else [axes]
    
    for i in range(num_slices):
        axes[i].imshow(voxmap[:, :, i].T, origin='lower', cmap='binary_r')
        axes[i].set_title(f'Alt {i*voxel_size}m', fontsize=10)
        axes[i].set_xlabel('East')
        axes[i].set_ylabel('North')
        axes[i].set_aspect('equal')
    
    for i in range(num_slices, len(axes)):
        axes[i].axis('off')
    
    plt.suptitle('3D VOXEL MAP - Altitude Slices')
    plt.tight_layout()
    plt.savefig('voxel_3d_slices.png', dpi=100)
    plt.close()
    print("Saved voxel_3d_slices.png")
    
    fig, ax = plt.subplots(figsize=(12, 12))
    xy_proj = np.max(voxmap, axis=2).T
    ax.imshow(xy_proj, origin='lower', cmap='binary_r')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('3D Voxel Map - XY Maximum')
    plt.savefig('voxel_xy_max.png', dpi=100)
    plt.close()
    print("Saved voxel_xy_max.png")

def heuristic_3d(position, goal):
    """3D Euclidean heuristic"""
    return np.sqrt(sum((a - b) ** 2 for a, b in zip(position, goal)))

def a_star_3d(voxmap, start, goal):
    """3D A* search"""
    from queue import PriorityQueue
    
    print(f"3D A* from {start} to {goal}")
    
    if start[0] >= voxmap.shape[0] or start[1] >= voxmap.shape[1] or start[2] >= voxmap.shape[2]:
        return None
    if goal[0] >= voxmap.shape[0] or goal[1] >= voxmap.shape[1] or goal[2] >= voxmap.shape[2]:
        return None
    
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set([start])
    branch = {start: None}
    
    moves = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]
    
    while not queue.empty():
        cost, current = queue.get()
        if current == goal:
            path = [goal]
            while branch[path[-1]]:
                path.append(branch[path[-1]])
            path.reverse()
            print(f"3D path found! Length: {len(path)}")
            return path
        
        for dn, de, da in moves:
            next_pos = (current[0]+dn, current[1]+de, current[2]+da)
            if (0 <= next_pos[0] < voxmap.shape[0] and 0 <= next_pos[1] < voxmap.shape[1] and 0 <= next_pos[2] < voxmap.shape[2]):
                if not voxmap[next_pos] and next_pos not in visited:
                    new_cost = cost + 1 + heuristic_3d(next_pos, goal)
                    visited.add(next_pos)
                    branch[next_pos] = current
                    queue.put((new_cost, next_pos))
    
    print("No 3D path found")
    return None

if __name__ == "__main__":
    print("=" * 50)
    print("3D VOXEL MAPPING")
    print("=" * 50)
    
    data = np.loadtxt('colliders.csv', delimiter=',', dtype=np.float64, skiprows=2)
    print(f"Loaded {data.shape[0]} obstacles")
    
    voxel_size = 10
    voxmap, north_min, east_min, voxel_size = create_voxmap(data, voxel_size)
    
    print(f"Voxel map created: {voxmap.shape}")
    print(f"Obstacle voxels: {np.sum(voxmap)}")
    print(f"Free voxels: {np.sum(~voxmap)}")
    
    print("\nCreating visualizations...")
    visualize_3d_volume(voxmap, north_min, east_min, voxel_size)
    
    print("\n3D Path Planning")
    start = (10, 10, 1)
    goal = (20, 20, 1)
    path = a_star_3d(voxmap, start, goal)
    
    if path:
        print(f"Path: {path}")