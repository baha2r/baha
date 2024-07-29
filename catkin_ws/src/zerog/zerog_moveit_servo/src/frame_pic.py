#!/usr/bin/env python

import rospy
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_transform(listener, parent_frame, child_frame):
    try:
        # Lookup the transform at the latest available time
        (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        # Convert quaternion to rotation matrix
        rotation = tf.transformations.quaternion_matrix(rot)
        rotation = rotation[:3, :3]
        return trans, rotation
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return None, None

def plot_frame(ax, origin, rotation, label):
    ax.quiver(origin[0], origin[1], origin[2], rotation[0][0], rotation[0][1], rotation[0][2], color='r', length=0.1)
    ax.quiver(origin[0], origin[1], origin[2], rotation[1][0], rotation[1][1], rotation[1][2], color='g', length=0.1)
    ax.quiver(origin[0], origin[1], origin[2], rotation[2][0], rotation[2][1], rotation[2][2], color='b', length=0.1)
    ax.text(origin[0], origin[1], origin[2], label)

    # Add labels for x, y, z directions
    ax.text(origin[0] + rotation[0][0] * 0.1, origin[1] + rotation[0][1] * 0.1, origin[2] + rotation[0][2] * 0.1, 'x', color='r')
    ax.text(origin[0] + rotation[1][0] * 0.1, origin[1] + rotation[1][1] * 0.1, origin[2] + rotation[1][2] * 0.1, 'y', color='g')
    ax.text(origin[0] + rotation[2][0] * 0.1, origin[1] + rotation[2][1] * 0.1, origin[2] + rotation[2][2] * 0.1, 'z', color='b')

if __name__ == '__main__':
    rospy.init_node('tf_plotter', anonymous=True)
    listener = tf.TransformListener()

    # Wait for the listener to get the first message
    rospy.sleep(2.0)

    frames = ['ceiling_arm_tool0', 'wall_arm_tool0', 'capture_frame']
    parent_frame = 'world'
    frame_positions = {}
    
    for frame in frames:
        trans, rotation = get_transform(listener, parent_frame, frame)
        if trans is not None and rotation is not None:
            frame_positions[frame] = (trans, rotation)

    fig = plt.figure(figsize=(18, 6))
    
    # Subplot 1: XY view
    ax_xy = fig.add_subplot(131, projection='3d')
    ax_xy.view_init(elev=90, azim=-90)
    ax_xy.set_title('XY View')
    plot_frame(ax_xy, [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'world')
    
    # Subplot 2: YZ view
    ax_yz = fig.add_subplot(132, projection='3d')
    ax_yz.view_init(elev=0, azim=-90)
    ax_yz.set_title('YZ View')
    plot_frame(ax_yz, [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'world')
    
    # Subplot 3: XZ view
    ax_xz = fig.add_subplot(133, projection='3d')
    ax_xz.view_init(elev=0, azim=0)
    ax_xz.set_title('XZ View')
    plot_frame(ax_xz, [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'world')

    # Plot each frame in all views
    for frame, (trans, rotation) in frame_positions.items():
        plot_frame(ax_xy, trans, rotation, frame)
        plot_frame(ax_yz, trans, rotation, frame)
        plot_frame(ax_xz, trans, rotation, frame)

    # Draw lines between frames in all views
    for frame in frame_positions.keys():
        trans = frame_positions[frame][0]
        ax_xy.plot([0, trans[0]], [0, trans[1]], [0, trans[2]], color='k')
        ax_yz.plot([0, trans[0]], [0, trans[1]], [0, trans[2]], color='k')
        ax_xz.plot([0, trans[0]], [0, trans[1]], [0, trans[2]], color='k')

    plt.tight_layout()
    plt.savefig('frame_poses_views.png')
    # plt.show()
