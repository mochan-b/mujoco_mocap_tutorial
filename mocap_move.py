import mujoco
import mujoco.viewer
import numpy as np
import pyquaternion as pyq


def rotate_quaternion(quat, axis, angle):
    """
    Rotate a quaternion by an angle around an axis
    """
    angle_rad = np.deg2rad(angle)
    axis = axis / np.linalg.norm(axis)
    q = pyq.Quaternion(quat)
    q = q * pyq.Quaternion(axis=axis, angle=angle_rad)
    return q.elements


def main():
    # Load the mujoco model basic.xml
    model = mujoco.MjModel.from_xml_path('mocap.xml')
    data = mujoco.MjData(model)

    def key_callback(key):
        if key == 265:  # Up arrow
            data.mocap_pos[0, 2] += 0.01
        elif key == 264:  # Down arrow
            data.mocap_pos[0, 2] -= 0.01
        elif key == 263:  # Left arrow
            data.mocap_pos[0, 0] -= 0.01
        elif key == 262:  # Right arrow
            data.mocap_pos[0, 0] += 0.01
        elif key == 320:  # Numpad 0
            data.mocap_pos[0, 1] += 0.01
        elif key == 330:  # Numpad .
            data.mocap_pos[0, 1] -= 0.01
        elif key == 260:  # Insert
            data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [1, 0, 0], 10)
        elif key == 261:  # Home
            data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [1, 0, 0], -10)
        elif key == 268:  # Home
            data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 1, 0], 10)
        elif key == 269:  # End
            data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 1, 0], -10)
        elif key == 266:  # Page Up
            data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 0, 1], 10)
        elif key == 267:  # Page Down
            data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 0, 1], -10)
        else:
            print(key)

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == '__main__':
    main()
