import mujoco
import mujoco.viewer

from mocap_move import key_callback_data


def main():
    # Load the mujoco model basic.xml
    model = mujoco.MjModel.from_xml_path('google_robot_scene.xml')
    data = mujoco.MjData(model)

    def key_callback(key):
        key_callback_data(key, data)

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == '__main__':
    main()
