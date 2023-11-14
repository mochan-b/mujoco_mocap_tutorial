import mujoco
import mujoco.viewer


def main():
    # Load the mujoco model basic.xml
    model = mujoco.MjModel.from_xml_path('mocap.xml')
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == '__main__':
    main()
