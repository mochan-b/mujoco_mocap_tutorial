import argparse
import mujoco
import mujoco.viewer


def main(xml_file, body):
    # Load the mujoco model basic.xml
    model = mujoco.MjModel.from_xml_path(xml_file)
    data = mujoco.MjData(model)

    # Get the ID of the body we want to track
    body_id = model.body(body).id

    # Do forward kinematics
    mujoco.mj_kinematics(model, data)

    # Get the position of the body from the data
    body_pos = data.xpos[body_id]
    body_quat = data.xquat[body_id]
    print(body_pos, body_quat)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == '__main__':
    # Parse command line args. xml_file is the path to the xml file to load.
    parser = argparse.ArgumentParser()
    parser.add_argument('xml_file', help='Path to the xml file to load')
    parser.add_argument('body', help='Name of the body to find position of')
    args = parser.parse_args()

    main(args.xml_file, body=args.body)
