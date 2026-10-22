"""Apply the shared measured geometry to the MuJoCo model template."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_PATH = ROOT/"ros2_ws/src/balancing_robot_sim/config/robot_geometry.json"
MODEL_PATH = Path(__file__).with_name("robot.xml")


def model_xml():
    geometry = json.loads(GEOMETRY_PATH.read_text())
    root = ET.parse(MODEL_PATH).getroot()
    body = root.find("worldbody/body[@name='base']")
    body.set("pos", f"0 0 {geometry['wheel_radius_m']+.001:.12g}")
    box = body.find("geom[@name='body']")
    com = geometry["body_height_m"]/2-geometry["body_bottom_below_axle_m"]
    box.set("pos", f"0 0 {com:.12g}")
    box.set("size", " ".join(f"{geometry[key]/2:.12g}" for key in
                             ("body_depth_m", "body_width_m", "body_height_m")))
    box.set("mass", str(geometry["body_mass_kg"]))
    body.find("site[@name='imu']").set("pos", f"0 0 {geometry['imu_above_axle_m']}")
    root.find("option").set("gravity", f"0 0 {-geometry['gravity_m_s2']}")
    for side, sign in (("left", 1), ("right", -1)):
        wheel = body.find(f"body[@name='{side}_wheel']")
        wheel.set("pos", f"0 {sign*geometry['wheel_center_separation_m']/2:.12g} 0")
        geom = wheel.find("geom")
        geom.set("size", f"{geometry['wheel_radius_m']} {geometry['wheel_width_m']/2}")
        geom.set("mass", str(geometry["wheel_mass_each_kg"]))
    return ET.tostring(root, encoding="unicode")


def load_model():
    import mujoco
    return mujoco.MjModel.from_xml_string(model_xml())
