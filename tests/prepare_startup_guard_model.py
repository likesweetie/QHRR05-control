"""Keep current CAD inertials/collisions; omit rendering assets for startup-only tests."""

from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

scene, config, output = map(Path, sys.argv[1:])
output.mkdir(parents=True, exist_ok=True)
robot_name = ET.parse(scene).getroot().find("include").get("file")
robot = ET.parse(scene.parent / robot_name)
root = robot.getroot()
for parent in root.iter():
    for child in list(parent):
        if child.tag == "mesh" or (child.tag == "geom" and "mesh" in child.attrib):
            parent.remove(child)
robot.write(output / robot_name, encoding="utf-8", xml_declaration=True)
(output / "scene.xml").write_bytes(scene.read_bytes())
control = re.sub(
    r"^model_path:.*$", f"model_path: {output / robot_name}", config.read_text(), flags=re.MULTILINE
)
(output / "control.yaml").write_text(control)
print("Startup guards: using current model inertials/collisions without rendering-only meshes")
