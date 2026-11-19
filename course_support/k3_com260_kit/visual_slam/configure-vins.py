"""Apply synthetic camera and IMU calibration to the upstream VINS config."""
import re
from pathlib import Path
from sys import argv

base = Path(argv[1]).resolve()
source = base / "src/vins/config_pkg/config/euroc/euroc_config.yaml"
config = source.read_text()
values = {
    "output_path": f'"{base}/out/vins/"',
    "pose_graph_save_path": f'"{base}/out/vins/graph/"',
    "image_width": 640, "image_height": 480,
    "save_image": 0, "show_track": 0,
    "k1": 0., "k2": 0., "p1": 0., "p2": 0.,
    "fx": 440., "fy": 440., "cx": 319.5, "cy": 239.5,
}
for key, value in values.items():
    config = re.sub(rf"(?m)^(\s*{key}: )[^\n]+", rf"\g<1>{value}", config)
for field, value in {
    "extrinsicRotation": "[0,0,1,-1,0,0,0,-1,0]",
    "extrinsicTranslation": "[0,0,0]",
}.items():
    config = re.sub(rf"({field}:.*?data: )\[[^]]+\]", rf"\g<1>{value}", config, flags=re.S)
(base / "sim-vins.yaml").write_text(config)
