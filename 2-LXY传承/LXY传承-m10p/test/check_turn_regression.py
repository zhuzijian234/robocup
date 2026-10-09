"""Compile actual portable geometry/controller with real Cartesian inputs."""
import json
from check_m10p import P, OUT, run
result = run('path_track', P/'test/test_path_track.c')
(OUT/'path_track_summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
