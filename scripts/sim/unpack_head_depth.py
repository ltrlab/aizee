#!/usr/bin/env python3
"""Unpack a grab_head_depth.py result into what head_depth_check.py reads.

    python scripts/sim/unpack_head_depth.py <result.json> <out dir>

<result.json> is robot_remote.py's JSON ({stdout, stderr, exit}) from running grab_head_depth.py on the Jetson.
Writes depth_median.npy (uint16, depth units), depth_count.npy, depth_mad.npy, head.jpg and header.json into <out dir>.
"""

import base64
import json
from pathlib import Path
import sys
import zlib

import numpy as np

result_path, out = Path(sys.argv[1]), Path(sys.argv[2])
result = json.loads(result_path.read_text(encoding='utf-8'))
if result['exit']:
    raise SystemExit('remote grab failed: ' + result['stderr'][-2000:])
data = json.loads(result['stdout'])
out.mkdir(parents=True, exist_ok=True)
for name in ('median', 'count', 'mad'):
    entry = data[name]
    array = np.frombuffer(zlib.decompress(base64.b64decode(entry['zlib_base64'])), dtype=entry['dtype'])
    np.save(out / f'depth_{name}.npy', array.reshape(entry['shape']))
(out / 'head.jpg').write_bytes(base64.b64decode(data['color_jpeg_base64']))
(out / 'header.json').write_text(
    json.dumps({'frames': data['frames'], 'stamps': data['stamps'], 'header': data['header']}, indent=1),
    encoding='utf-8',
)
print(data['frames'], 'frames;', json.dumps(data['header']['depth'])[:300])
