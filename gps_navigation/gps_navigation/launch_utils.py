"""Helpers the launch files use to adapt the tuned Nav2 YAML to a robot."""

import re
import tempfile

DEFAULT_BASE_FRAME = 'base_footprint'


def render_nav2_params(
    source_file: str,
    bt_dir: str,
    base_frame: str = DEFAULT_BASE_FRAME,
    scan_topic: str = '/scan',
    backup_scan_topic: str = '',
) -> str:
    """
    Write a robot-specific copy of a tuned Nav2 params file and return its path.

    The tuned values are never touched; only names are substituted:
      - @BT_DIR@                 -> this package's installed behavior_trees dir
      - base_footprint           -> base_frame
      - costmap `topic: /scan`   -> scan_topic
      - costmap `topic: /scan_raw` -> backup_scan_topic, or the second LiDAR
        source is dropped from observation_sources when it is empty
    """
    with open(source_file, 'r', encoding='utf-8') as params_file:
        text = params_file.read()

    text = text.replace('@BT_DIR@', bt_dir)
    if base_frame and base_frame != DEFAULT_BASE_FRAME:
        text = re.sub(r'\bbase_footprint\b', base_frame, text)

    # /scan first: the lookahead keeps /scan_raw untouched.
    text = re.sub(
        r'(topic:\s*)/scan(?![\w/])',
        lambda match: match.group(1) + scan_topic,
        text,
    )
    if backup_scan_topic:
        text = re.sub(
            r'(topic:\s*)/scan_raw\b',
            lambda match: match.group(1) + backup_scan_topic,
            text,
        )
    else:
        text = re.sub(r'(observation_sources:\s*lidar)\s+scan_raw\b', r'\1', text)

    rendered = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='gps_navigation_nav2_',
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    with rendered:
        rendered.write(text)
    return rendered.name
