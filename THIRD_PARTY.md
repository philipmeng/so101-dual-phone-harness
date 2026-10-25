# Third-party components

The harness code is MIT licensed. Dependencies retain their own licenses.

`models/SO101/` contains unmodified URDF and mesh files from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/a758567c3978dfeefe282ede0500085a48fe8f78/Simulation/SO101),
commit `a758567c3978dfeefe282ede0500085a48fe8f78`, under Apache-2.0.
The upstream license is included at `models/SO101/LICENSE`; source details and
file hashes are recorded in `models/SO101/source.json`. The MIT license does
not replace the license on these assets.

The harness imports the [LeRobot](https://github.com/huggingface/lerobot) motor
bus and kinematics APIs, and [Placo](https://github.com/Rhoban/placo) through
LeRobot. Neither dependency's source is vendored here.

HEBI Mobile I/O and the [HEBI Python API](https://docs.hebi.us/tools.html#python-api)
are external software. Install and use them under their applicable terms;
this repository does not redistribute their SDK binaries or mobile apps.

This project is independent of HEBI Robotics, Hugging Face, and The Robot Studio.
