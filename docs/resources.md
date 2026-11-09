# Resources

Projects, articles and papers behind this work. Reviewed 2026-11-08 from search results;
check each link before relying on details.

## Open-source projects

The most reusable code is the KR260 contest work by iotengineer22 (DPU + ROS 2 + GStreamer on
the same K26 SOM) and the AMD Kria platform repos; drone-specific KV260 code is scarce.

| Project | Area | Why it helps |
| --- | --- | --- |
| [Xilinx/kria-vitis-platforms](https://github.com/Xilinx/kria-vitis-platforms) | KV260 platform | Vivado platforms and DPU overlays (`xlnx_rel_v2023.2` branch); base for a custom bitstream |
| [Xilinx/DPU-PYNQ](https://github.com/Xilinx/DPU-PYNQ) | DPU from Python | B4096 overlays compatible with Model Zoo KV260 models; fastest path to a first inference |
| [Xilinx/Vitis-AI](https://github.com/Xilinx/Vitis-AI) (3.5 tag) | Quantize/compile | Model Zoo, quantizer, compiler and VART runtime for the DPU |
| [Vitis Libraries — vision](https://xilinx.github.io/Vitis_Libraries/vision/2021.2/) | PL acceleration | HLS kernels for pyramidal optical flow, corner tracking, FAST/Harris, stereo, DNN preprocessing |
| [amd/Kria-RoboticsAI](https://github.com/amd/Kria-RoboticsAI) | ROS 2 + AI | Vision-guided robotics with ROS 2 and PYNQ/Vitis AI on K26 |
| [iotengineer22/AMD-Pervasive-AI-Developer-Contest](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest) | DPU + ROS 2 | YOLOX on DPU from a USB camera via GStreamer, publishing to ROS 2 Humble |
| [goodrobots/vision_landing](https://github.com/goodrobots/vision_landing) | Precision landing | ArUco pose → ArduCopter precision landing; discontinued, reference only |
| [tulparlabs/maverick](https://github.com/tulparlabs/maverick) | Companion computer | MAVLink companion-computer system integrating vision_landing |
| [mavlink/mavros](https://github.com/mavlink/mavros) · [PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) · [MAVSDK](https://github.com/mavlink/MAVSDK) | FC link | ROS 2 bridge (MAVLink), PX4 uXRCE-DDS examples, lightweight C++/Python MAVLink API |

## Articles and tutorials

Start with the PX4 precision-landing page and the shieldUAV write-up: together they cover the
flight-stack side and a K26-on-a-drone example.

| Article | Type | Key takeaway |
| --- | --- | --- |
| [PX4 — Precision Landing](https://docs.px4.io/main/en/advanced_features/precland.html) | Official docs | Offboard target via MAVLink `LANDING_TARGET` (`MAV_FRAME_LOCAL_NED`, x/y/z only); needs `landing_target_estimator` and a valid global position |
| [An Autonomous shieldUAV (KR260)](https://hackster.io/520087/an-autonomous-shielduav-to-protect-and-save-lives-99a722) | Hackster project | K26 companion on a PX4 drone: dataset, YOLO training, deployment on DPUCZDX8G, tracking |
| [AI-Enhanced UAV for Item Retrieval (KV260)](https://www.hackster.io/dipowin/ai-enhanced-uav-for-autonomous-item-retrieval-delivery-0ab5f2) | Hackster project | KV260 used directly as a UAV vision computer |
| [Counting Trees with KR260 on the Flying Edge](https://hackster.io/520027/almost-counting-trees-w-kr260-on-the-flying-edge-36b1da) | Hackster project | Onboard inference to send findings instead of images |
| [Object Detection with KR260 from a Webcam](https://hackster.io/iotengineer22/object-detection-with-kr260-from-a-webcam-85c261) | Hackster tutorial | YOLOX on DPU + USB camera + ROS 2, step by step |
| [WildSight-AI](https://www.hackster.io/matjaz4/wildsight-ai-real-time-human-wildlife-conflict-detection-ff65fa) | Hackster project | Custom YOLOv5 on DPU with Vitis AI 3.5 + VVAS 3.0 as a ROS 2 node |
| [Tria Vitis Platforms — Vitis-AI 3.5](https://community.element14.com/technologies/fpga-group/b/blog/posts/tria-vitis-platforms-adding-support-for-vitis-ai-3-5) | Blog series | Building KV260 platforms with Vitis AI 3.5, Hailo-8 and ROS 2 support |
| [Precision landing with companion computer (ArduPilot forum)](https://discuss.ardupilot.org/t/precision-landing-using-companion-computer-not-working/85097) | Forum thread | ArUco precision landing on ArduCopter 4.1.5 with `PLND_ENABLED=1`, `PLND_TYPE=1`; send `LANDING_TARGET` + `DISTANCE_SENSOR` |
| [Optically-Guided Multirotor Landing (UCSB)](https://www.ece.ucsb.edu/sites/default/files/2021-11/md-yz-yi-autonomous-landing-itc-2021.pdf) | Paper | AprilTag on a Pi companion; authors switched from PX4 to ArduPilot for landing-target support |
| [FPGA Feature Extraction and Tracking for V-SLAM](https://mdpi.com/1424-8220/23/19/8035) | Paper (open access) | FAST + pyramidal optical flow on Zynq UltraScale+; reference for a PL-accelerated VO front-end |
| [Hardware-Aware Quantisation for VO on FPGA](https://arxiv.org/abs/2507.07903v1) | Paper (DSD 2025) | Quantized SuperPoint on Zynq UltraScale+ DPU / FINN; 640×480 at up to 54 fps |
| [Kria KV260 ATRover](https://hackster.io/dramoz/kria-kv260-atrover-mini-0e07e1) | Hackster project | YOLOv4 + stereo distance on KV260 with PYNQ; ground robot, same stack |

## Other notes

- [PX4 Vision Kit docs](https://docs.px4.io/main/en/complete_vehicles_mc/px4_vision_kit.html) — PX4-Avoidance is archived and no longer supported.
- [Vitis Vision Library index](https://xilinx.github.io/Vitis_Libraries/vision/2021.1/index.html) — targets Zynq UltraScale+ MPSoC.
- [Ultralytics — AMD Xilinx deployment](https://docs.ultralytics.com/integrations/xilinx) — newer Vitis AI moves to NPU / Versal.
- [KR260 Robotics Starter Kit](https://www.hackster.io/news/accelerate-your-robotics-design-with-the-kria-kr260-robotics-starter-kit-89191a42080d) — same K26 SOM as the KV260, with Ubuntu 22.04 and ROS 2 Humble.
