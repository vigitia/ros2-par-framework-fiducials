# Fiducials Detector for the ROS 2 PAR-Framework

> Readme written together with Claude Code. Comments in the Code indicate where Claude Code was used while creating this package. 

**Why a custom Fiducials detector?** 
- Projector light can and will negatively affect the detection of markers. This node can handle both RGB and Near-IR images in which the projector light becomes invisible.
- Support for depth cameras. When depth frames arrive via the provided ROS 2 interface, each detected marker gets added depth data automatically.


### Tracking Mode

Used for additional performance gains.

**When enabled**:
- **For every camera frame**: For each already known marker (from the last frames): Just check for it again in a defined area around its last known position (set via the parameter `tracking_window_size_px`) instead of in the whole frame. 
- **In a separate thread**: Search at a lower framerate (default 5 times per second), search the entire frame for new markers. 

Tested on a laptop with a mono camera at 160fps, 1920x1200 px without performance drops. 


### Depth Mode

When depth frames are available and the parameter `use_depth` is set to `True`, depth 