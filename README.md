# FreePIEVRController
FreePIE input plugin for Daydream and GearVR controller

# Fork difference

This fork contains modified plugin for GearVR controller. It exposes some new public methods: Reconnect(), SetAHRSBeta(float v), SetAHRSPeriod(float v) and useMagnetometer property.

# Installation
1. Copy FreePIEVRController.dll on "C:\Program Files (x86)\FreePIE\plugins"
2. (if required) Right click FreePIEVRController.dll and open property and then unblock file

# Sample
```
c = 0
alvr.buttons[c][alvr.Id("trackpad_click")] = alvr.buttons[c][alvr.Id("trackpad_click")] or vrcontroller[c].click
alvr.buttons[c][alvr.Id("trackpad_touch")] = alvr.buttons[c][alvr.Id("trackpad_touch")] or vrcontroller[c].touch
alvr.buttons[c][alvr.Id("trigger")] = alvr.buttons[c][alvr.Id("trigger")] or vrcontroller[c].trigger
alvr.trackpad[c][0] = vrcontroller[c].trackpad[0]
alvr.trackpad[c][1] = vrcontroller[c].trackpad[1]
alvr.buttons[c][alvr.Id("system")] = vrcontroller[c].volup
alvr.buttons[c][alvr.Id("grip")] = vrcontroller[c].voldown

alvr.trigger[c] = 1.0 if alvr.buttons[c][alvr.Id("trigger")] else 0.0
  
alvr.override_controller_position = True
  
alvr.controller_position[c][0] = vrcontroller[c].position[0]
alvr.controller_position[c][1] = vrcontroller[c].position[1]
alvr.controller_position[c][2] = vrcontroller[c].position[2]

```
[VRControllersExt.py](https://github.com/dnnkeeper/FreePIEVRController/blob/master/samples/VRControllersExt.py)

VRControllersExt.py script enables both hands in ALVR. Right hand is controlled by the main GearVR controller through ALVR API. Left hand is controlled by Bluetooth GearVR controller connected directly to PC.

Press Left controller Home button to align its rotation with the right controller.
Press LeftAlt + R to reconnect BT controller if it is stuck.

You can modify script updateLeftController and updateRightController functions to remap actions.

[test.py](https://github.com/polygraphene/FreePIEVRController/blob/master/samples/test.py)

[withalvr.py](https://github.com/polygraphene/FreePIEVRController/blob/master/samples/withalvr.py)

# Known issue
- Drift in GearVR Controller
