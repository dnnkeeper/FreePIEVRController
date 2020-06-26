import math, time

#----------------------------------------------------------------------------
#	Math Utility
#----------------------------------------------------------------------------

def magnitude(v):
	return math.sqrt(sum(v[i]*v[i] for i in range(len(v))))

def mul(u, v):
	return [ u[i]*v for i in range(len(u)) ]

def add(u, v):
	return [ u[i]+v[i] for i in range(len(u)) ]

def sub(u, v):
	return [ u[i]-v[i] for i in range(len(u)) ]

def dot(u, v):
	return sum(u[i]*v[i] for i in range(len(u)))

def normalize(v):
	vmag = magnitude(v)
	return [ v[i]/vmag	for i in range(len(v)) ]

def clampMagnitude(v, max):
	if ( (v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) > max*max):
		return mul(normalize(v), max)
	return v

def max(a, b):
	if (a > b):
		return a
	else:
		return b
		
def min(a, b):
	if (a < b):
		return a
	else:
		return b

def moveTowards(a, b, maxStep):
	if (b > a):
		return a + min(b-a, maxStep)
	else:
		return a + max(b-a, -maxStep)

def lerp(a, b, t):
	return a * (1 - t) + b * t

def sign(x): return 1 if x >= 0 else -1

def clamp(v, min, max):
	if (v < min):
		v = min
	elif (v > max):
		v = max
	return v

# conjugate quaternion
def conj(q):
	return [-q[0], -q[1], -q[2], q[3]]

# multiplication of quaternion
def multiply(a, b):
	x0, y0, z0, w0 = a
	x1, y1, z1, w1 = b
	return [x1 * w0 - y1 * z0 + z1 * y0 + w1 * x0,
		x1 * z0 + y1 * w0 - z1 * x0 + w1 * y0,
		-x1 * y0 + y1 * x0 + z1 * w0 + w1 * z0,
		-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0]

# convert quaternion to euler
def quaternion2euler(q):
	yaw_pitch_roll = [0.0, 0.0, 0.0]
	# roll (x-axis rotation)
	sinr = +2.0 * (q[3] * q[0] + q[1] * q[2])
	cosr = +1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])
	yaw_pitch_roll[2] = math.atan2(sinr, cosr)

	# pitch (y-axis rotation)
	sinp = +2.0 * (q[3] * q[1] - q[2] * q[0])
	if (math.fabs(sinp) >= 1):
		yaw_pitch_roll[1] = math.copysign(math.pi / 2, sinp)
	else:
		yaw_pitch_roll[1] = math.asin(sinp)

	# yaw (z-axis rotation)
	siny = +2.0 * (q[3] * q[2] + q[0] * q[1]);
	cosy = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
	yaw_pitch_roll[0] = math.atan2(siny, cosy);

	return yaw_pitch_roll

# convert euler to quaternion
def euler2quaternion(yaw_pitch_roll):
	cy = math.cos(yaw_pitch_roll[0] * 0.5);
	sy = math.sin(yaw_pitch_roll[0] * 0.5);
	cr = math.cos(yaw_pitch_roll[2] * 0.5);
	sr = math.sin(yaw_pitch_roll[2] * 0.5);
	cp = math.cos(yaw_pitch_roll[1] * 0.5);
	sp = math.sin(yaw_pitch_roll[1] * 0.5);

	return [cy * sr * cp - sy * cr * sp,
	cy * cr * sp + sy * sr * cp,
	sy * cr * cp - cy * sr * sp,
	cy * cr * cp + sy * sr * sp]

# rotate specified vector using yaw_pitch_roll
def rotatevec(yaw_pitch_roll, vec):
	q = euler2quaternion(yaw_pitch_roll)
	return multiply(multiply(q, vec), conj(q))

def cross(one, other):
	"""
	cross product of self and other vector
	the result is a new perpendicular vector to self and other
	the length of the new vector is defined as
	|cross product| = |self| * |other| * cos(theta)
	so the angle theta between self and other is calculated as follows
	theta = asin(|cross product| / (|self| * | other|))
	if self and other are unit vectors
	|self| = |other| = 1
	this simplifies to
	|cross product| = sin(theta)
	so you can use the cross product of two vectors two
	find the angle between these two vector, possible useful for shading/lightning
	"""
	return [one[1] * other[2] - one[2] * other[1], one[2] * other[0] - one[0] * other[2], one[0] * other[1] - one[1] * other[0] ]

def QuaternionLookRotation(forward, up):
	vector	= normalize(forward)
	vector2 = normalize(cross(up, vector))
	vector3 = normalize(cross(vector, vector2))
	m00 = vector2[0]
	m01 = vector2[1]
	m02 = vector2[2]
	m10 = vector3[0]
	m11 = vector3[1]
	m12 = vector3[2]
	m20 = vector[0]
	m21 = vector[1]
	m22 = vector[2]
	
	quaternion = [0,0,0,1]
	num8 = (m00 + m11) + m22
	if (num8 > 0.0):
		 num = math.sqrt(num8 + 1.0)
		 quaternion[3] = num * 0.5;
		 num = 0.5 / num;
		 quaternion[0] = (m12 - m21) * num;
		 quaternion[1] = (m20 - m02) * num;
		 quaternion[2] = (m01 - m10) * num;
		 return quaternion
	if ((m00 >= m11) and (m00 >= m22)):
		num7 = math.sqrt(((1.0 + m00) - m11) - m22)
		num4 = 0.5 / num7
		quaternion[0] = 0.5 * num7
		quaternion[1] = (m01 + m10) * num4
		quaternion[2] = (m02 + m20) * num4
		quaternion[3] = (m12 - m21) * num4
		return quaternion;
	if (m11 > m22):
		num6 = math.sqrt(((1.0 + m11) - m00) - m22)
		num3 = 0.5 / num6
		quaternion[0] = (m10+ m01) * num3
		quaternion[1] = 0.5 * num6
		quaternion[2] = (m21 + m12) * num3
		quaternion[3] = (m20 - m02) * num3
		return quaternion 
	num5 = math.sqrt(((1.0 + m22) - m00) - m11)
	num2 = 0.5 / num5
	quaternion[0] = (m20 + m02) * num2
	quaternion[1] = (m21 + m12) * num2
	quaternion[2] = 0.5 * num5
	quaternion[3] = (m01 - m10) * num2
	return quaternion

#----------------------------------------------------------------------------
# SETTINGS
#----------------------------------------------------------------------------

# Arm pivot relative to your head
_leftArmLocalPosition = [-0.15, -0.25, 0.1, 0.0]

# Arm pivot relative to your head
_rightArmLocalPosition = [0.15, -0.25, 0.1, 0.0]

# Hand is placed at the pivot position + hand offset controled with trackpad
DefaultHandOffset = [0.0, 0.0, -0.4, 0.0]

RightEyeOffset = [0.032, 0.0, 0.0, 0]

ArmMaxLength = 0.75

TouchSensivityX = 0.25

TouchSensivityY = 0.5

StandingHeight = 1.7

CrouchHeight = 1.0

# Enable this for head position displacement for correcting ARCore data
ARCoreCorrection = False

rightId = 0

leftId = 1

global _deltaTime

if starting:
	_timestamp = time.time()
	_deltaTime = 0
	_isClick = [False, False]
	_downClick = [False, False]
	_triggerPulled = [False, False]
	_isCrouch = False
	_gripTriggerToggle = [False, False]
	_backClick = [False, False]
	_backClickDouble = [False, False]
	_inputTouchId = alvr.InputId("trackpad_touch")
	_inputTrackpadClickId = alvr.InputId("trackpad_click")
	_inputTriggerId = alvr.InputId("trigger")
	_inputBackId = alvr.InputId("back")
	_inputVolumeUpId = alvr.InputId("volume_up")
	_inputVolumeDownId = alvr.InputId("volume_down")
	_touchId = alvr.Id("trackpad_touch")
	_clickId = alvr.Id("trackpad_click")
	_systemId = alvr.Id("system")
	_menuId = alvr.Id("application_menu")
	_gripId = alvr.Id("grip")
	
	alvr.two_controllers = True
	alvr.override_head_position = True
	alvr.override_head_orientation = False
	alvr.override_controller_position = True
	alvr.override_controller_orientation = True
	
def getALVRControllerPosition():
	return [alvr.input_controller_position[0], alvr.input_controller_position[1], alvr.input_controller_position[2]]
	
def getALVRControllerRotation():
	rightRotation = [math.degrees(alvr.input_controller_orientation[0]), math.degrees(alvr.input_controller_orientation[1]), math.degrees(alvr.input_controller_orientation[2]) ]
	diagnostics.watch(rightRotation[0]), diagnostics.watch(rightRotation[1]), diagnostics.watch(rightRotation[2])
	return [alvr.input_controller_orientation[0], alvr.input_controller_orientation[1], alvr.input_controller_orientation[2]]
	
def getVRControllerPosition(id):
	pos = vrcontroller[id]
	return [pos[0], pos[1], pos[2]]
	
global _correctionLeftRotation
if (starting):
	_correctionLeftRotation = [0,0,0,1]

if (starting):
	angle_radians = [0,0,0,0]
	pitch = 0
	yaw = 0
	roll = 0
	filteredRot = [0,0,0]
def getVRControllerRotation(id):
	global pitch,yaw,roll,filteredRot
	
	global _leftControllerRotation, _leftControllerOrientation
	_leftControllerOrientation = vrcontroller[id].quaternion
	newRotation = quaternion2euler( multiply(vrcontroller[id].quaternion, _correctionLeftRotation ) )
	
	_leftControllerRotation = newRotation
	
	finalRotation = [0,0,0]
	finalRotation[0] = (finalRotation[0] + newRotation[0])
	finalRotation[1] = (finalRotation[1] + newRotation[1])
	finalRotation[2] = (finalRotation[2] + newRotation[2])
	
	leftRotation = [ math.degrees(finalRotation[0]), math.degrees(finalRotation[1]), math.degrees(finalRotation[2]) ]
	diagnostics.watch(leftRotation[0]), diagnostics.watch(leftRotation[1]), diagnostics.watch(leftRotation[2])
	return finalRotation
	
def setControllerRotation(id, rot):
	alvr.controller_orientation[id][0] = rot[0]
	alvr.controller_orientation[id][1] = rot[1]
	alvr.controller_orientation[id][2] = rot[2]
	
def setControllerPosition(id, pos):
	alvr.controller_position[id][0] = pos[0]
	alvr.controller_position[id][1] = pos[1]
	alvr.controller_position[id][2] = pos[2]

global _inputHeadPosition, _inputHeadFiltered

if (starting):
	_inputHeadPosition = alvr.input_head_position
	_inputHeadFiltered = _inputHeadPosition
	
def updateHeadPosition():
	global _inputHeadPosition, _inputHeadFiltered
	inputHeadDelta = [0,0,0]
	inputHeadDelta[0] = alvr.input_head_position[0] - _inputHeadPosition[0]
	inputHeadDelta[1] = alvr.input_head_position[1] - _inputHeadPosition[1]
	inputHeadDelta[2] = alvr.input_head_position[2] - _inputHeadPosition[2]
	if (magnitude(inputHeadDelta) > 30):
		inputHeadDelta = [0,0,0]
	
	_inputHeadFiltered[0] += inputHeadDelta[0]
	_inputHeadFiltered[1] += inputHeadDelta[1]
	_inputHeadFiltered[2] += inputHeadDelta[2]
	
	_inputHeadPosition[0] = alvr.input_head_position[0]
	_inputHeadPosition[1] = alvr.input_head_position[1]
	_inputHeadPosition[2] = alvr.input_head_position[2]
	
	headLerp = 30.0*_deltaTime
	_inputHeadPosition[0] = lerp(_inputHeadPosition[0], _inputHeadFiltered[0], headLerp)
	_inputHeadPosition[1] = lerp(_inputHeadPosition[1], _inputHeadFiltered[1], headLerp)
	_inputHeadPosition[2] = lerp(_inputHeadPosition[2], _inputHeadFiltered[2], headLerp)
	
	inputHeadRotation = [alvr.input_head_orientation[0], alvr.input_head_orientation[1], alvr.input_head_orientation[2]]

	if (ARCoreCorrection):
		#headToDisplayDistance = [0, 0.075, -0.08, 0]
		headToDisplayDistance = [ -0.10, 0.075 - 0.05, -0.15, 0]
	else:
		headToDisplayDistance = [0, 0.075 - 0.05, -0.08 + 0.06, 0]

	headPosition = sub(_inputHeadPosition, rotatevec(inputHeadRotation, headToDisplayDistance))
	
	alvr.head_position[0] = headPosition[0]
	alvr.head_position[1] = headPosition[1]
	alvr.head_position[2] = headPosition[2]
	
	return

global _isTouchedRight
global _deltaTouchRight
global _rightTouch
global _lastTouchTimeRight
global _rightHandLocalPosition

if (starting):
	_isTouchedRight = False
	_rightTouch = [0,0]
	_deltaTouchRight = [0,0]
	_lastTouchTimeRight = time.time()
	_rightHandLocalPosition = [0,0,0,0]

def updateRightController():
	global _isTouchedRight, _deltaTouchRight, _rightTouch, _lastTouchTimeRight
	
	wasTouched = _isTouchedRight
	_isTouchedRight = alvr.input_buttons[_inputTouchId]
	oldTouch = [ _rightTouch[0], _rightTouch[1] ]
	_rightTouch = [ alvr.input_trackpad[0], alvr.input_trackpad[1] ]
	_deltaTouchRight = [0,0]
	if (_isTouchedRight):
		_lastTouchTimeRight = time.time()
		if (wasTouched):
			_deltaTouchRight = [(_rightTouch[0] - oldTouch[0]), (_rightTouch[1] - oldTouch[1])]
			
	diagnostics.watch(_isTouchedRight), diagnostics.watch(_rightTouch[0]), diagnostics.watch(_rightTouch[1]), diagnostics.watch(_deltaTouchRight[0]), diagnostics.watch(_deltaTouchRight[1])
	
	isClickNow = alvr.input_buttons[_inputTrackpadClickId]
	
	# Set Trackpad XY touch
	if (_isTouchedRight):
		alvr.trackpad[rightId][0] = _rightTouch[0]
		alvr.trackpad[rightId][1] = _rightTouch[1]
	else:
		alvr.trackpad[rightId][0] = 0
		alvr.trackpad[rightId][1] = 0

	alvr.buttons[rightId][_touchId] = _isTouchedRight
	
	alvr.buttons[rightId][_clickId] = isClickNow

	alvr.trigger[rightId] = alvr.input_buttons[_inputTriggerId]
	alvr.buttons[rightId][_menuId] = alvr.input_buttons[_inputBackId]
	alvr.buttons[rightId][_systemId] = alvr.input_buttons[_inputVolumeUpId]
	alvr.buttons[rightId][_gripId] = alvr.input_buttons[_inputVolumeDownId]
	
	return
	
def updateRightHand():
	global _rightHandLocalPosition
	if (_isTouchedRight):
		_rightHandLocalPosition[0] += _deltaTouchRight[0] * TouchSensivityX 
		_rightHandLocalPosition[2] -= _deltaTouchRight[1] * TouchSensivityY
	else:
		t = 2.0 * _deltaTime
		_rightHandLocalPosition[0] = lerp(_rightHandLocalPosition[0], DefaultHandOffset[0], t)
		_rightHandLocalPosition[2] = lerp(_rightHandLocalPosition[2], DefaultHandOffset[2], t)
	
	diagnostics.watch(_rightHandLocalPosition[0]), diagnostics.watch(_rightHandLocalPosition[1]), diagnostics.watch(_rightHandLocalPosition[2])
	
	handPivotPosition = add(alvr.head_position, rotatevec(alvr.head_orientation, _rightArmLocalPosition) )
	
	handRotation = getALVRControllerRotation()
	
	handPosition = add( handPivotPosition, rotatevec(handRotation, [ _rightHandLocalPosition[0], _rightHandLocalPosition[1], _rightHandLocalPosition[2], 0] ) )
	
	setControllerPosition(rightId, handPosition)
	
	setControllerRotation(rightId, handRotation)
	
	#handPosition = add( handPosition, rotatevec(alvr.head_orientation, [ handLocalPosition[0], 0, 0, 0] ) )
	
	return
	
global _isTouchedLeft
global _deltaTouchLeft
global _leftTouch
global _lastTouchTimeLeft
global _leftHandLocalPosition

if (starting):
	_isTouchedLeft = False
	_leftTouch = [0,0]
	_deltaTouchLeft = [0,0]
	_lastTouchTimeLeft = time.time()
	_leftControllerOrientation = [0,0,0,1] 
	_leftControllerRotation = [0,0,0,0]
	_leftHandLocalPosition = [0,0,0,0]
	_leftControllerHome = False

def updateLeftController():
	global _isTouchedLeft, _deltaTouchLeft, _leftTouch, _lastTouchTimeLeft
	
	wasTouched = _isTouchedLeft
	_isTouchedLeft = vrcontroller[0].touch
	oldTouch = [ _leftTouch[0], _leftTouch[1] ]
	_leftTouch = [ vrcontroller[0].trackpad[0], vrcontroller[0].trackpad[1] ]
	# Set Trackpad XY touch
	if (_isTouchedLeft):
		alvr.trackpad[leftId][0] = _leftTouch[0]
		alvr.trackpad[leftId][1] = _leftTouch[1]
	else:
		_leftTouch[0] = 0
		_leftTouch[1] = 0
		alvr.trackpad[leftId][0] = 0
		alvr.trackpad[leftId][1] = 0
		
	diagnostics.watch(_isTouchedLeft), diagnostics.watch(_leftTouch[0]), diagnostics.watch(_leftTouch[1]), diagnostics.watch(_deltaTouchLeft[0]), diagnostics.watch(_deltaTouchLeft[1])
	
	_deltaTouchLeft = [0,0]
	if (_isTouchedLeft):
		_lastTouchTimeLeft = time.time()
		if (wasTouched):
			_deltaTouchLeft = [(_leftTouch[0] - oldTouch[0]), (_leftTouch[1] - oldTouch[1])]
	
	isClickNow = vrcontroller[0].click
	
	alvr.buttons[leftId][_touchId] = _isTouchedLeft
		
	alvr.buttons[leftId][_clickId] = isClickNow

	alvr.trigger[leftId] = vrcontroller[0].trigger
	alvr.buttons[leftId][_menuId] = alvr.input_buttons[_inputBackId]
	alvr.buttons[leftId][_systemId] = vrcontroller[0].volup
	alvr.buttons[leftId][_gripId] = vrcontroller[0].voldown
	
	global _leftControllerHome
	if (vrcontroller[0].home and not _leftControllerHome):
		global _correctionLeftRotation
		_leftControllerHome = True
		#_correctionLeftRotation = multiply( conj( euler2quaternion(_leftControllerRotation) ), euler2quaternion(getALVRControllerRotation()) )
		_correctionLeftRotation = multiply( conj( _leftControllerOrientation ), euler2quaternion(getALVRControllerRotation()) )
		
		leftRotNorm = _leftControllerOrientation[0]*_leftControllerOrientation[0] + _leftControllerOrientation[1]*_leftControllerOrientation[1] + _leftControllerOrientation[2]*_leftControllerOrientation[2] + _leftControllerOrientation[3]*_leftControllerOrientation[3]
		diagnostics.debug(leftRotNorm)
		
		global filteredRot
		filteredRot = [0,0,0]
		diagnostics.debug("left controller correction")#, diagnostics.debug(_correctionQuaternionLeft[0]), diagnostics.debug(_correctionQuaternionLeft[1]), diagnostics.debug(_correctionQuaternionLeft[2]), diagnostics.debug(_correctionQuaternionLeft[3])
	else:
		_leftControllerHome = False
	return

def normalizeAngle(angle):
	angle =  angle % math.pi;
	# force into the minimum absolute value residue class, so that -math.pi < angle <= math.pi  
	if (angle > math.pi):
		angle -= math.pi
	return angle

def updateLeftHand():
	global _leftHandLocalPosition
	if (_isTouchedLeft):
		_leftHandLocalPosition[0] += _deltaTouchLeft[0] * TouchSensivityX 
		_leftHandLocalPosition[2] -= _deltaTouchLeft[1] * TouchSensivityY
	else:
		t = 2.0 * _deltaTime
		_leftHandLocalPosition[0] = lerp(_leftHandLocalPosition[0], DefaultHandOffset[0], t)
		_leftHandLocalPosition[2] = lerp(_leftHandLocalPosition[2], DefaultHandOffset[2], t)
	
	diagnostics.watch(_leftHandLocalPosition[0]), diagnostics.watch(_leftHandLocalPosition[1]), diagnostics.watch(_leftHandLocalPosition[2])
	
	handPivotPosition = add(alvr.head_position, rotatevec(alvr.head_orientation, _leftArmLocalPosition) )
	
	handRotation = getVRControllerRotation(0)
	
	handPosition = add( handPivotPosition, rotatevec(handRotation, [ _leftHandLocalPosition[0], _leftHandLocalPosition[1], _leftHandLocalPosition[2], 0] ) )
	
	setControllerPosition(leftId, handPosition)
	
	setControllerRotation(leftId, handRotation)
	
	return

#----------------------------------------------------------------------------
#	Main Program
#----------------------------------------------------------------------------
_deltaTime = time.time() - _timestamp
execTimestamp = time.time()
diagnostics.watch(_deltaTime)

if (starting):
	if (vrcontroller is not None):
		diagnostics.debug(vrcontroller)
		diagnostics.debug("controller found: "+str(vrcontroller[0]) )
		if (vrcontroller[0] is not None):
			vrcontroller[0].useMagnetometer = False
	else:
		diagnostics.debug("No BT controllers found")

if (keyboard.getPressed(Key.R) and keyboard.getKeyDown(Key.LeftAlt)):
	diagnostics.debug("Reconnect")
	vrcontroller[0].Reconnect()

if (keyboard.getPressed(Key.P) and keyboard.getKeyDown(Key.LeftAlt)):
	vrcontroller[0].SetAHRSPeriod(1.0/180.0)
	diagnostics.debug("Set AHRS Period")

if (keyboard.getPressed(Key.Space) and keyboard.getKeyDown(Key.LeftAlt)):
	vrcontroller[0].SetAHRSBeta(0.02)
	diagnostics.debug("Set AHRS Beta")

if (keyboard.getPressed(Key.M) and keyboard.getKeyDown(Key.LeftAlt)):
	vrcontroller[0].useMagnetometer = not vrcontroller[0].useMagnetometer
	diagnostics.debug(vrcontroller[0].useMagnetometer)

updateHeadPosition()
updateRightController()
updateRightHand()

if (vrcontroller[0] is not None):
	updateLeftController()
	updateLeftHand()

executeTime = time.time() - execTimestamp
_timestamp = time.time()
diagnostics.watch(executeTime)