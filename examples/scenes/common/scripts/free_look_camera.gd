class_name FreeLookCamera3D extends Camera3D

@export_range(0.0, 1000.0, 0.1, "or_greater", "exp", "suffix:u/s")
var initial_speed := 10.0

@export_range(0.0, 100.0, 0.5, "or_greater")
var interpolation_speed := 15.0

@export_range(0.01, 10.0, 0.01, "or_greater", "exp")
var mouse_sensitivity := 1.0

@export_range(1.0, 2.0, 0.01, "or_greater", "exp")
var speed_step_factor = 1.2

var speed := initial_speed

var target_position := position

func _input(event: InputEvent):
	if not current:
		return

	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			if event.pressed:
				Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
			else:
				Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			if event.pressed:
				_step_speed_up(event.factor)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			if event.pressed:
				_step_speed_down(event.factor)
	elif event is InputEventMouseMotion:
		if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
			_rotate_from_mouse(event.relative)

func _process(delta: float):
	if not current:
		return

	var step := speed * delta

	if Input.is_physical_key_pressed(KEY_SHIFT):
		step *= 4
	if Input.is_physical_key_pressed(KEY_ALT):
		step /= 4

	if Input.is_physical_key_pressed(KEY_W):
		target_position += -basis.z * step
	if Input.is_physical_key_pressed(KEY_S):
		target_position += basis.z * step
	if Input.is_physical_key_pressed(KEY_A):
		target_position += -basis.x * step
	if Input.is_physical_key_pressed(KEY_D):
		target_position += basis.x * step

	position = position.lerp(target_position, interpolation_speed * delta)

func _step_speed_up(factor: float = 1.0):
	_scale_speed(1.0 + (speed_step_factor - 1.0) * factor)

func _step_speed_down(factor: float = 1.0):
	_scale_speed(1.0 / (1.0 + (speed_step_factor - 1.0) * factor))

func _scale_speed(factor: float):
	var min_speed := near * 4
	var max_speed := far / 4

	if min_speed < max_speed:
		speed = clampf(speed * factor, min_speed, max_speed);
	else:
		speed = (min_speed + max_speed) / 2.0;

func _rotate_from_mouse(relative: Vector2):
	var delta := -relative * 0.0015 * mouse_sensitivity

	rotation = Vector3(
		clampf(rotation.x + delta.y, -PI / 2.0, PI / 2.0),
		wrapf(rotation.y + delta.x, 0.0, TAU),
		rotation.z
	)
