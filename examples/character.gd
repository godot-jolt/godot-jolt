extends CharacterBody3D


const SPEED = 5.0
const JUMP_VELOCITY = 4.5

# Get the gravity from the project settings to be synced with RigidBody nodes.
var gravity = ProjectSettings.get_setting("physics/3d/default_gravity")

@onready
var target_basis = $Arrow.global_transform.basis

func _physics_process(delta: float):
	# Add the gravity.
	if not is_on_floor():
		velocity.y -= gravity * delta

	# Handle Jump.
	if Input.is_action_just_pressed("ui_accept") and is_on_floor():
		velocity.y = JUMP_VELOCITY

	# Get the input direction and handle the movement/deceleration.
	# As good practice, you should replace UI actions with custom gameplay actions.
	var input_dir := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	var direction := (transform.basis * Vector3(input_dir.x, 0, input_dir.y)).normalized()

	var arrow_material := $Arrow/Mesh_Shaft.material as StandardMaterial3D

	if direction:
		velocity.x = direction.x * SPEED
		velocity.z = direction.z * SPEED
		var y := up_direction
		var z := -direction
		var x := y.cross(z)
		target_basis = Basis(x, y, z)
		arrow_material.albedo_color.r = 1.0
		arrow_material.albedo_color.g = 0.0
		arrow_material.albedo_color.b = 0.0
	else:
		velocity.x = move_toward(velocity.x, 0, SPEED)
		velocity.z = move_toward(velocity.z, 0, SPEED)
		arrow_material.albedo_color.r = 0.43
		arrow_material.albedo_color.g = 0.64
		arrow_material.albedo_color.b = 0.0

	$Arrow.global_transform.basis = $Arrow.global_transform.basis.slerp(target_basis, 30.0 * delta)

	move_and_slide()
