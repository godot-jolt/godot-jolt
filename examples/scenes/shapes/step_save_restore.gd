extends Node3D
# This script is used to record and replay physics states

# The number of states to save
const NUMBER_OF_STATES_TO_SAVE: int = 250

# The amount of time between each physics step
var PHYSICS_STEP_DELTA: float = 1.0 / Engine.physics_ticks_per_second

# Holds an array of states to replay
var state_array: Array[PackedByteArray]

# Keeps track of how many states have been saved
var states_saved: int = 0

# Keeps track of the current state to restore
var state_index: int = 0

# Whether or not we are recording state
var is_recording: bool = true

# Called when the node enters the scene tree for the first time.
func _ready():
	state_array.resize(NUMBER_OF_STATES_TO_SAVE)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	pass


func _physics_process(_delta):
	# When the scene starts, we want to record the initial states
	if is_recording:
		# Save the state
		state_array[state_index] = JoltPhysicsServer3D.save_state()
		state_index += 1
		states_saved += 1

		# If we have saved enough states, stop recording
		if states_saved >= NUMBER_OF_STATES_TO_SAVE:
			is_recording = false

		print("Saved state " + str(states_saved) + " of " + str(NUMBER_OF_STATES_TO_SAVE))
	else:
		# If we are not recording, we want to restore states

		# If the user presses the rewind (left arrow) button, we want to restore the state
		if Input.is_action_pressed("rewind"):
			print("Restoring state [rewind]" + str(state_index))
			state_index -= 1
			if state_index < 0:
				state_index = states_saved - 1
			
			# Set the physics server to active so that the state can be restored
			JoltPhysicsServer3D.set_active(true)
			JoltPhysicsServer3D.restore_state(state_array[state_index])

		# If the user presses the forward (right arrow) button, we want to restore the state
		elif Input.is_action_pressed("forward"):
			print("Restoring state [forward]" + str(state_index))
			state_index += 1
			if state_index >= states_saved:
				state_index = 0
			
			# Set the physics server to active so that the state can be restored
			JoltPhysicsServer3D.set_active(true)
			JoltPhysicsServer3D.restore_state(state_array[state_index])

		# If the user presses the step (down arrow) button, we want to step the simulation forward
		elif Input.is_action_just_pressed("step"):
			print("Stepping state by delta: " + str(PHYSICS_STEP_DELTA))
			
			JoltPhysicsServer3D.set_active(true)
			JoltPhysicsServer3D.step(PHYSICS_STEP_DELTA)

		else:
			# If the user is not pressing any buttons, we want to pause the simulation
			JoltPhysicsServer3D.set_active(false)
			
 