extends Node

func _input(event: InputEvent):
	if event is InputEventKey:
		if event.keycode == KEY_ESCAPE:
			get_tree().quit()
