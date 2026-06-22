class_name VehicleController
extends Node

@export var vehicle: VehicleBody3D
@export var target_engine_force: float = 100

func _process(_delta: float) -> void:
	if not vehicle: return
	vehicle.steering = Input.get_axis("steer_left", "steer_right") * (PI/4)
	vehicle.engine_force = Input.get_action_strength("throttle") * target_engine_force
	vehicle.brake = Input.get_action_strength("brake")
