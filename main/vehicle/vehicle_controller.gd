class_name VehicleController
extends Node

@export var vehicle: VehicleBody3D
@export var target_engine_force: Curve
@export var target_brake_force: float = 100
@export var accel_max_angular: float
@export var accel_torque: float

func _process(_delta: float) -> void:
	if not vehicle: return
	vehicle.steering = Input.get_axis("steer_left", "steer_right") * (PI/4)
	vehicle.engine_force = Input.get_action_strength("throttle") * target_engine_force.sample(vehicle.linear_velocity.length())
	vehicle.brake = Input.get_action_strength("brake") * target_brake_force

func _physics_process(delta: float) -> void:
	if not vehicle: return
	var local_angular: Vector3 = vehicle.angular_velocity * vehicle.global_basis.inverse()
	var input: float= Input.get_axis("throttle", "brake")
	if input > 0 and local_angular.x < accel_max_angular:
		vehicle.apply_torque(vehicle.global_basis.x * accel_torque * input)
	if input < 0 and local_angular.x > -accel_max_angular:
		vehicle.apply_torque(vehicle.global_basis.x * accel_torque * input)
