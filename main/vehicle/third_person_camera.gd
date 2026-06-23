extends Camera3D

@export var distance: float = 5
@export var height: float = 1
@export var position_smoothing: Curve
@export var position_smoothing_speed: float = 8
@export var rotation_smoothing: float = 8

func _physics_process(delta: float) -> void:
	var parent: Node = get_parent()
	if parent is not Node3D: return
	var use_side_view:= false
	$RayCast3D.position = position
	var target_dir: Vector3 = parent.global_basis.z
	var target_position: Vector3 = parent.global_position - (parent.global_basis.z * distance) + (parent.global_basis.y * height)
	if $RayCast3D.is_colliding():
		var ground_normal: Vector3 = $RayCast3D.get_collision_normal()
		target_dir -= target_dir.dot(ground_normal) * ground_normal
		target_position.y = $RayCast3D.get_collision_point().y + height
	var speed: float = position_smoothing.sample(position.distance_to(target_position)) * delta
	position = position.move_toward(target_position, speed * position_smoothing_speed)
	rotation.y = lerp_angle(rotation.y, atan2(target_dir.x, target_dir.z) + PI, delta * rotation_smoothing)
	rotation.x = lerp_angle(rotation.x, atan2(target_dir.y, 1), delta * rotation_smoothing)
	
