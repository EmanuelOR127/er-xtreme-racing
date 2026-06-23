@tool
class_name BasicWheelAssembly
##Provides a rectilinear suspension system for a vehicle.
extends Generic6DOFJoint3D


@export_group("Suspension")
@export var suspension_length: float = 0.2
@export var suspension_stiffness: float = 100
@export var suspension_damping: float = 0.9
@export_group("Wheel")
@export var rotor_radius: float = 0.05
@export var rotor_width: float = 0.01
@export_group("Steer by Wire")
@export var can_steer: bool
@export_range(-45, 45, 0.01, "radians_as_degrees") var target_steer: float = 0
@export var steer_force: float = 1
@export_group("Visuals")
@export var wheel: MeshInstance3D

func update_assembly() -> void:
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, -suspension_length)
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_EQUILIBRIUM_POINT, -suspension_length)
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_STIFFNESS, suspension_stiffness)
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_DAMPING, suspension_damping)
	if Engine.is_editor_hint():
		$Rotor.transform = global_transform
		$Rotor/CollisionShape3D.shape.radius = rotor_radius
		$Rotor/CollisionShape3D.shape.height = rotor_width

func _ready() -> void:
	if Engine.is_editor_hint(): return
	if wheel:
		wheel.reparent($Rotor/PhysicsDrivenWheel)
		$Rotor/PhysicsDrivenWheel.take_children()

func _physics_process(delta: float) -> void:
	update_assembly()
	if Engine.is_editor_hint(): return
	var parent:= get_parent()
	if parent is VehicleBody3D:
		target_steer = parent.steering
	if not can_steer: target_steer = 0
	$Rotor/PhysicsDrivenWheel.rotation.y = move_toward($Rotor/PhysicsDrivenWheel.rotation.y, -target_steer, delta * 4)
