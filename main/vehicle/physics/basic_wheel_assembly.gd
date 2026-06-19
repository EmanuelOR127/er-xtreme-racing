@tool
class_name BasicWheelAssembly
##Provides a rectilinear suspension system for a vehicle.
extends Generic6DOFJoint3D


@export_group("Suspension")
@export var flip: bool ##If true, the node positions get mirrored to accomodate for a different orientation.
@export var suspension_length: float = 0.2
@export var suspension_stiffness: float = 100
@export var suspension_damping: float = 0.9
@export_group("Wheel")
@export var wheel_radius: float = 0.15
@export var wheel_width: float = 0.1
@export var rotor_radius: float = 0.05
@export var rotor_width: float = 0.01
@export var rotor_tolerance: float = 0.01 ##The distance from the wheel to the rotor.
@export_group("Steer by Wire")
@export var can_steer: bool
@export_range(-45, 45, 0.01, "radians_as_degrees") var target_steer: float = 0
@export var steer_speed: float = 10
@export var steer_force: float = 3000
@export_group("Visuals")
@export var wheel: Mesh
@export var rotor: Mesh

func update_assembly() -> void:
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, -suspension_length)
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_EQUILIBRIUM_POINT, -suspension_length)
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_STIFFNESS, suspension_stiffness)
	set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_SPRING_DAMPING, suspension_damping)
	set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_FORCE_LIMIT, steer_force)
	set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, not can_steer)
	$Wheel/CollisionShape3D.shape.radius = wheel_radius
	$Wheel/CollisionShape3D.shape.height = wheel_width
	$Rotor/CollisionShape3D.shape.radius = rotor_radius
	$Rotor/CollisionShape3D.shape.height = rotor_width
	$Wheel.position = global_position
	$Rotor.position = global_position
	if flip: $Wheel.position.x -= (rotor_width+wheel_width)/2 + rotor_tolerance
	else: $Wheel.position.x += (rotor_width+wheel_width)/2 + rotor_tolerance

##Unlike in regular steering mechanisms, here the whole suspension mechanism rotates.
func steer_by_wire() -> void:
	var target_basis:= global_basis.rotated(global_basis.y, target_steer)
	var difference: float = $Rotor.global_basis.z.dot(target_basis.x)
	set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_MOTOR_TARGET_VELOCITY, difference * steer_speed)

func _process(_delta: float) -> void:
	if Engine.is_editor_hint():
		update_assembly()

func _physics_process(_delta: float) -> void:
	if Engine.is_editor_hint(): return
	if can_steer: steer_by_wire()
