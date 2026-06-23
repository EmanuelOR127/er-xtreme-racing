@tool
## [color=ffaa00]Implemented by ER Vehicle Physics.[/color]
## Simulates a physics driven vehicle wheel.
## Unlike [code]VehicleWheel[/code] this node can be attached to any [code]RigidBody3D[/code]
## making it work with more setups, and it also simulates a flexible outer layer that's driven
## by air pressure as well as being able to do burnouts, as RPM are not defined by the parent body.
class_name PhysicsDrivenWheel
extends Node3D

var body: RigidBody3D
##A parent body for the body that this wheel uses.
##Forces applied to the suspension that are perpendicular to the up direction of the wheel
##will get passed onto this body. Useful for a composite suspension mechanism.
@export var super_body: RigidBody3D 
##If true, grip applies along the direction of travel of the wheel.
@export var invert_grip: bool
##If true and [code]body[/code] is a [code]VehicleBody3D[/code], this wheel will steer with the 
## [code]steer[/code] property.
@export var use_as_steering: bool
##If true and [code]body[/code] is a [code]VehicleBody3D[/code], this wheel will accelerate with the 
## [code]engine_force[/code] and [code]brake[/code] properties
@export var use_as_traction: bool = true
##If true and [code]body[/code] is a [code]VehicleBody3D[/code], this wheel will brake with the 
## [code]engine_force[/code] and [code]brake[/code] properties
@export var use_as_brake: bool = true
##The tire mass in kilograms. To modify this property update the density values of the materials
##in the Tire and Rim sections.
@export_range(0, 99999, 0.001, "suffix:Kg") var wheel_mass: float
@export_group("Tire")
##The tire's radius in meters. This value is guaranteed to be higher than the rim's radius.
@export_range(0.05, 5, 0.001, "suffix:m") var tire_radius: float = 0.24:
	set(value):
		tire_radius = maxf(value, rim_radius+0.02)
@export_range(0, 60, 0.001, "suffix:psi") var tire_pressure: float = 25 ##The tire's pressure in PSI. Defines how bouncy or stiff the outer layer is.
@export_range(0.8, 3, 0.001) var pressure_damping: float = 1.15 ##The tire's damping. Defines how fast does the tire lose energy when contracting.
@export_range(0.01, 0.2, 0.001, "suffix:m") var tire_thickness: float = 0.01 ##The tire's virtual thickness in meters, used to calculate the wheel's mass.
@export_range(0, 22590, 1,"suffix:Kg/m^3") var tire_material_density: float = 920 ##The tire's material density in Kilograms per meter cubed, used to calculate the wheel's mass.
@export_range(0, 200, 0.001) var dry_grip: float = 2 ##How much grip does this tire have in dry conditions.
@export_range(0, 200, 0.001) var wet_grip: float = 0.5 ##How much grip does this tire have in wet conditions. (Unimplemented for now)
@export_range(0, 8, 0.001) var side_grip: float = 4 ##How much grip do tires have sideways. Multiplies the dry grip.
@export_range(0, 1, 0.001) var grip_loss_multiplier: float = 0.1 ##How much of the original grip remains during a grip loss.
@export_range(0, 1, 0.001) var side_grip_loss_multiplier: float = 0.02 ##How much of the original side grip remains during a grip loss.
@export_range(0, 200, 0.001) var grip_loss_friction: float = 4 ##At how much friction does the tire lose grip with the ground
@export_range(0, 200, 0.001) var grip_recovery_friction: float = 1 ##At how much friction does the tire recover grip with the ground
@export_range(0, 80, 0.001) var grip_change_rate: float = 2 ##Slowly moves the grip multiplier from one value to another at this rate, rather than changing instantly. Gives a more natural result when recovering grip.
var losing_grip: bool
var forward_grip_multiplier: float = 1
var side_grip_multiplier: float = 1

@export_group("Rim")
##The rim's radius in meters. This value is guaranteed to be lower than the tire's radius.
@export_range(0.02, 4.95, 0.001, "suffix:m") var rim_radius: float = 0.16:
	set(value):
		rim_radius = minf(value, tire_radius-0.02)
@export_range(0.01, 1, 0.001, "suffix:m") var rim_width: float = 0.1 ##The rim's width. Defines the width of the tire aswell.
@export_range(0, 1, 0.001) var rim_bounciness: float = 0 ##How much energy is restituted per collision with the rim.
@export_range(0.01, 0.2, 0.001, "suffix:m") var rim_thickness: float = 0.005 ##The rim's virtual thickness in meters, used to calculate the wheel's mass.
@export_range(0, 22590, 1,"suffix:Kg/m^3") var rim_material_density: float = 7810 ##The rim's material density in Kilograms per meter cubed, used to calculate the wheel's mass.
@export_range(0, 4, 0.001) var bearing_damp: float = 0.1 ##How much does the wheel slow over time while freely spinning.
@export var max_surface_speed: float = 100

@export_group("Suspension")
##From the origin point of this node, how far down does the suspension expand.
## Together with [code]spring_stiffness[/code] this defines the resting point of the wheel.
@export_range(0, 5, 0.001, "suffix:m") var suspension_travel: float = 0.2
@export_range(0, 250, 0.001, "suffix:N/mm") var spring_stiffness: float = 70 ##Defines how high the wheel rests on the suspension. Lower values make the suspension rest higher up.
@export_range(0, 2, 0.001, "suffix:times") var wheel_force_multiplier: float = 0.125 ##Reduces the amount of force the suspension can exert over the wheel to improve the simulation's stability.
@export_range(0, 40000, 0.001, "suffix:N") var suspension_max_load: float = 5000 ##Caps the max force the suspension can exert on the body.
@export_range(0, 250, 0.001) var spring_damping: float = 45 ##Damps the wheel movement on the suspension. For simulation stability keep this value close to [code]spring_stiffness[/code].
@export_range(0, 25000, 0.001) var suspension_damping: float = 4500 ##Defines how hard or soft the suspension is for the body.

var origin: Vector3
##Along the path of travel, where is the wheel currently.
## The suspension travels from the node origin downwards until [code]suspension_travel[/code].
var suspension_position: float
##How fast is the wheel currenlty moving along the suspension in meters per second.
var suspension_velocity: float

var braking_force: float ##How much does the wheel slow down per second.
var throttle_force: float ##How much torque does the wheel recieve per second.
## Tire's surface velocity in m/s. Do NOT modify this value directly, instead
## apply torque using the [code]apply_torque[/code] method.
var surface_speed: float
var _previous_position: Vector3
var _pivot_previous_position: Vector3

var _raycast: RayCast3D
var _shapecast: ShapeCast3D
var _shape: CylinderShape3D

func _ready() -> void:
	_setup_tire()
	if Engine.is_editor_hint() or get_tree().debug_collisions_hint: _create_gizmo()

#region Gizmo
var _gizmo_rim: CylinderMesh
var _gizmo_tire: CylinderMesh
var _rim_instance: MeshInstance3D
var _tire_instance: MeshInstance3D
static var _gizmo_material: Material = load("res://main/vehicle/physics/gizmo.tres")
static var _grip_loss_material: Material = load("res://main/vehicle/physics/grip_loss.tres")

##Private function that creates the tire gizmo. Do NOT call this function manually as
## it is automatically called on [code]_ready[/code] if in the editor or displaying
## collision shapes, and calling it again will NOT free the previous nodes.
func _create_gizmo() -> void:
	assert(not _gizmo_rim and not _gizmo_tire, "_create_gizmo must be called only once.")
	_gizmo_rim = CylinderMesh.new()
	_gizmo_tire = CylinderMesh.new()
	_gizmo_rim.material = _gizmo_material
	_gizmo_tire.material = _gizmo_material
	_rim_instance = MeshInstance3D.new()
	_tire_instance = MeshInstance3D.new()
	_rim_instance.mesh = _gizmo_rim
	_tire_instance.mesh = _gizmo_tire
	_rim_instance.rotation.z = PI/2
	_tire_instance.rotation.z = PI/2
	add_child(_rim_instance)
	add_child(_tire_instance)

func _process(_delta: float) -> void:
	if Engine.is_editor_hint() or get_tree().debug_collisions_hint: _update_gizmo()

##Private function that updates the tire gizmo. Automatically called in the editor
## and when displaying collision shapes.
func _update_gizmo() -> void:
	if not _gizmo_rim: return
	if not _gizmo_tire: return
	_gizmo_rim.height = rim_width
	_gizmo_rim.bottom_radius = rim_radius
	_gizmo_rim.top_radius = rim_radius
	_gizmo_tire.height = rim_width
	_gizmo_tire.bottom_radius = tire_radius
	_gizmo_tire.top_radius = tire_radius
	_gizmo_tire.material = _gizmo_material if not losing_grip else _grip_loss_material
#endregion

##Private function that setups the tire. Do NOT call this function manually as
## it is automatically called on [code]_ready[/code] and calling it again will
## NOT free the previous nodes.
func _setup_tire() -> void:
	origin = position
	assert(not _shapecast, "_setup_tire must be called only once.")
	var children:= get_children()
	_raycast = RayCast3D.new()
	_shapecast = ShapeCast3D.new()
	_shapecast.max_results = 4
	_shapecast.rotation.z = PI/2
	_shape = CylinderShape3D.new()
	_shapecast.shape = _shape
	_shapecast.target_position = Vector3.ZERO
	add_child(_shapecast)
	add_child(_raycast)
	if not Engine.is_editor_hint(): return
	for c: Node in children:
		c.reparent(_shapecast)

func take_children() -> void:
	var children:= get_children()
	for c: Node in children:
		if c is MeshInstance3D:
			c.reparent(_shapecast)
			c.position = Vector3.ZERO

func _physics_process(delta: float) -> void:
	_shape.height = rim_width
	_shape.radius = tire_radius
	_raycast.target_position = Vector3(0, -(suspension_travel+tire_radius), 0)
	wheel_mass = calculate_mass()
	if get_parent() is RigidBody3D: body = get_parent()
	else: body = null
	if Engine.is_editor_hint(): return
	if not body: return
	if body is VehicleBody3D:
		if use_as_steering: rotation.y = -body.steering
		else: rotation.y = 0
		if use_as_traction: throttle_force = body.engine_force
		if use_as_brake: braking_force = body.brake
	if super_body and super_body is VehicleBody3D:
		if use_as_traction: throttle_force = super_body.engine_force
		if use_as_brake: braking_force = super_body.brake
	assert(is_instance_valid(_shapecast), "Wheel simulation cannot be performed without _shapecast.")
	if _shapecast.is_colliding():
		var collision_count: int = 0
		for i: int in _shapecast.get_collision_count():
			if _shapecast.get_collider(i) == body: continue
			if super_body and _shapecast.get_collider(i) == super_body: continue
			collision_count += 1
		
		for i: int in _shapecast.get_collision_count():
			if _shapecast.get_collider(i) == body: continue
			if super_body and _shapecast.get_collider(i) == super_body: continue
			var point:= _shapecast.get_collision_point(i)
			var velocity:= calculate_velocity(delta)
			point += velocity * delta
			var local_point: Vector3 = point * global_transform
			local_point.x = 0 #Center the local point along the wheel's X plane
			var point_dir: Vector3 = (local_point * global_basis.inverse()).direction_to(Vector3.ZERO) #Where is the point applying pressure on the tire
			var point_radius: float = local_point.length() #The distance of the point relative to the wheel's center
			var distance_to_edge: float = tire_radius - point_radius #The distance of the point to the tire's edge
			
			var surface_velocity: Vector3 = calculate_surface_velocity(local_point, delta)
			
			#Rim Force. This part sucks so i don't use it for now.
			#if point_radius < rim_radius:
			#	var collision_normal:= _shapecast.get_collision_normal(i)
			#	var dot:= -collision_normal.dot(velocity)
			#	var slide_velocity:= velocity - (collision_normal*dot)
			#	var requiered_impulse:= velocity - slide_velocity #Velocity requiered to slide along the surface
			#	var bounce = velocity - velocity.bounce(collision_normal)
			#	body.apply_impulse((requiered_impulse + (bounce * rim_bounciness)) * body.mass / 4, global_position - body.global_position)
			
			#Pressure force
			point_dir /= collision_count
			#1 PSI = 6894.75729 Pascal
			var along_axis_movement: float = surface_velocity.dot(point_dir)
			apply_central_force(point_dir * distance_to_edge * (tire_pressure * 6894.75729))
			apply_central_force(-point_dir * along_axis_movement * pressure_damping * 6894.75729)
			
			#Friction
			var mass: float = body.mass
			if super_body: mass = super_body.mass
			
			var sideways_friction: float = surface_velocity.dot(global_basis.x)
			var forward_friction: float = surface_velocity.dot(global_basis.z)
			
			if losing_grip:
				if max(abs(forward_friction), abs(sideways_friction)) < grip_recovery_friction: losing_grip = false
			else:
				if max(abs(forward_friction), abs(sideways_friction)) > grip_loss_friction: losing_grip = true
			
			sideways_friction *= side_grip_multiplier
			forward_friction *= forward_grip_multiplier
			sideways_friction /= collision_count
			forward_friction /= collision_count
			apply_force(-global_basis.x * sideways_friction * mass, local_point)
			apply_force(-global_basis.z * forward_friction * mass, local_point)
			apply_torque(forward_friction * mass, point_radius)
	if losing_grip or not _shapecast.is_colliding():
		side_grip_multiplier = move_toward(side_grip_multiplier, side_grip * dry_grip * side_grip_loss_multiplier, delta * grip_change_rate)
		forward_grip_multiplier = move_toward(forward_grip_multiplier, dry_grip * grip_loss_multiplier, delta * grip_change_rate)
	else:
		side_grip_multiplier = move_toward(side_grip_multiplier, side_grip * dry_grip, delta * grip_change_rate)
		forward_grip_multiplier = move_toward(forward_grip_multiplier, dry_grip, delta * grip_change_rate)
	
	apply_torque(throttle_force, tire_radius)
	var braking_torque: float = calculate_torque(braking_force, tire_radius)
	braking_torque = clampf(braking_torque, 0, abs(surface_speed))
	if surface_speed > 0: surface_speed -= braking_torque
	else: surface_speed += braking_torque
	
	surface_speed -= surface_speed * bearing_damp * delta
	suspension_physics(delta)
	_shapecast.rotation.x += (surface_speed / (tire_radius * 2 * TAU)) * delta
	_previous_position = global_position

##Calculates the node's velocity relative to the world.
##Used to calculate the surface velocity.
func calculate_velocity(delta: float) -> Vector3:
	return (global_position - _previous_position)/delta

##Calculates the node's suspension pivot velocity relative to the world.
##Used to calculate the suspension load.
func calculate_pivot_velocity(delta: float) -> Vector3:
	return (_raycast.global_position - _pivot_previous_position)/delta

##Calculates the surface velocity of the wheel relative to the world for any given local point on it.
func calculate_surface_velocity(point: Vector3, delta: float) -> Vector3:
	var perpendicular: Vector3 = point.rotated(Vector3.RIGHT, PI/2)
	var surface_local_velocity: Vector3 = perpendicular * surface_speed
	var surface_velocity: Vector3
	surface_velocity = surface_local_velocity * global_basis.inverse()
	surface_velocity += calculate_velocity(delta)
	return surface_velocity

##Returns the total mass estimate using the wheel's tire and rim density and volumes.
func calculate_mass() -> float:
	var tire_inner_volume:= PI*pow(tire_radius-tire_thickness, 2)*(rim_width-tire_thickness)
	var tire_outer_volume:= PI*pow(tire_radius, 2)*rim_width
	var tire_voulme:= tire_outer_volume-tire_inner_volume
	var tire_mass_result:= tire_voulme * tire_material_density
	
	var rim_inner_volume:= PI*pow(rim_radius-rim_thickness, 2)*rim_width
	var rim_outer_volume:= PI*pow(rim_radius, 2)*rim_width
	var rim_voulme:= rim_outer_volume-rim_inner_volume
	var rim_mass: float = rim_voulme * rim_material_density
	
	return tire_mass_result + rim_mass

##Applies leverage onto the wheel in newtons at the specified distance. Lower distances
## give higher speeds but with lower acceleration.
func apply_torque(force: float, distance: float) -> void:
	if force == 0: return
	var torque: float = force * distance
	var acceleration: float = torque/wheel_mass * get_physics_process_delta_time()
	surface_speed += acceleration
	surface_speed = clampf(surface_speed, -max_surface_speed, max_surface_speed)

func calculate_torque(force: float, distance: float) -> float:
	var torque: float = force * distance
	var acceleration: float = torque/wheel_mass * get_physics_process_delta_time()
	return acceleration


##Applies a force on the wheel in newtons to act against the suspension.
##The force is always centered on the wheel's pivot.
func apply_central_force(force: Vector3) -> void:
	var delta: float = get_physics_process_delta_time()
	if suspension_travel > 0:
		var suspension_dot: float = global_basis.y.dot(force)
		var suspension_force: Vector3 = global_basis.y * suspension_dot
		var perpendicular_force: Vector3 = force - suspension_force
		var increment: float = (suspension_dot/wheel_mass) * delta
		increment *= wheel_force_multiplier
		suspension_velocity += increment# clampf(increment, -relaxation, compression)
		body.apply_force(perpendicular_force * 60 * delta, global_position - body.global_position)
		if suspension_position <= 0:
			suspension_velocity = min(0, suspension_velocity)
			body.apply_force(suspension_force * delta, global_position - body.global_position)
	else:
		if super_body:
			var parallel_force:= global_basis.y * global_basis.y.dot(force)
			var perpendicular_force:= force-parallel_force
			body.apply_force(parallel_force * delta, global_position - body.global_position)
			super_body.apply_force(perpendicular_force * delta, global_position - super_body.global_position)
		else:
			body.apply_force(force * delta, global_position - body.global_position)

func apply_force(force: Vector3, offset: Vector3) -> void:
	var delta: float = get_physics_process_delta_time()
	if super_body:
		var parallel_force:= global_basis.y * global_basis.y.dot(force)
		var perpendicular_force:= force-parallel_force
		body.apply_force(parallel_force * delta, (global_position + offset) - body.global_position)
		super_body.apply_force(perpendicular_force * delta, (global_position + offset) - super_body.global_position)
	else:
		body.apply_force(force * delta, (global_position + offset) - body.global_position)

func suspension_physics(delta: float) -> void:
	var suspension_push: float = (suspension_travel-suspension_position) * (spring_stiffness*1000)
	var suspension_pushback: Vector3 = global_basis.y * clampf(suspension_push, 0, suspension_max_load)
	#Push the vehicle against the suspension limit under extreme cases.
	
	suspension_position -= suspension_velocity * delta
	suspension_position = clampf(suspension_position, -0.01, suspension_travel)
	position = origin + Vector3.DOWN * suspension_position
	_raycast.position.y = suspension_position
	
	apply_central_force(body.get_gravity())
	apply_central_force(-global_basis.y * suspension_push)
	
	if suspension_velocity > 0: suspension_velocity -= clampf(suspension_velocity * spring_damping * delta, 0, suspension_velocity)
	else: suspension_velocity -= clampf(suspension_velocity * spring_damping * delta, suspension_velocity, 0)
	
	if suspension_travel <= 0: return
	
	var suspension_dampforce: Vector3 = global_basis.y * clampf(get_suspension_load(delta) * suspension_damping, 0 , suspension_max_load)
	if _raycast.is_colliding():
		body.apply_force(suspension_pushback, global_position - body.global_position)
		body.apply_force(suspension_dampforce, global_position - body.global_position)
	_pivot_previous_position = _raycast.global_position

func get_suspension_load(delta: float) -> float:
	return -global_basis.y.dot(calculate_pivot_velocity(delta))

##Returns the wheel spin in revolutions per minute. This is calculated using
## the wheel's surface velocity and radius.
func get_rpm() -> float:
	if tire_radius == 0: return 0
	return (surface_speed / (tire_radius * 2 * TAU)) * 60

func _get_configuration_warnings() -> PackedStringArray:
	var warnings: PackedStringArray
	if get_parent() is not RigidBody3D:
		warnings.append("The parent of this node must inherit RigidBody3D to function properly.")
	return warnings

func _validate_property(property: Dictionary) -> void:
	if property.name == "wheel_mass": property.usage = PROPERTY_USAGE_EDITOR
	if get_parent() is VehicleBody3D or super_body and super_body is VehicleBody3D:
		if property.name == "use_as_steering": property.usage = PROPERTY_USAGE_DEFAULT
		if property.name == "use_as_traction": property.usage = PROPERTY_USAGE_DEFAULT
		if property.name == "use_as_brake": property.usage = PROPERTY_USAGE_DEFAULT
	else:
		if property.name == "use_as_steering": property.usage = PROPERTY_USAGE_NONE
		if property.name == "use_as_traction": property.usage = PROPERTY_USAGE_NONE
		if property.name == "use_as_brake": property.usage = PROPERTY_USAGE_NONE
