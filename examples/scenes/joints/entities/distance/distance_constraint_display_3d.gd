@tool
extends Node
class_name DistanceConstraintDisplay3D

@export var distance_constraint: JoltDistanceConstraint3D
@export var rope_radius: float = 0.05

var material: StandardMaterial3D = preload("res://scenes/common/materials/grey.tres")

var path: Path3D
var csg_polygon: CSGPolygon3D

func _ready() -> void:
	path = Path3D.new()
	path.curve = Curve3D.new()
	add_child(path)
	
	csg_polygon = CSGPolygon3D.new()
	add_child(csg_polygon)
	csg_polygon.mode = CSGPolygon3D.MODE_PATH
	csg_polygon.material = material
	csg_polygon.path_node = csg_polygon.get_path_to(path)
	csg_polygon.polygon = PackedVector2Array([
		Vector2(-rope_radius, rope_radius),
		Vector2(rope_radius, rope_radius),
		Vector2(rope_radius, -rope_radius),
		Vector2(-rope_radius, -rope_radius)
	])
	csg_polygon.smooth_faces = true

func _process(_delta: float) -> void:
	var body_a: Node3D = distance_constraint.get_node(distance_constraint.node_a)
	var global_point_a: Vector3 = body_a.to_global(distance_constraint.point_a)
	var body_b: Node3D = distance_constraint.get_node(distance_constraint.node_b)
	var global_point_b: Vector3 = body_b.to_global(distance_constraint.point_b)
	
	path.curve.clear_points()
	path.curve.add_point(global_point_a)
	path.curve.add_point(global_point_b)
