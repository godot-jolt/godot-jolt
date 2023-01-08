@tool

extends StaticBody3D

@export
var collision_shape := NodePath():
	get:
		return collision_shape
	set(value):
		collision_shape = value
		properties_changed()

@export
var mesh_instance := NodePath():
	get:
		return mesh_instance
	set(value):
		mesh_instance = value
		properties_changed()

@export_range(4, 64, 1, "or_greater")
var resolution: int = 16:
	get:
		return resolution
	set(value):
		resolution = value
		properties_changed()

@export_range(0.1, 10.0, 0.1, "or_greater")
var amplitude: float = 1.0:
	get:
		return amplitude
	set(value):
		amplitude = value
		properties_changed()

@export
var noise_seed: int = 0:
	get:
		return noise_seed
	set(value):
		noise_seed = value
		properties_changed()

@export
var noise_frequency: float = 0.2:
	get:
		return noise_frequency
	set(value):
		noise_frequency = value
		properties_changed()

func properties_changed():
	if Engine.is_editor_hint():
		generate()

func generate():
	var _collision_shape := get_node_or_null(collision_shape)
	if not _collision_shape:
		return

	var height_map_shape := _collision_shape.shape as HeightMapShape3D
	if height_map_shape == null:
		printerr("Collision shape must point to a HeightMapShape3D.")

	var _mesh_instance := get_node_or_null(mesh_instance)
	if not _mesh_instance:
		return

	var array_mesh := _mesh_instance.mesh as ArrayMesh
	if array_mesh == null:
		printerr("Mesh instance must point to an ArrayMesh.")

	var noise_gen := FastNoiseLite.new()
	noise_gen.seed = noise_seed
	noise_gen.frequency = noise_frequency
	noise_gen.noise_type = FastNoiseLite.TYPE_PERLIN
	noise_gen.fractal_type = FastNoiseLite.FRACTAL_NONE

	var heights := PackedFloat32Array()
	heights.resize(resolution * resolution)

	for z in range(resolution):
		for x in range(resolution):
			heights[z * resolution + x] = noise_gen.get_noise_2d(x, z)

	var min_height: float = +INF
	var max_height: float = -INF

	for height in heights:
		min_height = min(min_height, height)
		max_height = max(max_height, height)

	for z in range(resolution):
		for x in range(resolution):
			var i := z * resolution + x
			var fy := smoothstep(min_height, max_height, heights[i])
			var fx: float = 1 - (abs((x as float) / resolution - 0.5) * 2)
			var fz: float = 1 - (abs((z as float) / resolution - 0.5) * 2)
			heights[i] = fy * fx * fz * amplitude

	height_map_shape.map_width = resolution
	height_map_shape.map_depth = resolution
	height_map_shape.map_data = heights

	var plane_mesh := PlaneMesh.new()
	plane_mesh.size = Vector2i(resolution - 1, resolution - 1)
	plane_mesh.subdivide_width = resolution - 2
	plane_mesh.subdivide_depth = resolution - 2

	var mesh_arrays := plane_mesh.get_mesh_arrays()
	var plane_vertices := mesh_arrays[Mesh.ARRAY_VERTEX] as PackedVector3Array
	var plane_indices := mesh_arrays[Mesh.ARRAY_INDEX] as PackedInt32Array
	var vertex_count := plane_indices.size()

	var vertices := PackedVector3Array()
	vertices.resize(vertex_count)

	var extent := resolution / 2.0

	for n in range(vertex_count):
		var i := plane_indices[n]
		var v := plane_vertices[i]
		var fx := (v.x + extent) / resolution
		var fz := (v.z + extent) / resolution
		var ix: int = max(1, ceil(fx * resolution)) - 1
		var iz: int = max(1, ceil(fz * resolution)) - 1
		var vy := heights[iz * resolution + ix]
		vertices[n] = Vector3(v.x, vy, v.z)

	var normals := PackedVector3Array()
	normals.resize(vertex_count)

	for n in range(0, vertex_count, 3):
		var v0 := vertices[n + 0]
		var v1 := vertices[n + 1]
		var v2 := vertices[n + 2]

		var v0_to_v1 := (v1 - v0).normalized()
		var v0_to_v2 := (v2 - v0).normalized()
		var normal := v0_to_v2.cross(v0_to_v1)

		normals[n + 0] = normal
		normals[n + 1] = normal
		normals[n + 2] = normal

	array_mesh.clear_surfaces()

	mesh_arrays.clear()
	mesh_arrays.resize(Mesh.ARRAY_MAX)
	mesh_arrays[Mesh.ARRAY_VERTEX] = vertices
	mesh_arrays[Mesh.ARRAY_NORMAL] = normals
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, mesh_arrays)
