@tool
class_name MinimalFabrik3D extends SkeletonModifier3D
## Minimalistic 3D FABRIK re-implementation in GDScript.
## This script is not meant to be a plug-and-play solution. It was created as a learning project and can serve as a minimal base for getting started.
## It's also not a plugin, it doesn't create any UI beyond exported fields in the inspector, so can be moved around the project freely.
## There are better/faster solutions using GDNative out there, this one can be too slow for large-scale applications (1000+ instances).
## Takes skeleton scale into account. No angle constraints as of now.
##
## Designed to be inherited for specific types of IK implementations (arms, legs, tentacles, tails and what not),
## where actual animation deforms can be snapshotted and stored before the IK solve pass.
## Data allocation phase is containd in `init_ik()` method, and needs to be run as soon as the node is ready. 
## The entire IK pass is contained in the `resolve_ik()` method, which needs to be run in the `_process_modification()` hook.
## All transforms/origins, such as target or pole, are stored as values directly to minimize external node reliance.
##
## Based on https://github.com/ditzel/SimpleIK with tweaks and fixes.
## Operates on matrices instead of quaternions to ensure better bone orientation.

## Bone from which the IK will start and follow.
@export var root_bone: int = -1
## Bone on which the IK will end.
@export var tip_bone: int = -1
## Target position of tip bone. In global coordinates space. Should be set from scripts dynamically.
@export var target: Transform3D = Transform3D.IDENTITY
## Pole position designating in which direction knee or elbow should bend. In global coordinates space.
## Should be set from scripts dynamically.
@export var pole: Vector3 = Vector3.ZERO
## Maximum number of forward and backward reaching iterations per frame. 
## Actual iterations rarely reach 10 every time for 2-3 bone chain.
@export var max_iterations: int = 10
## Minimum allowed error to consider the tip bone being at its target destination.
@export var distance_error_margin: float = 0.01
## Minimum allowed error to consider bones being already in place when calculating orientations.
@export var rotation_error_margin: float = 0.01
## How much bones will try to snap back to their initial positions.
# @export_range(0.0, 1.0, 0.01) var snap_back_strength: float = 0.0

@onready var skeleton: Skeleton3D = get_skeleton()

var bone_idx_chain: PackedInt32Array
var bone_xforms: Array[Transform3D] = []
var bone_lengths: PackedFloat32Array
var init_bones_basis: Array[Basis] = []
var init_dir_to_successor: PackedVector3Array
var init_bone_positions: PackedVector3Array

var total_chain_length: float = 0.0
var total_chain_length_sqrd: float = 0.0
var distance_error_margin_sqrd: float = 0.0
var rotation_error_margin_sqrd: float = 0.0

var scale_orig: Vector3
var scale_inverse: Vector3
var skeleton_global_xform_prescaled: Transform3D
var skeleton_global_xform_prescaled_inv: Transform3D

var _is_started := false

#region Inspector stuff
func _validate_property(property: Dictionary) -> void:
	if skeleton && is_inside_tree():
		var bone_hint_string := &"None (-1):-1," + skeleton.get_concatenated_bone_names()
		if property.name == &"root_bone" || property.name == &"tip_bone":
			property.hint = PROPERTY_HINT_ENUM
			property.hint_string = bone_hint_string
#endregion

func _ready() -> void:
	_is_started = false
	init_ik()

func _process_modification() -> void:
	if Engine.is_editor_hint() || !is_inside_tree() || !_is_started:
		return
	resolve_ik()

#region Public methods
## This method must be run in `_ready()` to allocate data, in case if `_ready()` is overridden
func init_ik() -> void:
	# build bone idx chain from root to tip
	bone_idx_chain = _build_bone_idx_chain()

	# allocate data
	bone_xforms.resize(bone_idx_chain.size())
	bone_lengths.resize(bone_idx_chain.size() - 1)
	init_bones_basis.resize(bone_idx_chain.size())
	init_dir_to_successor.resize(bone_idx_chain.size())
	init_bone_positions.resize(bone_idx_chain.size())
	
	scale_orig = skeleton.global_transform.basis.get_scale()
	scale_inverse = Vector3.ONE / scale_orig
	skeleton_global_xform_prescaled = skeleton.global_transform.scaled(scale_inverse)
	skeleton_global_xform_prescaled_inv = skeleton_global_xform_prescaled.inverse()

	_build_bone_xforms(true) # build initial xforms on rest poses
	_build_init_bones_positions()
	_build_bone_lengths_and_init_dirs_to_successor()

	total_chain_length = _get_total_chain_length()
	total_chain_length_sqrd = total_chain_length * total_chain_length
	distance_error_margin_sqrd = distance_error_margin * distance_error_margin
	rotation_error_margin_sqrd = rotation_error_margin * rotation_error_margin

	_build_init_bones_basis()
	_is_started = true

## This metod runs the solver, it should be run inside `_process_modification()` method in case if `_process_modification()` is overridden
func resolve_ik() -> void:
	# calculate xform inverse and scale inverse once per frame
	scale_orig = skeleton.global_transform.basis.get_scale()
	scale_inverse = Vector3.ONE / scale_orig
	skeleton_global_xform_prescaled = skeleton.global_transform.scaled(scale_inverse)
	skeleton_global_xform_prescaled_inv = skeleton_global_xform_prescaled.inverse()

	_build_bone_xforms()
	target = target.scaled(scale_inverse)
	pole = pole * scale_inverse
	
	var bones_size := bone_xforms.size()
	var first_bone_to_target := (target.origin - bone_xforms[0].origin)

	# check if target is withing chain length's reach	
	if first_bone_to_target.length_squared() > total_chain_length_sqrd:
		var target_dir := first_bone_to_target.normalized()
		# stretch chain in target direction after root bone
		for i: int in range(1, bones_size):
			bone_xforms[i].origin = bone_xforms[i - 1].origin + (target_dir * bone_lengths[i - 1])
	else:
		var iters := 0
		for curr_iter: int in max_iterations:
			_backward_solve()
			_forward_solve()
			# break early if results are decent enough
			var dist_to_target_sqrd := bone_xforms[bones_size - 1].origin.distance_squared_to(target.origin)
			iters += 1
			if dist_to_target_sqrd < distance_error_margin_sqrd:
				break
	# _apply_snapback()
	_bend_to_pole()
	_orientate_bones()
	_apply_resolved_xforms()
#endregion

#region Private methods
# doesn't look that good, but it's here, maybe will be useful for special cases
# func _apply_snapback() -> void:
# 	var xforms_size := bone_xforms.size()
# 	for i: int in range(xforms_size - 1):
# 		bone_xforms[i + 1].origin = bone_xforms[i + 1].origin.lerp(
# 			bone_xforms[i].origin + init_dir_to_successor[i] * bone_lengths[i],
# 			snap_back_strength
# 		)

func _orientate_bones() -> void:
	var xforms_size := bone_xforms.size()

	for i: int in range(xforms_size):
		if i == xforms_size - 1:
			# TODO add option to handle tip bone orientation differently, for now it copies target orientation
			bone_xforms[i].basis = target.basis
		else:
			# calculate current direction in skeleton's local space
			var current_dir_global := (bone_xforms[i + 1].origin - bone_xforms[i].origin).normalized()
			var current_dir_local := skeleton_global_xform_prescaled_inv.basis * current_dir_global
			# get initial direction precomputed in local space
			var initial_dir_local := init_dir_to_successor[i]
			var rotation_axis := initial_dir_local.cross(current_dir_local)

			# no rotation needed if directions are aligned
			if rotation_axis.length_squared() < rotation_error_margin_sqrd:
				continue
			rotation_axis = rotation_axis.normalized()
			var rotation_angle := initial_dir_local.angle_to(current_dir_local)
			var basis_local := init_bones_basis[i].rotated(rotation_axis, rotation_angle)

			# convert back to global space and update bone basis
			bone_xforms[i].basis = skeleton_global_xform_prescaled.basis * basis_local

func _bend_to_pole() -> void:
	for i: int in range(1, bone_xforms.size() - 1):
		var plane := Plane(
			bone_xforms[i + 1].origin - bone_xforms[i - 1].origin,
			bone_xforms[i - 1].origin
		).normalized()
		var pole_proj := plane.project(pole)
		var bone_proj := plane.project(bone_xforms[i].origin)
		var angle := (bone_proj - bone_xforms[i - 1].origin) \
			.signed_angle_to(pole_proj - bone_xforms[i - 1].origin, plane.normal)

		var rot_basis := Basis(plane.normal, angle)
		bone_xforms[i].origin = rot_basis * \
			(bone_xforms[i].origin - bone_xforms[i - 1].origin) + \
			 bone_xforms[i - 1].origin

# traverse from tip bone to root bone
func _backward_solve() -> void:
	for i: int in range(bone_xforms.size() - 1, 0, -1):
		# set last bone position to target position
		if i == bone_xforms.size() - 1:
			bone_xforms[i].origin = target.origin
		else:
			var dir := (bone_xforms[i].origin - bone_xforms[i + 1].origin).normalized()
			bone_xforms[i].origin = bone_xforms[i + 1].origin + dir * bone_lengths[i]

# traverse from root bone to tip bone
func _forward_solve() -> void:
	for i: int in range(1, bone_xforms.size()):
		var dir := (bone_xforms[i].origin -  bone_xforms[i - 1].origin).normalized()
		bone_xforms[i].origin = bone_xforms[i - 1].origin + dir * bone_lengths[i - 1]

func _apply_resolved_xforms() -> void:
	for i: int in range(bone_idx_chain.size()):
		# back from global xform land
		var final_bone_pose := skeleton_global_xform_prescaled_inv * bone_xforms[i]
		skeleton.set_bone_global_pose(bone_idx_chain[i], final_bone_pose)

# going from root to tip, does not count last xform
func _build_bone_lengths_and_init_dirs_to_successor() -> void:
	var last_bone_idx := bone_idx_chain.size() - 1

	# middle bones
	for i: int in range(last_bone_idx):
		var pos_to_next := bone_xforms[i + 1].origin - bone_xforms[i].origin
		init_dir_to_successor[i] = skeleton_global_xform_prescaled_inv.basis * pos_to_next.normalized()
		bone_lengths[i] = pos_to_next.length()

	# leaf bone
	var target_dir_global = (target.origin - bone_xforms[last_bone_idx].origin).normalized()
	init_dir_to_successor[last_bone_idx] = skeleton_global_xform_prescaled_inv.basis * target_dir_global

func _build_init_bones_positions() -> void:
	for i: int in range(bone_idx_chain.size()):
		init_bone_positions[i] = bone_xforms[i].origin

func _build_init_bones_basis() -> void:
	for i: int in range(bone_idx_chain.size()):
		init_bones_basis[i] = skeleton_global_xform_prescaled_inv.basis * bone_xforms[i].basis

func _build_bone_xforms(on_rest_pose: bool = false) -> void:
	if on_rest_pose:
		for i: int in range(bone_idx_chain.size()):
			bone_xforms[i] = skeleton_global_xform_prescaled * skeleton.get_bone_global_rest(bone_idx_chain[i])
			
		return

	for i: int in range(bone_idx_chain.size()):
		bone_xforms[i] = skeleton_global_xform_prescaled * skeleton.get_bone_global_pose(bone_idx_chain[i])

func _get_total_chain_length() -> float:
	var sum := 0.0
	for bone_len: float in bone_lengths: sum += bone_len

	return sum

func _build_bone_idx_chain() -> Array[int]:
	var chain: Array[int] = []
	if root_bone == -1 || tip_bone == -1:
		return chain

	chain.push_back(tip_bone)
	var parent: int = tip_bone

	while true:
		parent = skeleton.get_bone_parent(parent)
		if parent == -1:
			break
		if parent == root_bone:
			chain.push_back(parent)
			break
		chain.push_back(parent)

	# tip -> root is now root -> tip
	chain.reverse()
	return chain
#endregion