Minimalistic 3D FABRIK re-implementation in GDScript.

This script is not meant to be a plug-and-play solution. It was created as a learning project and can serve as a minimal base for getting started.

It's also not a plugin, it doesn't create any UI beyond exported fields in the inspector, so can be moved around the project freely. There are better/faster solutions using GDNative out there, this one can be too slow for large-scale applications (1000+ instances). Takes skeleton scale into account. No angle constraints as of now.

 Designed to be inherited for specific types of IK implementations (arms, legs, tentacles, tails and what not), where actual animation deforms can be snapshotted and stored before the IK solve pass.

Data allocation phase is containd in `init_ik()` method, and needs to be run as soon as the node is ready. The entire IK pass is contained in the `resolve_ik()` method, which needs to be run in the `_process_modification()` hook. All transforms/origins, such as target or pole, are stored as values directly to minimize external node reliance.

Based on https://github.com/ditzel/SimpleIK with tweaks and fixes.
Operates on matrices instead of quaternions to ensure better bone orientation.