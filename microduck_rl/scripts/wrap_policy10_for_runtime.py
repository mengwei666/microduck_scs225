#!/usr/bin/env python3
"""Wrap a legs-only 10-DoF policy ONNX for the 14-action/61-obs robotd runtime.

Training task: Mjlab-Velocity-Flat-MicroDuck10
  actor obs: 3 ang_vel + 3 gravity + 10 joint_pos + 10 joint_vel + 10 actions
             + 3 twist + 4 head_command + 6 body_command = 49
  actions:   10 (left leg 0-4, right leg 5-9)

Runtime (duck-control / robotd):
  obs: 3 + 3 + 14 joint_pos + 14 joint_vel + 14 last_action
       + 3 twist + 4 head + 6 body = 61
  actions: 14 slots = left leg(0-4), head(5-8), right leg(9-13)

The wrapper selects the 10 leg joints/actions from the runtime vectors, zeroes
the unused head/body command slots, runs the 10-DoF policy, and pads its output
back to 14 actions. The resulting ONNX loads in the existing robotd unchanged.

Usage:
    uv run --no-sync scripts/wrap_policy10_for_runtime.py \
        --input policy10.onnx --output policy10_runtime.onnx
"""

import argparse
from copy import deepcopy

import numpy as np
import onnx
import onnxruntime as ort
from onnx import TensorProto, helper, numpy_helper

# Runtime 14-joint order: left leg 0-4, head/neck 5-8, right leg 9-13.
RUNTIME_LEG_SLOTS = [0, 1, 2, 3, 4, 9, 10, 11, 12, 13]
POS_BASE = 6
VEL_BASE = 20
ACT_BASE = 34
TWIST_BASE = 48

JOINT_ORDER = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]


def _const(name, arr):
    return numpy_helper.from_array(np.asarray(arr, dtype=np.int64), name)


def _const_f32(name, arr):
    return numpy_helper.from_array(np.asarray(arr, dtype=np.float32), name)


def wrap(input_path, output_path, flips=()):
    model = onnx.load(input_path)
    graph = deepcopy(model.graph)
    if len(graph.input) != 1:
        raise SystemExit(f"expected one input, got {[i.name for i in graph.input]}")
    if len(graph.output) != 1:
        raise SystemExit(f"expected one output, got {[o.name for o in graph.output]}")

    old_in = graph.input[0].name
    old_out = graph.output[0].name
    new_out = "actions"
    obs10 = "obs_10dof"
    # The original output name is often "actions"; rename it so the padded
    # 14-wide output can take that name without breaking SSA.
    policy_out = "policy10_actions"
    for node in graph.node:
        for i, name in enumerate(node.output):
            if name == old_out:
                node.output[i] = policy_out
        for i, name in enumerate(node.input):
            if name == old_out:
                node.input[i] = policy_out
    old_out = policy_out

    # Every consumer of the old input now reads the 49-D projection.
    for node in graph.node:
        for i, name in enumerate(node.input):
            if name == old_in:
                node.input[i] = obs10

    sign = np.ones((1, 10), dtype=np.float32)
    for name in flips:
        if name not in JOINT_ORDER:
            raise SystemExit(f"unknown joint in --flip: {name}")
        sign[0, JOINT_ORDER.index(name)] = -1.0
    if np.any(sign < 0):
        print("flipping direction for: " + ", ".join(flips))

    # Build the 61 -> 49 projection.
    inits = [
        _const_f32("_sign", sign),
        _const("_idx_pos", [POS_BASE + i for i in RUNTIME_LEG_SLOTS]),
        _const("_idx_vel", [VEL_BASE + i for i in RUNTIME_LEG_SLOTS]),
        _const("_idx_act", [ACT_BASE + i for i in RUNTIME_LEG_SLOTS]),
        _const("_idx_twist", [TWIST_BASE + i for i in range(3)]),
    ]
    nodes = [
        helper.make_node("Slice", ["obs", "_zero", "_six", "_one"], ["_ang_grav"]),
        helper.make_node("Gather", ["obs", "_idx_pos"], ["_jpos_raw"], axis=1),
        helper.make_node("Gather", ["obs", "_idx_vel"], ["_jvel_raw"], axis=1),
        helper.make_node("Gather", ["obs", "_idx_act"], ["_last_act_raw"], axis=1),
        helper.make_node("Gather", ["obs", "_idx_twist"], ["_twist"], axis=1),
        helper.make_node("Mul", ["_jpos_raw", "_sign"], ["_jpos"]),
        helper.make_node("Mul", ["_jvel_raw", "_sign"], ["_jvel"]),
        helper.make_node("Mul", ["_last_act_raw", "_sign"], ["_last_act"]),
        helper.make_node(
            "Constant", [], ["_head"], value=_const_f32("_head_val", np.zeros((1, 4)))
        ),
        helper.make_node(
            "Constant", [], ["_body"], value=_const_f32("_body_val", np.zeros((1, 6)))
        ),
        helper.make_node(
            "Concat",
            ["_ang_grav", "_jpos", "_jvel", "_last_act", "_twist", "_head", "_body"],
            [obs10],
            axis=1,
        ),
    ]
    inits += [
        _const("_zero", np.array([0], dtype=np.int64)),
        _const("_six", np.array([6], dtype=np.int64)),
        _const("_one", np.array([1], dtype=np.int64)),
    ]

    # Pad 10 actions -> [left 5, head 4 zeros, right 5].
    nodes += [
        helper.make_node("Mul", [old_out, "_sign"], ["_actions_signed"]),
        helper.make_node("Slice", ["_actions_signed", "_a0", "_a5", "_ax"], ["_left"]),
        helper.make_node("Slice", ["_actions_signed", "_a5", "_a10", "_ax"], ["_right"]),
        helper.make_node(
            "Constant", [], ["_head_act"], value=_const_f32("_head_act_val", np.zeros((1, 4)))
        ),
        helper.make_node("Concat", ["_left", "_head_act", "_right"], [new_out], axis=1),
    ]
    inits += [
        _const("_a0", np.array([0], dtype=np.int64)),
        _const("_a5", np.array([5], dtype=np.int64)),
        _const("_a10", np.array([10], dtype=np.int64)),
        _const("_ax", np.array([1], dtype=np.int64)),
    ]

    # 11 projection nodes: Slice, 4 Gather, 3 Mul, 2 Constant, Concat.
    pre_nodes = nodes[:11]
    post_nodes = nodes[11:]
    for node in reversed(pre_nodes):
        graph.node.insert(0, node)
    graph.node.extend(post_nodes)
    graph.initializer.extend(inits)
    del graph.input[:]
    graph.input.append(helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 61]))
    del graph.output[:]
    graph.output.append(helper.make_tensor_value_info(new_out, TensorProto.FLOAT, [1, 14]))

    wrapped = helper.make_model(
        graph,
        producer_name="microduck-10dof-runtime-wrapper",
        opset_imports=model.opset_import,
    )
    wrapped.ir_version = model.ir_version
    onnx.checker.check_model(wrapped)
    onnx.save(wrapped, output_path)
    print(f"wrote {output_path}: obs[1,61] -> actions[1,14]")

    # Smoke-test shapes and mapping against the original policy.
    x61 = np.random.randn(1, 61).astype(np.float32)
    x49 = np.concatenate([
        x61[:, :6],
        x61[:, [POS_BASE + i for i in RUNTIME_LEG_SLOTS]] * sign,
        x61[:, [VEL_BASE + i for i in RUNTIME_LEG_SLOTS]] * sign,
        x61[:, [ACT_BASE + i for i in RUNTIME_LEG_SLOTS]] * sign,
        x61[:, TWIST_BASE:TWIST_BASE + 3],
        np.zeros((1, 4), np.float32),
        np.zeros((1, 6), np.float32),
    ], axis=1)
    s0 = ort.InferenceSession(input_path, providers=["CPUExecutionProvider"])
    s1 = ort.InferenceSession(output_path, providers=["CPUExecutionProvider"])
    a10 = s0.run(None, {s0.get_inputs()[0].name: x49})[0]
    a14 = s1.run(None, {s1.get_inputs()[0].name: x61})[0]
    assert a14.shape == (1, 14), a14.shape
    expect = a10 * sign
    assert np.allclose(a14[:, :5], expect[:, :5], atol=1e-6)
    assert np.allclose(a14[:, 5:9], 0.0, atol=1e-6)
    assert np.allclose(a14[:, 9:], expect[:, 5:], atol=1e-6)
    print("smoke test OK: runtime actions match policy10 actions")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="10-DoF policy .onnx")
    ap.add_argument("--output", required=True, help="wrapped runtime .onnx")
    ap.add_argument("--flip", default="",
                    help="comma-separated joint names whose direction is reversed")
    args = ap.parse_args()
    flips = [x for x in args.flip.split(",") if x]
    wrap(args.input, args.output, flips)
