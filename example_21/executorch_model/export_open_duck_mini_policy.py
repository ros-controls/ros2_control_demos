# Copyright (C) 2026 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Anthony Shoumikhin

"""
Export the Open Duck Mini walking policy to an ExecuTorch program (.pte).

The policy network is written as a PyTorch module. The bundled program takes its weights from the
ONNX policy of example_18, so that both examples drive the robot the same way. A policy trained
in PyTorch only needs the last step: pass the trained module to export_policy(). The controller
links no CPU operator kernels, so every operator of the policy must be delegated to XNNPACK.

Requirements: pip install executorch==1.6.0 onnx
"""

import argparse

import torch
from executorch.backends.xnnpack.partition.xnnpack_partitioner import XnnpackPartitioner
from executorch.exir import to_edge_transform_and_lower


class OpenDuckMiniPolicy(torch.nn.Module):
    """Brax PPO policy: normalize the observation, MLP with SiLU, tanh of the action mean."""

    def __init__(self, observation_size=101, action_size=14, hidden_sizes=(512, 256, 128)):
        super().__init__()
        self.action_size = action_size
        self.register_buffer("observation_mean", torch.zeros(1, observation_size))
        self.register_buffer("observation_std", torch.ones(1, observation_size))
        layers = []
        sizes = (observation_size, *hidden_sizes)
        for in_size, out_size in zip(sizes[:-1], sizes[1:]):
            layers += [torch.nn.Linear(in_size, out_size), torch.nn.SiLU()]
        # The last layer returns the mean and the scale of the action distribution.
        layers.append(torch.nn.Linear(sizes[-1], 2 * action_size))
        self.mlp = torch.nn.Sequential(*layers)

    def forward(self, observation):
        x = (observation - self.observation_mean) / self.observation_std
        return torch.tanh(self.mlp(x)[:, : self.action_size])


def load_onnx_weights(policy, onnx_path):
    """Copy the weights of example_18's ONNX policy, which has the same layers in the same order."""
    # Imported here so that export_policy() needs only executorch.
    import onnx
    from onnx import numpy_helper

    initializers = {
        tensor.name: torch.from_numpy(numpy_helper.to_array(tensor).copy())
        for tensor in onnx.load(onnx_path).graph.initializer
    }
    policy.observation_mean.copy_(initializers["const_10"])
    policy.observation_std.copy_(initializers["const_11"])
    linear_layers = [m for m in policy.mlp if isinstance(m, torch.nn.Linear)]
    for i, layer in enumerate(linear_layers):
        # ONNX stores each weight as (in, out), PyTorch as (out, in).
        layer.weight.copy_(initializers[f"const_{2 + 2 * i}"].T)
        layer.bias.copy_(initializers[f"const_{3 + 2 * i}"])


def export_policy(policy, observation_size, pte_path):
    example_inputs = (torch.zeros(1, observation_size),)
    exported = torch.export.export(policy.eval(), example_inputs)
    program = to_edge_transform_and_lower(
        exported, partitioner=[XnnpackPartitioner()]
    ).to_executorch()
    with open(pte_path, "wb") as f:
        f.write(program.buffer)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--onnx", required=True, help="example_18 ONNX policy to take weights from"
    )
    parser.add_argument("--output", default="open_duck_mini_policy.pte", help="output .pte file")
    args = parser.parse_args()

    policy = OpenDuckMiniPolicy()
    with torch.no_grad():
        load_onnx_weights(policy, args.onnx)
    export_policy(policy, 101, args.output)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
