:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_21/doc/userdoc.rst

.. _ros2_control_demos_example_21_userdoc:

ros2_control_demo_example_21
================================

This demo runs the walking policy of :ref:`example_18 <ros2_control_demos_example_18_userdoc>` from a
PyTorch model. The policy is exported with `ExecuTorch <https://github.com/pytorch/executorch>`_,
PyTorch's solution for deploying models on devices, and runs inside the controller's ``update()``.

The robot, the MuJoCo simulation, the message, and the observation and action classes come from
example_18. The new controller follows the same control steps as example_18's controller, with the
policy running through ExecuTorch, and a test runs both controllers side by side to keep them in
step. Read example_18 first.

Why ExecuTorch in a controller
------------------------------

* The PyTorch module is exported with ``torch.export`` and the ExecuTorch tools into a single
  ``.pte`` file, without an ONNX step.
* The memory of the policy is set up once in ``on_configure()``: the buffers ExecuTorch planned at
  export time, and the XNNPACK workspace, which a first run of the policy creates. After that,
  the policy call in ``update()`` does not allocate heap memory. A test counts allocations to check
  this. The rest of ``update()`` works like example_18's and still allocates on every cycle (it
  copies the interface names and builds several vectors).
* The policy runs on the controller manager's update thread, with no worker threads. It has its
  own XNNPACK workspace, so it does not take the workspace lock that other models in the process
  share. XNNPACK still takes one short process-wide lock once per layer on every call, so the call
  is not lock-free.

The bundled policy runs on the CPU with the XNNPACK backend. A small policy like this one takes
tens of microseconds on one CPU core, so it does not need a GPU. This controller runs only XNNPACK
programs. ExecuTorch also has TensorRT (through Torch-TensorRT) and CUDA backends for NVIDIA GPUs;
using them would need changes to this controller, and they are not tested with this demo.

Where the policy comes from
---------------------------

``executorch_model/export_open_duck_mini_policy.py`` writes the Brax PPO policy network of
example_18 as a PyTorch module. For the bundled ``executorch_model/open_duck_mini_policy.pte``, it
copies the weights from example_18's ONNX model, so both examples drive the robot the same way.
A test checks that both give the same actions.

To deploy your own policy trained in PyTorch, pass your module to ``export_policy()`` in that script
(run Python from the ``executorch_model`` directory of this example):

.. code-block:: python

   from export_open_duck_mini_policy import export_policy

   export_policy(my_policy, 101, "my_policy.pte")  # 101 is the observation size

Then point the ``model_path`` parameter at the new file. The policy must be trained on example_18's
observation and action layout: the controller only checks the sizes (101 inputs, 14 outputs), so a
policy trained on another layout loads, runs and drives the robot wrong.

The export needs ``pip install executorch==1.6.0``, the version whose C++ libraries
``executorch_vendor`` provides. The controller itself needs no Python and no PyTorch.
The controller links only the XNNPACK backend and no CPU operator kernels, so every operator of the
policy must be delegated to XNNPACK, as it is for this MLP. Otherwise loading the policy fails in
``on_configure()``.

Prerequisites
-------------

Install the dependencies of example_18. This example also needs the ``executorch_vendor`` package,
which provides the ExecuTorch C++ libraries for ROS 2. Until it is released, clone
`rosidl_buffer_backends <https://github.com/ros2/rosidl_buffer_backends>`_ into the ``src``
directory of your workspace. ``executorch_vendor`` is proposed in its pull request 36, so until
that is merged, check out its branch. Then install the dependencies of this example and of
``executorch_vendor`` only, since that repository also has packages that need CUDA:

.. code-block:: bash

   cd ~/ros2_ws/src  # the src directory of your ROS 2 workspace
   git clone https://github.com/ros2/rosidl_buffer_backends.git
   cd rosidl_buffer_backends && git fetch origin pull/36/head && git checkout FETCH_HEAD
   cd ~/ros2_ws
   rosdep install --from-paths src/ros2_control_demos \
     src/rosidl_buffer_backends/executorch_vendor/executorch_vendor --ignore-src -r -y

Build
-----

.. code-block:: bash

   cd ~/ros2_ws
   colcon build --symlink-install --packages-up-to ros2_control_demo_example_21
   source install/setup.bash

Run
---

.. code-block:: bash

   ros2 launch ros2_control_demo_example_21 example_21_mujoco.launch.py

In another terminal (source your workspace first), check that the controllers are active:

.. code-block:: bash

   ros2 control list_controllers

.. code-block:: text

   executorch_policy_controller executorch_policy_controller/ExecuTorchPolicyController  active
   joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster            active
   state_interfaces_broadcaster state_interfaces_broadcaster/StateInterfacesBroadcaster  active

Then send a velocity command:

.. code-block:: bash

   ros2 topic pub -r 50 /executorch_policy_controller/cmd_velocity_with_head \
     ros2_control_demo_example_18/msg/VelocityCommandWithHead "{base_velocity: {linear: {x: 0.1}}}"

The controller waits for ``stabilization_delay`` updates, then blends the policy in over
``blend_in_steps`` updates. You should see the duck walk forward, as in example_18.

How the controller runs the policy
----------------------------------

``ExecuTorchPolicy`` (``controller/src/executorch_policy.cpp``) uses the ExecuTorch runtime API:

* ``load()``, called from ``on_configure()``: loads the program, allocates the memory buffers the
  ``forward`` method was planned with at export time, loads the method with its own XNNPACK
  workspace and no thread pool, and runs it once.
* ``run()``, called from ``update()``: copies the observation into the input tensor, executes the
  method and copies the actions out. All buffers already exist, so nothing is allocated.

The higher level ``executorch::extension::Module`` class is simpler to use, but its ``forward()``
returns a new ``std::vector`` on every call, so it allocates in ``update()``.
