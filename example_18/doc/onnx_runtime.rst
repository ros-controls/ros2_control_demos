ONNX Runtime Integration
========================

Introduction
------------

ONNX Runtime is a cross-platform library for executing ONNX machine learning models. The MotionController uses ONNX Runtime to run inference with trained policy models, converting sensor observations into joint position commands.

Tensor Handling
---------------

In C++, ONNX Runtime represents tensors as ``Ort::Value`` objects that wrap contiguous memory arrays with shape metadata. The implementation follows these steps:

Tensor Creation
~~~~~~~~~~~~~~~

The formatted observation vector (``std::vector<float>``) is wrapped into an ``Ort::Value`` tensor using ``Ort::Value::CreateTensor<float>()``:

- Memory allocator: Uses CPU allocator created during model loading
- Data pointer: Points to the underlying C++ array data (from ``std::vector::data()``)
- Element count: Total number of elements in the observation vector
- Shape array: ``std::vector<int64_t>`` defining tensor dimensions (e.g., ``[1, 46]`` for batch size 1 and 46 features)
- Dimension count: Number of dimensions in the shape array

The tensor references the original memory without copying, avoiding unnecessary memory allocation. Dynamic dimensions (marked as ``-1`` in the model) are resolved at runtime based on the actual input size.

Tensor Type
~~~~~~~~~~~

The element type is specified by ``ONNXTensorElementDataType`` enum. Most ML models use ``ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT`` (float32), which matches the C++ ``float`` type. The observation formatter produces ``std::vector<float>`` to match this requirement.

Inference Execution
~~~~~~~~~~~~~~~~~~~

The input tensor is passed to ``onnx_session_->Run()`` along with:

- Input/output name pointers: String pointers to input and output tensor names (e.g., ``"obs"`` for input)
- Input tensor: The ``Ort::Value`` tensor containing the observation data
- Output tensor count: Number of output tensors expected (typically 1)

ONNX Runtime performs the inference and returns output tensors as a vector of ``Ort::Value`` objects.

Output Extraction
~~~~~~~~~~~~~~~~~

After inference, output tensors are accessed via:

- ``GetTensorMutableData<float>()``: Returns a typed pointer (``float*``) to the underlying tensor data
- ``GetTensorTypeAndShapeInfo().GetElementCount()``: Returns the total number of elements

The output data is then converted from ``float`` to ``double`` and copied into a ``std::vector<double>`` for compatibility with ROS2 control interfaces, which use double precision.

This approach avoids data copying during tensor creation while maintaining type safety and compatibility with both ONNX Runtime (float32) and ROS2 control interfaces (float64).

Model Metadata
--------------

During model loading, the controller extracts and validates model metadata:

- Input count and names: Queried via ``GetInputCount()`` and ``GetInputNameAllocated()``
- Input shape: Extracted via ``GetInputTypeInfo()`` and ``GetTensorTypeAndShapeInfo().GetShape()``
- Output count and names: Queried via ``GetOutputCount()`` and ``GetOutputNameAllocated()``
- Output shape: Extracted via ``GetOutputTypeInfo()`` and ``GetTensorTypeAndShapeInfo().GetShape()``

The ``Ort::AllocatorWithDefaultOptions`` is used when retrieving string names from the session, as ``GetInputNameAllocated()`` and ``GetOutputNameAllocated()`` require an allocator to return the name strings.

Model Output Processing
-----------------------

The ONNX model outputs relative joint positions (N joints) that are processed by the ActionProcessor:

1. Scaling: Model outputs are multiplied by ``action_scale`` (default: 0.25, configurable via parameter)
2. Offset addition: Default joint positions are added to convert relative positions to absolute positions
3. Result: Absolute joint positions ready for hardware command interfaces

The default joint positions are initialized from sensor data on the first controller update, or can be provided via the ``default_joint_positions`` parameter in the controller configuration.
