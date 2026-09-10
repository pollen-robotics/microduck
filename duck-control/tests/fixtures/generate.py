"""Regenerate tiny contract fixtures: python with onnx + numpy installed.

The recurrent fixture uses the real ONNX LSTM operator, with deterministic weights.
No trained model or hardware is needed. Output order deliberately differs from mjlab.
"""
from pathlib import Path
import copy
import numpy as np
import onnx
from onnx import TensorProto as T, helper as h, numpy_helper as nh

ROOT = Path(__file__).parent

def info(name, shape, dtype=T.FLOAT):
    return h.make_tensor_value_info(name, dtype, shape)

def save(name, nodes, inputs, outputs, initializers):
    model = h.make_model(h.make_graph(nodes, name, inputs, outputs, initializers),
                        opset_imports=[h.make_opsetid('', 17)], ir_version=8)
    onnx.checker.check_model(model)
    onnx.save(model, ROOT / (name + '.onnx'))
    return model

ff = save('feedforward', [h.make_node('Gather', ['obs', 'indices'], ['output'], axis=1)],
          [info('obs', [1, 61])], [info('output', [1, 14])],
          [nh.from_array(np.arange(14, dtype=np.int64), 'indices')])
rng = np.random.default_rng(42)
weights = [nh.from_array(rng.normal(0, .15, shape).astype('float32'), name)
           for name, shape in [('W', (1, 8, 61)), ('R', (1, 8, 2)), ('B', (1, 16))]]
model = save('lstm', [
    h.make_node('Unsqueeze', ['obs', 'axis'], ['x']),
    h.make_node('LSTM', ['x', 'W', 'R', 'B', '', 'h_in', 'c_in'],
                ['y', 'h_out', 'c_out'], hidden_size=2),
    h.make_node('Squeeze', ['h_out', 'axis'], ['flat']),
    h.make_node('Tile', ['flat', 'repeats'], ['actions']),
], [info('c_in', [1, 1, 2]), info('obs', [1, 61]), info('h_in', [1, 1, 2])],
   [info('c_out', [1, 1, 2]), info('actions', [1, 14]), info('h_out', [1, 1, 2])],
   weights + [nh.from_array(np.array([0], dtype=np.int64), 'axis'),
              nh.from_array(np.array([1, 7], dtype=np.int64), 'repeats')])

def variant(name, mutate, source=model):
    m = copy.deepcopy(source)
    mutate(m)
    # Some invalid contracts also violate graph shape inference: they must be
    # rejected, whether by ORT itself or by our load-time contract validation.
    onnx.save(m, ROOT / (name + '.onnx'))

def dimension(m, io, name, index, value):
    entry = next(x for x in getattr(m.graph, io) if x.name == name)
    d = entry.type.tensor_type.shape.dim[index]
    d.ClearField('dim_param')
    if isinstance(value, str):
        d.ClearField('dim_value')
        d.dim_param = value
    else:
        d.dim_value = value

variant('bad_width', lambda m: dimension(m, 'input', 'obs', 1, 60))
variant('bad_batch', lambda m: dimension(m, 'input', 'obs', 0, 2))
variant('bad_state_shape', lambda m: dimension(m, 'input', 'c_in', 2, 3))
variant('dynamic_hidden', lambda m: dimension(m, 'input', 'h_in', 2, 'hidden'))
variant('missing_state', lambda m: m.graph.input.remove(m.graph.input[0]))
variant('extra_input', lambda m: m.graph.input.append(info('unexpected', [1])))
variant('wrong_type', lambda m: setattr(m.graph.input[0].type.tensor_type, 'elem_type', T.DOUBLE))
variant('dynamic_batch', lambda m: [dimension(m, io, x.name, 0 if x.name in ('obs', 'actions') else 1, 'batch')
                                   for io in ('input', 'output') for x in getattr(m.graph, io)])
variant('bad_rank', lambda m: m.graph.input[0].type.tensor_type.shape.dim.insert(0, h.make_tensor_type_proto(T.FLOAT, [1]).tensor_type.shape.dim[0]), ff)
variant('bad_action_count', lambda m: m.graph.initializer[0].CopyFrom(nh.from_array(np.arange(13, dtype=np.int64), 'indices')), ff)
# Non-finite state should fail during warm-up even when its action output is finite.
nan = copy.deepcopy(model)
for node in nan.graph.node:
    for i, name in enumerate(node.output):
        if name == 'c_out': node.output[i] = 'cell'
nan.graph.node.append(h.make_node('Add', ['cell', 'nan'], ['c_out']))
nan.graph.initializer.append(nh.from_array(np.array(np.nan, dtype=np.float32), 'nan'))
onnx.save(nan, ROOT / 'nan_state.onnx')

variant('lstm_changed', lambda m: next(x for x in m.graph.initializer if x.name == 'B').CopyFrom(
    nh.from_array(np.full((1, 16), .3, dtype=np.float32), 'B')))
