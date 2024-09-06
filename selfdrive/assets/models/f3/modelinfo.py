import onnx
import sys

def get_tensor_size(tensor_type):
    size = 1
    for dim in tensor_type.shape.dim:
        size *= dim.dim_value
    return size

def print_model_info(model_path):
    model = onnx.load(model_path)
    graph = model.graph

    print("Inputs:")
    offset = 0
    for input_tensor in graph.input:
        tensor_shape = [dim.dim_value for dim in input_tensor.type.tensor_type.shape.dim]
        tensor_size = get_tensor_size(input_tensor.type.tensor_type)
        print(f"Name: {input_tensor.name}, Shape: {tensor_shape}, Size: {tensor_size}, Offset: {offset}")
        offset += tensor_size

    print("\nOutputs:")
    offset = 0
    for output_tensor in graph.output:
        tensor_shape = [dim.dim_value for dim in output_tensor.type.tensor_type.shape.dim]
        tensor_size = get_tensor_size(output_tensor.type.tensor_type)
        print(f"Name: {output_tensor.name}, Shape: {tensor_shape}, Size: {tensor_size}, Offset: {offset}")
        offset += tensor_size

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <path_to_model.onnx>")
        sys.exit(1)

    model_path = sys.argv[1]
    print_model_info(model_path)
