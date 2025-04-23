from pathlib import Path
import google.protobuf.text_format as text_format

def get_input_dir_data_size(path):
    return sum(f.stat().st_size for f in Path(path).rglob('*') if f.is_file())

def get_pb_from_text_file(filename, pb_value):
    """Get a proto from given text file."""
    with open(filename, 'r', encoding='utf-8') as file_in:
        return text_format.Merge(file_in.read(), pb_value)
