import gdb
import gdb.printing
import re

_MAX_DIM = 1024  # 超过此大小认为是未初始化的垃圾值，跳过
_MAX_STD_VECTOR_ITEMS = 12  # std::vector 预览最多显示多少项
_LAST_VECTOR_SNAPSHOTS = {}


def _get_dims_and_getter(val):
    """
    Returns (rows, cols, get_fn).
    get_fn(i) returns the i-th element in column-major order.
    Returns (rows, cols, None) when data is unavailable or looks uninitialized.
    """
    ms = val['m_storage']
    t = str(val.type.strip_typedefs())

    try:
        rows = int(ms['m_rows'])
    except Exception:
        m = re.search(r'Matrix<[^,]+,\s*(-?\d+),\s*(-?\d+)', t)
        rows = int(m.group(1)) if m else -1

    try:
        cols = int(ms['m_cols'])
    except Exception:
        m = re.search(r'Matrix<[^,]+,\s*(-?\d+),\s*(-?\d+)', t)
        if m:
            cols = int(m.group(2))
            if cols == -1:
                cols = 1
        else:
            cols = 1
    if cols == -1:
        cols = 1

    if rows <= 0 or cols <= 0 or rows > _MAX_DIM or cols > _MAX_DIM:
        return rows, cols, None

    md = ms['m_data']
    try:
        arr = md['array']
        def get(i): return float(arr[i])
    except Exception:
        def get(i): return float(md[i])

    return rows, cols, get


def _is_vector_shape(rows, cols):
    return rows > 0 and cols > 0 and (rows == 1 or cols == 1)


def _vector_values(rows, cols, get):
    size = rows if cols == 1 else cols
    vals = []
    for i in range(size):
        idx = i if cols == 1 else i * rows
        vals.append(get(idx))
    return vals


def _format(name, val):
    rows, cols, get = _get_dims_and_getter(val)
    prefix = f"{name} = " if name else ""

    if get is None:
        if _looks_like_vector_type(val):
            return f"{prefix}Vector(?): <not yet initialized>"
        return f"{prefix}Matrix({rows}x{cols}): <not yet initialized>"

    col_w = 12
    if _is_vector_shape(rows, cols):
        size = rows if cols == 1 else cols
        lines = [f"{prefix}Vector({size}):"]
        for i in range(size):
            idx = i if cols == 1 else i * rows
            try:
                lines.append(f"  [ {get(idx):{col_w}.6g} ]")
            except Exception:
                lines.append(f"  [ {'?':>{col_w}} ]")
        return "\n".join(lines)

    lines = [f"{prefix}Matrix({rows}x{cols}):"]
    for r in range(rows):
        parts = []
        for c in range(cols):
            idx = c * rows + r
            try:
                parts.append(f"{get(idx):{col_w}.6g}")
            except Exception:
                parts.append(f"{'?':>{col_w}}")
        lines.append("  [ " + "  ".join(parts) + " ]")
    return "\n".join(lines)


def _looks_like_vector_type(val):
    t = str(val.type.strip_typedefs())
    return t.startswith('Eigen::Matrix') and (', 1,' in t or ', 1>' in t or ', -1, 1' in t)


def _is_eigen_vector_value(val):
    try:
        rows, cols, get = _get_dims_and_getter(val)
        return get is not None and _is_vector_shape(rows, cols)
    except Exception:
        return False


def _vector_snapshot_key(name, val):
    rows, cols, get = _get_dims_and_getter(val)
    if get is None or not _is_vector_shape(rows, cols):
        return None
    try:
        values = tuple(round(v, 12) for v in _vector_values(rows, cols, get))
        size = rows if cols == 1 else cols
        return (name, size, values)
    except Exception:
        return None


def _discover_local_symbols():
    names = []
    seen = set()
    try:
        block = gdb.selected_frame().block()
    except Exception:
        return names

    while block:
        for symbol in block:
            try:
                if not symbol.is_variable:
                    continue
                name = symbol.print_name
                if not name or name in seen:
                    continue
                seen.add(name)
                names.append(name)
            except Exception:
                continue
        try:
            block = block.superblock
        except Exception:
            break
    return names


def _discover_local_eigen_vectors():
    names = []
    for name in _discover_local_symbols():
        try:
            val = gdb.parse_and_eval(name)
        except Exception:
            continue
        if _is_eigen_vector_value(val):
            names.append(name)
    return names


def _is_std_vector_type_name(type_name):
    return type_name.startswith('std::vector<') or 'std::vector<' in type_name


def _std_vector_element_type_name(type_name):
    m = re.search(r'std::vector<\s*(.+?)\s*(?:,\s*std::allocator<.*>)?>', type_name)
    return m.group(1) if m else ""


def _is_std_vector_of_eigen_vectors_type(val):
    try:
        t = str(val.type.strip_typedefs())
    except Exception:
        return False
    if not _is_std_vector_type_name(t):
        return False
    elem = _std_vector_element_type_name(t)
    return elem.startswith('Eigen::Matrix') and (', 1,' in elem or ', 1>' in elem or ', -1, 1' in elem)


def _std_vector_size(val):
    impl = val['_M_impl']
    start = impl['_M_start']
    finish = impl['_M_finish']
    return int(finish - start)


def _std_vector_element(val, index):
    impl = val['_M_impl']
    start = impl['_M_start']
    return (start + index).dereference()


def _std_vector_snapshot_key(name, val):
    if not _is_std_vector_of_eigen_vectors_type(val):
        return None
    try:
        size = _std_vector_size(val)
    except Exception:
        return None
    if size < 0 or size > _MAX_DIM:
        return None

    items = []
    try:
        for i in range(size):
            elem = _std_vector_element(val, i)
            elem_key = _vector_snapshot_key(f"{name}[{i}]", elem)
            if elem_key is None:
                return None
            items.append(elem_key[2])
        return (name, size, tuple(items))
    except Exception:
        return None


def _format_std_vector_of_eigen_vectors(name, val):
    size = _std_vector_size(val)
    lines = [f"{name} = std::vector<Vector> (size={size}):"]
    preview = min(size, _MAX_STD_VECTOR_ITEMS)
    for i in range(preview):
        elem = _std_vector_element(val, i)
        lines.extend(_format(f"{name}[{i}]", elem).splitlines())
    if size > preview:
        lines.append(f"  ... ({size - preview} more items)")
    return "\n".join(lines)


def _discover_local_std_vector_of_eigen_vectors():
    names = []
    for name in _discover_local_symbols():
        try:
            val = gdb.parse_and_eval(name)
        except Exception:
            continue
        if _is_std_vector_of_eigen_vectors_type(val):
            names.append(name)
    return names


def print_changed_vector(name):
    try:
        val = gdb.parse_and_eval(name)
    except Exception:
        return False, ""

    snapshot = _vector_snapshot_key(name, val)
    if snapshot is None:
        return False, ""

    previous = _LAST_VECTOR_SNAPSHOTS.get(name)
    if previous == snapshot:
        return False, ""

    _LAST_VECTOR_SNAPSHOTS[name] = snapshot
    return True, _format(name, val) + "\n"


def print_changed_std_vector(name):
    try:
        val = gdb.parse_and_eval(name)
    except Exception:
        return False, ""

    snapshot = _std_vector_snapshot_key(name, val)
    if snapshot is None:
        return False, ""

    previous = _LAST_VECTOR_SNAPSHOTS.get(name)
    if previous == snapshot:
        return False, ""

    _LAST_VECTOR_SNAPSHOTS[name] = snapshot
    return True, _format_std_vector_of_eigen_vectors(name, val) + "\n"


def print_changed_local_vectors():
    parts = []
    for name in _discover_local_eigen_vectors():
        changed, text = print_changed_vector(name)
        if changed:
            parts.append(text)
    for name in _discover_local_std_vector_of_eigen_vectors():
        changed, text = print_changed_std_vector(name)
        if changed:
            parts.append(text)
    return parts


def clear_vector_snapshots():
    _LAST_VECTOR_SNAPSHOTS.clear()


class EigenPrettyPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        try:
            return _format("", self.val).split(" = ", 1)[-1]
        except Exception:
            return str(self.val)

    def display_hint(self):
        return 'string'


def build_pp():
    pp = gdb.printing.RegexpCollectionPrettyPrinter("eigen")
    pp.add_printer('Matrix', r'^Eigen::Matrix', EigenPrettyPrinter)
    pp.add_printer('Vector', r'^Eigen::Vector', EigenPrettyPrinter)
    return pp


gdb.printing.register_pretty_printer(None, build_pp(), replace=True)