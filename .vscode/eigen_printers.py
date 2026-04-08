import gdb
import gdb.printing
import re

_MAX_DIM = 1024  # 超过此大小认为是未初始化的垃圾值，跳过


def _get_dims_and_getter(val):
    """
    Returns (rows, cols, get_fn).
    get_fn(i) returns the i-th element in column-major order.
    Returns (rows, cols, None) when data is unavailable or looks uninitialized.
    """
    ms = val['m_storage']
    t = str(val.type.strip_typedefs())

    # --- rows ---
    try:
        rows = int(ms['m_rows'])
    except Exception:
        m = re.search(r'Matrix<[^,]+,\s*(-?\d+),\s*(-?\d+)', t)
        rows = int(m.group(1)) if m else -1

    # --- cols ---
    try:
        cols = int(ms['m_cols'])
    except Exception:
        m = re.search(r'Matrix<[^,]+,\s*(-?\d+),\s*(-?\d+)', t)
        if m:
            cols = int(m.group(2))
            if cols == -1:
                cols = 1   # VectorXd alias: fixed 1 column
        else:
            cols = 1
    if cols == -1:
        cols = 1

    # Sanity-check: reject uninitialized / garbage dims
    if rows <= 0 or cols <= 0 or rows > _MAX_DIM or cols > _MAX_DIM:
        return rows, cols, None

    # --- data ---
    md = ms['m_data']
    try:
        arr = md['array']
        def get(i): return float(arr[i])
    except Exception:
        def get(i): return float(md[i])

    return rows, cols, get


def _format(name, val):
    rows, cols, get = _get_dims_and_getter(val)

    prefix = f"{name} = " if name else ""

    if get is None:
        return f"{prefix}Matrix({rows}x{cols}): <not yet initialized>"

    col_w = 12
    lines = [f"{prefix}Matrix({rows}x{cols}):"]
    for r in range(rows):
        parts = []
        for c in range(cols):
            idx = c * rows + r   # column-major
            try:
                parts.append(f"{get(idx):{col_w}.6g}")
            except Exception:
                parts.append(f"{'?':>{col_w}}")
        lines.append("  [ " + "  ".join(parts) + " ]")
    return "\n".join(lines)


def print_eigen(name):
    """Print Eigen variable. Called from hook-stop in .gdbinit."""
    try:
        val = gdb.parse_and_eval(name)
        gdb.write(_format(name, val) + "\n")
    except Exception:
        pass   # variable not in scope yet — silently skip


class EigenPrettyPrinter:
    """Used by 'p var' in Debug Console."""
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
