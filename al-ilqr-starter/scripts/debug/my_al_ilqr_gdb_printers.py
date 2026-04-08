import gdb
import gdb.printing


def _safe_int(v, default=0):
    try:
        return int(v)
    except Exception:
        try:
            return int(v.cast(gdb.lookup_type('int')))
        except Exception:
            return default


def _float_from_value(v, default=0.0):
    try:
        return float(v)
    except Exception:
        try:
            return float(v.cast(gdb.lookup_type('double')))
        except Exception:
            return default


def _strip_typedefs(t):
    try:
        return t.strip_typedefs()
    except Exception:
        return t


def _type_name(val):
    try:
        return str(_strip_typedefs(val.type))
    except Exception:
        return str(val.type)


class EigenMatrixPrinter:
    def __init__(self, val):
        self.val = val

    def _rows_cols(self):
        t = _strip_typedefs(self.val.type)
        try:
            rows = _safe_int(t.template_argument(1), -1)
            cols = _safe_int(t.template_argument(2), -1)
        except Exception:
            rows = -1
            cols = -1

        try:
            storage = self.val['m_storage']
            if rows == -1:
                try:
                    rows = _safe_int(storage['m_rows'], rows)
                except Exception:
                    pass
            if cols == -1:
                try:
                    cols = _safe_int(storage['m_cols'], cols)
                except Exception:
                    pass
        except Exception:
            pass

        return rows, cols

    def _data_ptr(self):
        storage = self.val['m_storage']
        if 'm_data' in [f.name for f in storage.type.fields()]:
            return storage['m_data']
        return storage

    def _options(self):
        t = _strip_typedefs(self.val.type)
        try:
            return _safe_int(t.template_argument(3), 0)
        except Exception:
            return 0

    def _row_major(self):
        # Eigen RowMajor bit = 0x1
        return (self._options() & 0x1) != 0

    def _scalar_type(self):
        t = _strip_typedefs(self.val.type)
        try:
            return t.template_argument(0)
        except Exception:
            return gdb.lookup_type('double')

    def to_string(self):
        rows, cols = self._rows_cols()
        major = 'row-major' if self._row_major() else 'col-major'
        return f"Eigen::Matrix [{rows} x {cols}] ({major})"

    def children(self):
        rows, cols = self._rows_cols()
        if rows < 0 or cols < 0:
            return

        ptr = self._data_ptr()
        scalar_ptr = ptr.cast(self._scalar_type().pointer())
        row_major = self._row_major()

        for i in range(rows):
            row_vals = []
            for j in range(cols):
                idx = i * cols + j if row_major else j * rows + i
                try:
                    item = (scalar_ptr + idx).dereference()
                    row_vals.append(f"{_float_from_value(item):.6g}")
                except Exception:
                    row_vals.append("?")
            yield (f"row[{i}]", "[ " + ", ".join(row_vals) + " ]")

    def display_hint(self):
        return 'map'


class TrajectoryPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        try:
            horizon = _safe_int(self.val['horizon'])
        except Exception:
            horizon = _safe_int(self.val['horizon_'])
        try:
            nx = _safe_int(self.val['state_dim_'])
            nu = _safe_int(self.val['control_dim_'])
        except Exception:
            nx = -1
            nu = -1
        return f"Trajectory(horizon={horizon}, state_dim={nx}, control_dim={nu})"

    def children(self):
        try:
            yield ('state_dim', self.val['state_dim_'])
            yield ('control_dim', self.val['control_dim_'])
            yield ('horizon', self.val['horizon_'])
            yield ('dt', self.val['dt_'])
            yield ('states', self.val['states_'])
            yield ('controls', self.val['controls_'])
        except Exception:
            return

    def display_hint(self):
        return 'map'


class MyALILQRCollectionPrinter:
    def __init__(self):
        self.name = 'my_al_ilqr_printers'
        self.enabled = True

    def __call__(self, val):
        tname = _type_name(val)
        if 'Eigen::Matrix' in tname:
            return EigenMatrixPrinter(val)
        if 'my_al_ilqr::Trajectory' in tname:
            return TrajectoryPrinter(val)
        return None


gdb.printing.register_pretty_printer(gdb.current_objfile(), MyALILQRCollectionPrinter(), replace=True)
print('[my_al_ilqr] GDB pretty printers loaded')
