set print pretty on
set print object on
set print static-members off
set pagination off
source /home/mini/Desktop/PrivateFiles/Al_ILQR_Starter/.vscode/eigen_printers.py

define hook-run
  python
import sys
sys.path.insert(0, '/home/mini/Desktop/PrivateFiles/Al_ILQR_Starter/.vscode')
import eigen_printers
eigen_printers.clear_vector_snapshots()
  end
end

define hook-stop
  python
import gdb, sys
# 只在有源码上下文时打印，跳过动态库加载事件
try:
    frame = gdb.selected_frame()
    sal = frame.find_sal()
    if sal.symtab is None:
        raise Exception("no source")
    src = sal.symtab.filename
    if not (src.endswith(".cpp") or src.endswith(".cc") or src.endswith(".c")):
        raise Exception("not cpp")
except Exception:
    pass
else:
    sys.path.insert(0, '/home/mini/Desktop/PrivateFiles/Al_ILQR_Starter/.vscode')
    import eigen_printers
    parts = eigen_printers.print_changed_local_vectors()
    if parts:
        gdb.write("\n========== [当前更新向量] ==========\n")
        for text in parts:
            gdb.write(text)
        gdb.write("=====================================\n")
  end
end
