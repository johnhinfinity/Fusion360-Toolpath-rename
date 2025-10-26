# RenameToolpaths_ZPlaceholders.py
# Renames every CAM operation to: "<Tool Description> | ZINC:? & ZABS:? | <Operation Type>"
# Skips any op whose name already contains ZINC: or ZABS:

import adsk.core, adsk.cam, adsk.fusion, traceback

SHOW_DIAGNOSTICS = False  # optional peek at the first few renames

def run(context):
    app = adsk.core.Application.get()
    ui  = app.userInterface
    try:
        product = app.activeProduct
        if not isinstance(product, adsk.cam.CAM):
            ui.messageBox('Open a Manufacturing (CAM) document, then run this script.')
            return

        cam: adsk.cam.CAM = adsk.cam.CAM.cast(product)
        if cam.setups.count == 0:
            ui.messageBox('No setups found.')
            return

        renamed = []
        diag_rows = []
        placeholder = 'ZINC:? & ZABS:?'

        for setup in cam.setups:
            for op in setup.allOperations:
                if not isinstance(op, adsk.cam.Operation):
                    continue

                # --- Skip ops you've already named/edited ---
                name_u = op.name.upper()
                if 'ZINC:' in name_u or 'ZABS:' in name_u:
                    continue

                tool_desc = get_tool_description(op)
                op_type   = get_operation_type(op)
                new_name  = f'{tool_desc} | {placeholder} | {op_type}'

                if op.name != new_name:
                    op.name = new_name
                    renamed.append(new_name)

                if SHOW_DIAGNOSTICS and len(diag_rows) < 20:
                    diag_rows.append(f'{op.name}\n  Tool="{tool_desc}"  Placeholder="{placeholder}"  Type="{op_type}"')

        if SHOW_DIAGNOSTICS and diag_rows:
            ui.messageBox('Diagnostics (first 20 ops):\n' + '\n'.join(diag_rows))

        if renamed:
            shown = '\n'.join(renamed[:80])
            more  = '' if len(renamed) <= 80 else f'\n...and {len(renamed)-80} more.'
            ui.messageBox('Toolpath names updated.\n' + shown + more)
        else:
            ui.messageBox('No toolpaths changed (either already named or matched).')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# ---------------- helpers ----------------

def get_tool_description(op: adsk.cam.Operation) -> str:
    try:
        tool = op.tool
        if tool:
            for attr in ('description', 'comment', 'name'):
                val = getattr(tool, attr, None)
                if isinstance(val, str) and val.strip():
                    return val.strip()
            num = getattr(tool, 'number', None)
            if isinstance(num, int) and num > 0:
                return f'T{num}'
    except:
        pass

    s = get_string_param(op, (
        'toolDescription','tool_description',
        'toolComment','tool_comment',
        'toolName','tool_name'
    ))
    if s:
        return s

    tnum = get_numeric_param(op, ('toolNumber','tool_number','tNumber','tNum'))
    if tnum is not None and int(tnum) > 0:
        return f'T{int(tnum)}'

    return 'Tool'

def get_operation_type(op: adsk.cam.Operation) -> str:
    for prop in ('strategy', 'strategyType', 'operationType'):
        try:
            val = getattr(op, prop, None)
            if val is not None:
                return tidy_type(str(val))
        except:
            pass
    try:
        return tidy_type(op.classType().split('.')[-1])
    except:
        return 'Operation'

def tidy_type(s: str) -> str:
    s = s.replace('adsk.cam.', '').replace('Operation', '')
    s = s.replace('twoD','2D').replace('TwoD','2D').replace('threeD','3D').replace('ThreeD','3D')
    out = []
    for i,ch in enumerate(s):
        if i and ch.isupper() and not s[i-1].isspace():
            out.append(' ')
        out.append(ch)
    s = ''.join(out).strip()
    fixes = {
        'Contour 2 D':'2D Contour','Pocket 2 D':'2D Pocket','Bore 2 D':'2D Bore','Slot 2 D':'2D Slot',
        'Parallel 3 D':'3D Parallel','Contour 3 D':'3D Contour','Pocket 3 D':'3D Pocket',
        'Adaptive Clearing':'Adaptive'
    }
    return fixes.get(s, ' '.join(w.capitalize() for w in s.split()))

def get_string_param(op, names):
    params = getattr(op, 'parameters', None)
    if not params:
        return None
    for n in names:
        p = params.itemByName(n)
        if p:
            expr = getattr(p, 'expression', None)
            if isinstance(expr, str) and expr.strip():
                return expr.strip()
            val = getattr(p, 'value', None)
            if isinstance(val, str) and val.strip():
                return val.strip()
    return None

def get_numeric_param(op, names):
    params = getattr(op, 'parameters', None)
    if not params:
        return None
    for n in names:
        p = params.itemByName(n)
        if p:
            val = getattr(p, 'value', None)
            if isinstance(val, (int,float)):
                return float(val)
            expr = getattr(p, 'expression', None)
            if isinstance(expr, str) and expr.strip():
                try:
                    return float(expr)
                except:
                    pass
    return None

def stop(context):
    pass