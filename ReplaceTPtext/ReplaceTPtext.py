# RenameToolpaths_ToolDepthOp_paramaware_chamfer_autogen_scan.py
# Names toolpaths: "<Tool Description> | Depth: <depth from TOP> | <Operation Type>"
#
# Highlights:
# - Unitless numeric expressions -> mm (fixes 10× inflation)
# - Parameter-aware expressions (resolves names like chamferDiameter)
# - Chamfer drilling: depth = (D/2)/tan(pointAngle/2)
# - NEW: Optional auto-generate missing toolpaths to get Z extents
# - NEW: Depth-parameter sweep: if common params are missing, search any "depth"-like param
# - Safety: if depth can't be trusted, op is not renamed

import adsk.core, adsk.cam, adsk.fusion, traceback, re, math, time

SHOW_DIAGNOSTICS = False
STRICT_NO_RENAME_IF_DEPTH_UNKNOWN = True
AUTO_GENERATE_MISSING_TOOLPATHS = True
GENERATE_TIMEOUT_SEC = 8.0

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

        um = getattr(app.activeProduct, 'unitsManager', None) or app.unitsManager
        display_units  = getattr(um, 'displayUnits', 'in')
        internal_units = getattr(um, 'internalUnits', 'cm')  # Fusion CAM internal usually 'cm'

        renamed = []
        diag_rows = []
        skipped = 0

        for setup in cam.setups:
            stock_thick_cm = get_setup_stock_thickness_cm(setup, um, internal_units)

            for op in setup.allOperations:
                if not isinstance(op, adsk.cam.Operation):
                    continue

                tool_desc = get_tool_description(op)
                op_type   = get_operation_type(op)

                # Build parameter map for this operation (name -> value in cm)
                param_map_cm = build_param_map_cm(op, um, internal_units)

                # ---- Candidates (all cm) ----
                depth_chamfer_cm = get_chamfer_depth_cm(op, um, param_map_cm)       # chamfer diameter + tip angle
                depth_drill_cm   = get_drill_depth_cm(op, um, param_map_cm)         # drill-specific params

                # Heights: expression-first with unitless-as-mm & param substitution
                top_expr_cm, bot_expr_cm = get_height_top_bottom_expr_paramaware(op, um, internal_units, param_map_cm)
                depth_param_expr_cm = abs(bot_expr_cm - top_expr_cm) if (top_expr_cm is not None and bot_expr_cm is not None) else None

                # Heights: numeric-as-mm fallback
                top_mm_cm, bot_mm_cm = get_height_top_bottom_numeric_as_mm(op, um)
                depth_param_mm_cm = abs(bot_mm_cm - top_mm_cm) if (top_mm_cm is not None and bot_mm_cm is not None) else None

                # Depth-like parameter sweep (catch-all)
                depth_sweep_cm = scan_depthlike_params_cm(op, um, param_map_cm)

                # Toolpath vs top (prefer expr top; else stock top)
                maybe_generate_toolpath(op)  # try to generate if missing
                top_hint_cm = top_expr_cm
                depth_tp_cm = get_toolpath_depth_cm(op, setup, um, internal_units, top_hint_cm)

                # ---- Reconcile (handles 10×) ----
                depth_cm, src, corrected = reconcile_all(depth_chamfer_cm, depth_drill_cm, depth_param_expr_cm,
                                                         depth_param_mm_cm, depth_sweep_cm, depth_tp_cm, stock_thick_cm)

                if not is_positive(depth_cm) and STRICT_NO_RENAME_IF_DEPTH_UNKNOWN:
                    skipped += 1
                    if SHOW_DIAGNOSTICS and len(diag_rows) < 20:
                        diag_rows.append(f'(SKIP) {op.name}\n  Tool="{tool_desc}"  Depth=unknown  Type="{op_type}"  src=None')
                    continue

                depth_display = convert_units(depth_cm, um, internal_units, display_units)
                depth_str = format_depth(depth_display, display_units)
                new_name = f'{tool_desc} | Depth: {depth_str} | {op_type}'

                if op.name != new_name:
                    op.name = new_name
                    renamed.append(new_name)

                if SHOW_DIAGNOSTICS and len(diag_rows) < 20:
                    star = '*' if corrected else ''
                    diag_rows.append(f'{op.name}\n  Tool="{tool_desc}"  Depth={depth_str}  Type="{op_type}"  src={src}{star}')

        if SHOW_DIAGNOSTICS and diag_rows:
            ui.messageBox('Diagnostics (first 20 ops):\n' + '\n'.join(diag_rows))

        if renamed:
            shown = '\n'.join(renamed[:80])
            more  = '' if len(renamed) <= 80 else f'\n...and {len(renamed)-80} more.'
            msg = 'Toolpath names updated.\n' + shown + more
            if STRICT_NO_RENAME_IF_DEPTH_UNKNOWN and skipped:
                msg += f'\n\nSkipped {skipped} ops with unknown depth (safety ON).'
            ui.messageBox(msg)
        else:
            if STRICT_NO_RENAME_IF_DEPTH_UNKNOWN and skipped:
                ui.messageBox(f'No toolpaths changed.\nSkipped {skipped} ops with unknown depth (safety ON).')
            else:
                ui.messageBox('No toolpaths changed (names may already match).')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# ---------------- auto-generate toolpaths ----------------

def maybe_generate_toolpath(op):
    if not AUTO_GENERATE_MISSING_TOOLPATHS:
        return
    try:
        if not getattr(op, 'hasToolpath', False):
            op.generateToolpath()
            # wait up to timeout for generation
            t0 = time.time()
            while time.time() - t0 < GENERATE_TIMEOUT_SEC:
                adsk.doEvents()
                if getattr(op, 'hasToolpath', False):
                    break
                time.sleep(0.1)
    except:
        pass

# ---------------- tool / op type helpers ----------------

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
    s = get_string_param(op, ('toolDescription','tool_description','toolComment','tool_comment','toolName','tool_name'))
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
    s = s.replace('twoD','2D').replace('TwoD','2D').replace('threeD','3D')
    out = []
    for i,ch in enumerate(s):
        if i and ch.isupper() and not s[i-1].isspace():
            out.append(' ')
        out.append(ch)
    s = ''.join(out).strip()
    fixes = {'Contour 2 D':'2D Contour','Pocket 2 D':'2D Pocket','Bore 2 D':'2D Bore','Slot 2 D':'2D Slot',
             'Parallel 3 D':'3D Parallel','Contour 3 D':'3D Contour','Pocket 3 D':'3D Pocket','Adaptive Clearing':'Adaptive'}
    return fixes.get(s, ' '.join(w.capitalize() for w in s.split()))

# ---------------- chamfer drilling depth ----------------

def get_chamfer_depth_cm(op: adsk.cam.Operation, um, param_map_cm: dict):
    D_cm = get_chamfer_diameter_cm(op, um, param_map_cm)
    if not is_positive(D_cm):
        return None
    angle_deg = get_tool_point_angle_deg(op, um, param_map_cm) or 118.0
    try:
        half = math.radians(angle_deg * 0.5)
        if half <= 0 or math.tan(half) == 0:
            return None
        depth_cm = (D_cm / 2.0) / math.tan(half)
        return depth_cm if depth_cm > 0 else None
    except:
        return None

def get_chamfer_diameter_cm(op, um, param_map_cm: dict):
    names = ('chamferDiameter','chamfer_diameter','cskDiameter','countersinkDiameter')
    params = getattr(op, 'parameters', None)
    if params:
        for n in names:
            p = params.itemByName(n)
            if p:
                v = read_param_to_cm_unitless_mm_paramaware(p, um, param_map_cm)
                if is_positive(v):
                    return v
    for k, v in param_map_cm.items():
        kl = k.lower()
        if 'chamfer' in kl and ('diam' in kl or 'csk' in kl or 'sink' in kl):
            if is_positive(v):
                return float(v)
    return None

def get_tool_point_angle_deg(op, um, param_map_cm: dict):
    try:
        tool = op.tool
        for attr in ('pointAngle','tipAngle','drillPointAngle'):
            if hasattr(tool, attr):
                val = getattr(tool, attr)
                if isinstance(val, (int,float)) and val > 0:
                    return math.degrees(val) if val <= 3.5 else float(val)
    except:
        pass
    deg_names = ('toolPointAngle','pointAngle','tipAngle','drillPointAngle')
    params = getattr(op, 'parameters', None)
    if params:
        for n in deg_names:
            p = params.itemByName(n)
            if p:
                try:
                    expr = getattr(p, 'expression', None)
                    if isinstance(expr, str) and expr.strip():
                        return float(um.evaluateExpression(expr, 'deg'))
                except:
                    pass
                try:
                    v = getattr(p, 'value', None); u = getattr(p, 'unit', None)
                    if isinstance(v, (int,float)):
                        if u: return float(um.evaluateExpression(f'{float(v)} {u}', 'deg'))
                        return float(v) if v > 3.5 else math.degrees(float(v))
                except:
                    pass
    try:
        tool = op.tool
        tparams = getattr(tool, 'parameters', None)
        if tparams:
            for n in deg_names:
                p = tparams.itemByName(n)
                if p:
                    try:
                        expr = getattr(p, 'expression', None)
                        if isinstance(expr, str) and expr.strip():
                            return float(um.evaluateExpression(expr, 'deg'))
                    except:
                        pass
                    try:
                        v = getattr(p, 'value', None); u = getattr(p, 'unit', None)
                        if isinstance(v, (int,float)):
                            if u: return float(um.evaluateExpression(f'{float(v)} {u}', 'deg'))
                            return float(v) if v > 3.5 else math.degrees(float(v))
                    except:
                        pass
    except:
        pass
    return 118.0

# ---------------- depth readers ----------------

def is_positive(v) -> bool:
    try:
        return v is not None and float(v) > 0.0
    except:
        return False

def get_drill_depth_cm(op: adsk.cam.Operation, um, param_map_cm) -> float:
    names = ('holeDepth','hole_depth','drillDepth','drill_depth','depth','totalDepth','total_depth')
    params = getattr(op, 'parameters', None)
    if not params:
        return None
    for n in names:
        p = params.itemByName(n)
        if not p: continue
        expr = getattr(p, 'expression', None)
        if isinstance(expr, str) and expr.strip():
            try:
                v = eval_expr_with_params_to_cm(expr, um, param_map_cm, default_unit_when_unitless='mm')
                if v > 0: return v
            except: pass
        v = getattr(p, 'value', None); u = getattr(p, 'unit', None)
        if isinstance(v, (int,float)):
            try:
                if u: return float(um.evaluateExpression(f'{float(v)} {u}', 'cm'))
                else: return float(um.evaluateExpression(f'{float(v)} mm', 'cm'))
            except:
                try: return float(v) / 10.0
                except: pass
    return None

def get_height_top_bottom_expr_paramaware(op, um, internal_units: str, param_map_cm: dict):
    top = None; bot = None
    try:
        h = getattr(op, 'heights', None)
        if h:
            top = read_cam_param_expr_paramaware(h.topHeight, um, internal_units, param_map_cm)
            bot = read_cam_param_expr_paramaware(h.bottomHeight, um, internal_units, param_map_cm)
    except:
        pass
    if top is None:
        top = read_named_param_expr_paramaware(op, ('topHeight_value','topHeight','zTop','top_height_value'), um, internal_units, param_map_cm)
    if bot is None:
        bot = read_named_param_expr_paramaware(op, ('bottomHeight_value','bottomHeight','zBottom','bottom_height_value'), um, internal_units, param_map_cm)
    return top, bot

def get_height_top_bottom_numeric_as_mm(op, um):
    top = None; bot = None
    try:
        h = getattr(op, 'heights', None)
        if h:
            top = read_cam_param_numeric_as_mm(h.topHeight, um)
            bot = read_cam_param_numeric_as_mm(h.bottomHeight, um)
    except:
        pass
    if top is None:
        top = read_named_param_numeric_as_mm(op, ('topHeight_value','topHeight','zTop','top_height_value'), um)
    if bot is None:
        bot = read_named_param_numeric_as_mm(op, ('bottomHeight_value','bottomHeight','zBottom','bottom_height_value'), um)
    return top, bot

def scan_depthlike_params_cm(op, um, param_map_cm):
    """
    Sweep all operation parameters; return a plausible overall depth (cm) if we find one.
    Heuristics:
      - Match names containing 'depth' (excludes 'stepDown', 'peckDepth' by taking the MAX that is <= 2× stock where possible)
      - Evaluate param-aware; unitless -> mm
    """
    params = getattr(op, 'parameters', None)
    if not params: return None
    best = None
    for i in range(params.count):
        p = params.item(i)
        name = getattr(p, 'name', '') or ''
        ln = name.lower()
        if 'depth' not in ln:
            continue
        # skip obvious incremental depths
        if 'peck' in ln or 'step' in ln or 'increment' in ln:
            continue
        v = read_param_to_cm_unitless_mm_paramaware(p, um, param_map_cm)
        if is_positive(v):
            best = v if (best is None or v > best) else best
    return best

def get_toolpath_depth_cm(op: adsk.cam.Operation, setup: adsk.cam.Setup, um, internal_units: str, top_cm_hint: float):
    try:
        if not getattr(op, 'hasToolpath', False):
            return None
        zmin_cm, _ = get_toolpath_minmax_z_cm(op, um, internal_units)
        if zmin_cm is None:
            return None
        topZ_cm = top_cm_hint if top_cm_hint is not None else get_setup_stock_top_cm(setup, um, internal_units)
        if topZ_cm is None:
            return None
        return abs(topZ_cm - zmin_cm)
    except:
        return None

def get_toolpath_minmax_z_cm(op: adsk.cam.Operation, um, internal_units: str):
    tp = None
    try:
        tp = getattr(op, 'toolpath', None)
    except:
        tp = None
    if not tp:
        return None, None

    try:
        bbox = getattr(tp, 'boundingBox', None) or getattr(tp, 'bbox', None)
        if bbox:
            min_pt = getattr(bbox, 'minPoint', None) or getattr(bbox, 'min', None)
            max_pt = getattr(bbox, 'maxPoint', None) or getattr(bbox, 'max', None)
            if min_pt and max_pt:
                zmin_cm = to_internal_cm(min_pt.z, um, internal_units)
                zmax_cm = to_internal_cm(max_pt.z, um, internal_units)
                return zmin_cm, zmax_cm
    except:
        pass

    zmin, zmax = None, None
    try:
        for coll_name in ('entities', 'paths', 'segments'):
            coll = getattr(tp, coll_name, None)
            if not coll: continue
            count = getattr(coll, 'count', 0)
            for i in range(count):
                ent = coll.item(i)
                for getter in ('points','getPoint','getPoints'):
                    try:
                        pts = getattr(ent, getter, None)
                        if callable(pts): pts = pts()
                        if pts is None: continue
                        for p in pts:
                            zcm = to_internal_cm(p.z, um, internal_units)
                            zmin = zcm if zmin is None else min(zmin, zcm)
                            zmax = zcm if zmax is None else max(zmax, zcm)
                    except:
                        continue
        if zmin is not None:
            return zmin, zmax
    except:
        pass
    return None, None

# ---------------- reconcile & sanity ----------------

def reconcile_all(chamfer_cm, drill_cm, param_expr_cm, param_mm_cm, sweep_cm, tp_cm, stock_thick_cm):
    """
    Return (depth_cm, src, corrected_flag).
    Priority after 10× reconciliation:
       chamfer > drill > param_expr > param_mm > sweep > toolpath
    """
    candidates = []
    if is_positive(chamfer_cm): candidates.append(['chamfer',    float(chamfer_cm)])
    if is_positive(drill_cm):   candidates.append(['drill',      float(drill_cm)])
    if is_positive(param_expr_cm): candidates.append(['param_expr', float(param_expr_cm)])
    if is_positive(param_mm_cm):   candidates.append(['param_mm',   float(param_mm_cm)])
    if is_positive(sweep_cm):      candidates.append(['sweep',      float(sweep_cm)])
    if is_positive(tp_cm):         candidates.append(['toolpath',   float(tp_cm)])

    corrected = False
    for i in range(len(candidates)):
        for j in range(i+1, len(candidates)):
            (si, vi), (sj, vj) = candidates[i], candidates[j]
            small, big = (vi, vj) if vi <= vj else (vj, vi)
            if looks_like_ten_x(big, small):
                big_corrected = big / 10.0
                if vi > vj: candidates[i][1] = big_corrected
                else:       candidates[j][1] = big_corrected
                corrected = True

    order = {'chamfer':0, 'drill':1, 'param_expr':2, 'param_mm':3, 'sweep':4, 'toolpath':5}
    candidates.sort(key=lambda x: (order.get(x[0], 99), x[1]))

    if candidates:
        best_src, best_val = candidates[0]
        best_val, corr2 = sanity_vs_stock(best_val, stock_thick_cm)
        corrected = corrected or corr2
        return best_val, best_src, corrected

    return None, None, False

# ---------------- parameter & unit utils ----------------

_UNITLESS_NUM_RE = re.compile(r'^\s*-?\d+(\.\d+)?\s*$')

def build_param_map_cm(op, um, internal_units: str):
    m = {}
    params = getattr(op, 'parameters', None)
    if not params: return m
    for i in range(params.count):
        p = params.item(i)
        name = getattr(p, 'name', None)
        if not isinstance(name, str) or not name: continue
        val = read_cam_param_expr_paramaware(p, um, internal_units, {})  # expr (no map to avoid recursion)
        if val is None:
            val = read_cam_param_numeric_as_mm(p, um)
        if val is not None and isinstance(val, (int,float)):
            m[name.lower()] = float(val)
    return m

def eval_expr_with_params_to_cm(expr: str, um, param_map_cm: dict, default_unit_when_unitless='mm') -> float:
    s = expr.strip()
    if _UNITLESS_NUM_RE.match(s):
        return float(um.evaluateExpression(f'{s} {default_unit_when_unitless}', 'cm'))
    names = sorted(param_map_cm.keys(), key=len, reverse=True)
    for nm in names:
        val_cm = param_map_cm[nm]
        pattern = re.compile(r'\b' + re.escape(nm) + r'\b', flags=re.IGNORECASE)
        s = pattern.sub(f'({val_cm} cm)', s)
    return float(um.evaluateExpression(s, 'cm'))

def read_param_to_cm_unitless_mm_paramaware(p, um, param_map_cm: dict):
    if not p: return None
    try:
        expr = getattr(p, 'expression', None)
        if isinstance(expr, str) and expr.strip():
            return eval_expr_with_params_to_cm(expr, um, param_map_cm, default_unit_when_unitless='mm')
    except:
        pass
    try:
        v = getattr(p, 'value', None); u = getattr(p, 'unit', None)
        if isinstance(v, (int,float)):
            if u: return float(um.evaluateExpression(f'{float(v)} {u}', 'cm'))
            else: return float(um.evaluateExpression(f'{float(v)} mm', 'cm'))
    except:
        pass
    return None

def read_cam_param_expr_paramaware(p, um, internal_units: str, param_map_cm: dict):
    if not p: return None
    try:
        expr = getattr(p, 'expression', None)
        if isinstance(expr, str) and expr.strip():
            return eval_expr_with_params_to_cm(expr, um, param_map_cm, default_unit_when_unitless='mm')
    except:
        pass
    try:
        v = getattr(p, 'value', None); u = getattr(p, 'unit', None)
        if isinstance(v, (int,float)):
            if u: return float(um.evaluateExpression(f'{float(v)} {u}', 'cm'))
            else: return float(um.evaluateExpression(f'{float(v)} {internal_units}', 'cm'))
    except:
        pass
    return None

def read_cam_param_numeric_as_mm(p, um):
    if not p: return None
    try:
        v = getattr(p, 'value', None); u = getattr(p, 'unit', None)
        if isinstance(v, (int,float)):
            if u: return float(um.evaluateExpression(f'{float(v)} {u}', 'cm'))
            else: return float(um.evaluateExpression(f'{float(v)} mm', 'cm'))
    except:
        pass
    try:
        expr = getattr(p, 'expression', None)
        if isinstance(expr, str) and expr.strip():
            return eval_expr_with_params_to_cm(expr, um, {}, default_unit_when_unitless='mm')
    except:
        pass
    return None

def read_named_param_expr_paramaware(op, names, um, internal_units: str, param_map_cm: dict):
    params = getattr(op, 'parameters', None)
    if not params: return None
    for n in names:
        p = params.itemByName(n)
        if p:
            v = read_cam_param_expr_paramaware(p, um, internal_units, param_map_cm)
            if v is not None: return v
    return None

def read_named_param_numeric_as_mm(op, names, um):
    params = getattr(op, 'parameters', None)
    if not params: return None
    for n in names:
        p = params.itemByName(n)
        if p:
            v = read_cam_param_numeric_as_mm(p, um)
            if v is not None: return v
    return None

def get_string_param(op, names):
    params = getattr(op, 'parameters', None)
    if not params: return None
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
    if not params: return None
    for n in names:
        p = params.itemByName(n)
        if p:
            val = getattr(p, 'value', None)
            if isinstance(val, (int,float)):
                return float(val)
            expr = getattr(p, 'expression', None)
            if isinstance(expr, str) and expr.strip():
                try: return float(expr)
                except: pass
    return None

# ---------------- stock & units ----------------

def get_setup_stock_top_cm(setup: adsk.cam.Setup, um, internal_units: str):
    try:
        ext = getattr(setup, 'stockExtents', None)
        if ext:
            max_pt = getattr(ext, 'maxPoint', None)
            if max_pt:
                return to_internal_cm(max_pt.z, um, internal_units)
    except:
        pass
    return None

def get_setup_stock_bottom_cm(setup: adsk.cam.Setup, um, internal_units: str):
    try:
        ext = getattr(setup, 'stockExtents', None)
        if ext:
            min_pt = getattr(ext, 'minPoint', None)
            if min_pt:
                return to_internal_cm(min_pt.z, um, internal_units)
    except:
        pass
    return None

def get_setup_stock_thickness_cm(setup: adsk.cam.Setup, um, internal_units: str):
    top = get_setup_stock_top_cm(setup, um, internal_units)
    bot = get_setup_stock_bottom_cm(setup, um, internal_units)
    if top is not None and bot is not None:
        try: return abs(top - bot)
        except: return None
    return None

def to_internal_cm(val_in_internal_units: float, um, internal_units: str) -> float:
    v = float(val_in_internal_units)
    try: return float(um.evaluateExpression(f'{v} {internal_units}', 'cm'))
    except: return v

def convert_units(value_internal: float, um, internal_units: str, display_units: str) -> float:
    try: return float(um.evaluateExpression(f'{value_internal} {internal_units}', display_units))
    except:
        if 'mm' in display_units: return value_internal * 10.0
        if 'in' in display_units: return value_internal / 2.54
        if 'cm' in display_units: return value_internal
        return value_internal

def format_depth(depth_val: float, display_units: str) -> str:
    unit = 'mm' if 'mm' in display_units else ('cm' if 'cm' in display_units else 'in')
    return f'{depth_val:.3f} {unit}'

# ---------------- misc helpers ----------------

def looks_like_ten_x(big, small):
    try:
        if small <= 0 or big <= 0: return False
        return (big >= 8.0 * small) and (abs(big / 10.0 - small) <= max(0.2 * small, 0.01))
    except:
        return False

def sanity_vs_stock(value_cm, stock_thick_cm):
    corrected = False
    try:
        if is_positive(stock_thick_cm) and is_positive(value_cm):
            if value_cm > 3.0 * stock_thick_cm and (value_cm / 10.0) <= 1.5 * stock_thick_cm:
                value_cm = value_cm / 10.0
                corrected = True
    except:
        pass
    return value_cm, corrected

def stop(context):
    pass
