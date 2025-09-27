# RenameToolpaths_ToolDepthOp_FACE_bbox_cutspan.py
# Renames each op: "<Tool Description> | Depth: <depth from TOP> | <Operation Type>"
#
# New (Face fix):
# - For selection-like Top Height modes (selection/point/surface/face), if TopZ isn't exposed,
#   compute depth via toolpath bounding box + feed height:
#       topOfCut ≈ min(feedZ, bbox.zMax)   (or retractZ - ε if no feedZ)
#       depth    = topOfCut - bbox.zMin
# - Prioritize this source for Face operations.
#
# Also includes:
# - Param-aware expression evaluation (resolves named params in expressions)
# - Unitless numbers interpreted as mm (prevents 10× inflation)
# - Drill chamfer support (from diameter & point angle)
# - Selection-aware drill depths (avoid holeTop-derived params)
# - Toolpath autogen if missing
# - 10× reconciliation + sanity vs stock thickness
# - Safety skip if depth unknown

import adsk.core, adsk.cam, adsk.fusion, traceback, re, math, time

SHOW_DIAGNOSTICS = True
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
        internal_units = getattr(um, 'internalUnits', 'cm')  # Fusion CAM commonly 'cm'

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
                is_face   = op_type.lower() == 'face'

                # Per-op parameter map (name -> value in cm)
                param_map_cm = build_param_map_cm(op, um, internal_units)

                # Height mode (top) and whether it's selection-like
                raw_top_mode = get_height_mode(op, which='top')
                top_mode     = normalize_mode(raw_top_mode)
                sel_like     = is_selection_like(top_mode)

                # ---- Candidates (all cm) ----
                depth_chamfer_cm = get_chamfer_depth_cm(op, um, param_map_cm)                # chamfer drill
                depth_drill_cm   = get_drill_cycle_depth_cm(op, um, param_map_cm, top_mode)  # drill cycle

                # Heights: expression-first (param-aware, unitless→mm)
                top_expr_cm, bot_expr_cm = get_height_top_bottom_expr_paramaware(op, um, internal_units, param_map_cm)
                depth_param_expr_cm = abs(bot_expr_cm - top_expr_cm) if (top_expr_cm is not None and bot_expr_cm is not None) else None

                # Heights: numeric-as-mm fallback
                top_mm_cm, bot_mm_cm = get_height_top_bottom_numeric_as_mm(op, um)
                depth_param_mm_cm = abs(bot_mm_cm - top_mm_cm) if (top_mm_cm is not None and bot_mm_cm is not None) else None

                # Sweep any depth-like params (catch-all)
                depth_sweep_cm = scan_depthlike_params_cm(op, um, param_map_cm, top_mode)

                # Toolpath-based candidates
                maybe_generate_toolpath(op)
                zmin_cm, zmax_cm = get_toolpath_minmax_z_cm(op, um, internal_units)

                # Try absolute TopZ from params for selection-like
                topZ_sel_cm = None
                if sel_like:
                    topZ_sel_cm = get_topZ_from_params(op, um, internal_units, param_map_cm)
                if topZ_sel_cm is None:
                    topZ_sel_cm = top_expr_cm  # if expressions gave absolute

                depth_tp_from_top_cm = None
                if zmin_cm is not None and topZ_sel_cm is not None:
                    depth_tp_from_top_cm = abs(topZ_sel_cm - zmin_cm)

                # Face-safe cut-span via bbox + feed/retract (works even if TopZ hidden)
                depth_tp_cut_span_bbox_cm = None
                if zmin_cm is not None and zmax_cm is not None:
                    feedZ_cm    = get_feedZ_cm(op, um, internal_units, param_map_cm)
                    retractZ_cm = get_retractZ_cm(op, um, internal_units, param_map_cm)
                    eps = 1e-4
                    topCut_candidates = []
                    if feedZ_cm is not None:
                        topCut_candidates.append(min(feedZ_cm, zmax_cm))
                    if retractZ_cm is not None:
                        topCut_candidates.append(min(retractZ_cm - eps, zmax_cm))
                    if topCut_candidates:
                        topCut_cm = max(topCut_candidates)
                        if topCut_cm > zmin_cm:
                            depth_tp_cut_span_bbox_cm = abs(topCut_cm - zmin_cm)

                # "Plain" toolpath depth using stock/model top if we at least have that
                top_hint_cm = top_expr_cm
                depth_tp_cm = get_toolpath_depth_cm_given_top(op, setup, um, internal_units, top_hint_cm)

                # ---- Reconcile (with 10× detection) ----
                depth_cm, src, corrected = reconcile_all(
                    depth_chamfer_cm,
                    depth_drill_cm,
                    depth_param_expr_cm,
                    depth_param_mm_cm,
                    depth_sweep_cm,
                    depth_tp_from_top_cm,
                    depth_tp_cut_span_bbox_cm,
                    depth_tp_cm,
                    stock_thick_cm,
                    prefer_tp_from_top=(sel_like or is_face)
                )

                if not is_positive(depth_cm) and STRICT_NO_RENAME_IF_DEPTH_UNKNOWN:
                    skipped += 1
                    if SHOW_DIAGNOSTICS and len(diag_rows) < 60:
                        diag_rows.append(f"(SKIP) {op.name}\n  Tool=\"{tool_desc}\"  Depth=unknown  Type=\"{op_type}\"  src=None  top_mode={repr(top_mode)}")
                    continue

                depth_display = convert_units(depth_cm, um, internal_units, display_units)
                depth_str = format_depth(depth_display, display_units)
                new_name = f"{tool_desc} | Depth: {depth_str} | {op_type}"

                if op.name != new_name:
                    op.name = new_name
                    renamed.append(new_name)

                if SHOW_DIAGNOSTICS and len(diag_rows) < 60:
                    star = '*' if corrected else ''
                    hints = []
                    if depth_tp_from_top_cm is not None:       hints.append('tp_from_top')
                    if depth_tp_cut_span_bbox_cm is not None:  hints.append('tp_cut_span_bbox')
                    diag_rows.append(f"{op.name}\n  Tool=\"{tool_desc}\"  Depth={depth_str}  Type=\"{op_type}\"  src={src}{star}  top_mode={repr(top_mode)}  hints={','.join(hints)}")

        if SHOW_DIAGNOSTICS and diag_rows:
            ui.messageBox('Diagnostics:\n' + '\n'.join(diag_rows))

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
            t0 = time.time()
            while time.time() - t0 < GENERATE_TIMEOUT_SEC:
                adsk.doEvents()
                if getattr(op, 'hasToolpath', False):
                    break
                time.sleep(0.1)
    except:
        pass

# ---------------- tool / op type ----------------

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
    s = s.replace('twoD','2D').replace('TwoD','2D').replace('threeD','3D').replace('ThreeD','3D')
    out = []
    for i,ch in enumerate(s):
        if i and ch.isupper() and not s[i-1].isspace():
            out.append(' ')
        out.append(ch)
    s = ''.join(out).strip()
    fixes = {'Contour 2 D':'2D Contour','Pocket 2 D':'2D Pocket','Bore 2 D':'2D Bore','Slot 2 D':'2D Slot',
             'Parallel 3 D':'3D Parallel','Contour 3 D':'3D Contour','Pocket 3 D':'3D Pocket','Adaptive Clearing':'Adaptive'}
    return fixes.get(s, ' '.join(w.capitalize() for w in s.split()))

# ---------------- height mode helpers ----------------

def get_height_mode(op, which='top') -> str:
    names = (f'{which}Height_mode', f'{which}HeightMode', f'{which}_height_mode')
    params = getattr(op, 'parameters', None)
    if not params:
        return ''
    for n in names:
        p = params.itemByName(n)
        if not p:
            continue
        try:
            expr = getattr(p, 'expression', None)
            if isinstance(expr, str) and expr.strip():
                return str(expr).strip()
        except:
            pass
        try:
            v = getattr(p, 'value', None)
            if isinstance(v, (int,float)):
                iv = int(v)
                if iv == 3: return 'selection'
                if iv == 2: return 'holetop'
        except:
            pass
    return ''

def normalize_mode(mode_str: str) -> str:
    if not isinstance(mode_str, str):
        return ''
    s = mode_str.strip().strip('"').strip("'").lower()
    return s

def is_selection_like(mode_str: str) -> bool:
    s = normalize_mode(mode_str)
    tokens = ('selection', 'select', 'point', 'surface', 'face')
    return any(t in s for t in tokens)

# ---------------- TopZ (selection-like) ----------------

_TOPZ_CANDIDATE_NAMES = (
    'topHeight_value','topHeightValue','zTop','topZ','top_value','topValue',
    'topHeight_abs','topHeightAbsolute','topAbs','topHeight_world','topWorld'
)

def get_topZ_from_params(op, um, internal_units, param_map_cm):
    # 1) Try known names
    v = read_named_param_expr_paramaware(op, _TOPZ_CANDIDATE_NAMES, um, internal_units, param_map_cm)
    if is_positive_or_zero(v):
        return float(v)
    # 2) Fuzzy scan
    params = getattr(op, 'parameters', None)
    if not params:
        return None
    best = None
    for i in range(params.count):
        p = params.item(i)
        name = (getattr(p, 'name', '') or '').lower()
        if 'top' not in name:
            continue
        if ('value' not in name and 'z' not in name):
            continue
        if 'offset' in name or 'mode' in name:
            continue
        vcm = read_param_to_cm_unitless_mm_paramaware(p, um, param_map_cm)
        if vcm is not None:
            best = vcm
            if name.endswith('value') or name.endswith('z') or name.endswith('_value'):
                return float(vcm)
    return float(best) if best is not None else None

# ---------------- Drill chamfer ----------------

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

# ---------------- Drill cycle depth (selection-aware) ----------------

def get_drill_cycle_depth_cm(op: adsk.cam.Operation, um, param_map_cm: dict, top_mode: str):
    """Prefer total/cycle depth. If top is selection-like, avoid hole-top-based params."""
    prefer = ('totalDepth','drillDepth','finalDepth','targetDepth','depth')
    avoid_if_selection = ('holeDepth','hole_top_depth','holeTopDepth','zFromHoleTop')
    params = getattr(op, 'parameters', None)
    if not params:
        return None

    for n in prefer:
        p = params.itemByName(n)
        if p:
            v = read_param_to_cm_unitless_mm_paramaware(p, um, param_map_cm)
            if is_positive(v):
                return v

    if not is_selection_like(top_mode):
        for n in avoid_if_selection:
            p = params.itemByName(n)
            if p:
                v = read_param_to_cm_unitless_mm_paramaware(p, um, param_map_cm)
                if is_positive(v):
                    return v
    return None

# ---------------- Heights & toolpath depths ----------------

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

def get_toolpath_minmax_z_cm(op: adsk.cam.Operation, um, internal_units: str):
    tp = None
    try:
        tp = getattr(op, 'toolpath', None)
    except:
        tp = None
    if not tp:
        return None, None

    # Try bounding box first
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

    # Optional fallback: try to iterate points if exposed
    zmin, zmax = None, None
    try:
        zs = get_toolpath_all_z_cm(op, um, internal_units)
        if zs:
            for zcm in zs:
                zmin = zcm if zmin is None else min(zmin, zcm)
                zmax = zcm if zmax is None else max(zmax, zcm)
            if zmin is not None:
                return zmin, zmax
    except:
        pass
    return None, None

def get_toolpath_all_z_cm(op: adsk.cam.Operation, um, internal_units: str):
    """Optional: enumerate individual toolpath points if the API exposes them."""
    tp = None
    try:
        tp = getattr(op, 'toolpath', None)
    except:
        tp = None
    if not tp:
        return None
    zlist = []
    for coll_name in ('entities','paths','segments'):
        coll = getattr(tp, coll_name, None)
        if not coll:
            continue
        count = getattr(coll, 'count', 0)
        for i in range(count):
            ent = coll.item(i)
            for getter in ('points','getPoint','getPoints'):
                try:
                    pts = getattr(ent, getter, None)
                    if callable(pts): pts = pts()
                    if not pts: continue
                    for p in pts:
                        try:
                            zlist.append(to_internal_cm(p.z, um, internal_units))
                        except:
                            continue
                except:
                    continue
    return zlist if zlist else None

def get_toolpath_depth_cm_given_top(op: adsk.cam.Operation, setup: adsk.cam.Setup, um, internal_units: str, top_cm_hint: float):
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

# ---------------- Face/selection helpers ----------------

def get_feedZ_cm(op, um, internal_units, param_map_cm):
    h = getattr(op, 'heights', None)
    if h:
        v = read_cam_param_expr_paramaware(h.feedHeight, um, internal_units, param_map_cm)
        if v is not None:
            return float(v)
    # name-based fallback
    v = read_named_param_expr_paramaware(op, ('feedHeight','feed_height'), um, internal_units, param_map_cm)
    return float(v) if v is not None else None

def get_retractZ_cm(op, um, internal_units, param_map_cm):
    h = getattr(op, 'heights', None)
    if h:
        v = read_cam_param_expr_paramaware(h.retractHeight, um, internal_units, param_map_cm)
        if v is not None:
            return float(v)
        v = read_cam_param_expr_paramaware(h.clearanceHeight, um, internal_units, param_map_cm)
        if v is not None:
            return float(v)
    v = read_named_param_expr_paramaware(op, ('retractHeight','clearanceHeight','retract_height','clearance_height'), um, internal_units, param_map_cm)
    return float(v) if v is not None else None

# ---------------- depth-like sweep ----------------

def scan_depthlike_params_cm(op, um, param_map_cm, top_mode: str):
    params = getattr(op, 'parameters', None)
    if not params:
        return None
    best = None
    for i in range(params.count):
        p = params.item(i)
        name = getattr(p, 'name', '') or ''
        ln = name.lower()
        if 'depth' not in ln:
            continue
        if any(k in ln for k in ('peck','step','increment','pass','stepdown','stepover')):
            continue
        if is_selection_like(top_mode):
            if any(k in ln for k in ('hole', 'hole_top', 'holetop')):
                continue
        v = read_param_to_cm_unitless_mm_paramaware(p, um, param_map_cm)
        if is_positive(v):
            best = v if (best is None or v > best) else best
    return best

# ---------------- reconcile & sanity ----------------

def reconcile_all(chamfer_cm, drill_cm, param_expr_cm, param_mm_cm, sweep_cm, tp_from_top_cm, tp_cut_span_bbox_cm, tp_cm, stock_thick_cm, prefer_tp_from_top=False):
    """
    Return (depth_cm, src, corrected_flag).

    Default priority after 10× reconciliation:
      chamfer > drill > param_expr > param_mm > sweep > tp_from_top > tp_cut_span_bbox > toolpath

    If prefer_tp_from_top=True (selection-like top OR Face ops), priority becomes:
      chamfer > drill > tp_from_top > tp_cut_span_bbox > param_expr > param_mm > sweep > toolpath
    """
    base_order = {'chamfer':0, 'drill':1, 'param_expr':2, 'param_mm':3, 'sweep':4, 'tp_from_top':5, 'tp_cut_span_bbox':6, 'toolpath':7}
    sel_order  = {'chamfer':0, 'drill':1, 'tp_from_top':2, 'tp_cut_span_bbox':3, 'param_expr':4, 'param_mm':5, 'sweep':6, 'toolpath':7}
    order = sel_order if prefer_tp_from_top else base_order

    candidates = []
    if is_positive(chamfer_cm):            candidates.append(['chamfer',           float(chamfer_cm)])
    if is_positive(drill_cm):              candidates.append(['drill',             float(drill_cm)])
    if is_positive(param_expr_cm):         candidates.append(['param_expr',        float(param_expr_cm)])
    if is_positive(param_mm_cm):           candidates.append(['param_mm',          float(param_mm_cm)])
    if is_positive(sweep_cm):              candidates.append(['sweep',             float(sweep_cm)])
    if is_positive(tp_from_top_cm):        candidates.append(['tp_from_top',       float(tp_from_top_cm)])
    if is_positive(tp_cut_span_bbox_cm):   candidates.append(['tp_cut_span_bbox',  float(tp_cut_span_bbox_cm)])
    if is_positive(tp_cm):                 candidates.append(['toolpath',           float(tp_cm)])

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
    if not params:
        return m
    for i in range(params.count):
        p = params.item(i)
        name = getattr(p, 'name', None)
        if not isinstance(name, str) or not name:
            continue
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
    if not p:
        return None
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
    if not p:
        return None
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
    if not p:
        return None
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
    if not params:
        return None
    for n in names:
        p = params.itemByName(n)
        if p:
            v = read_cam_param_expr_paramaware(p, um, internal_units, param_map_cm)
            if v is not None:
                return v
    return None

def read_named_param_numeric_as_mm(op, names, um):
    params = getattr(op, 'parameters', None)
    if not params:
        return None
    for n in names:
        p = params.itemByName(n)
        if p:
            v = read_cam_param_numeric_as_mm(p, um)
            if v is not None:
                return v
    return None

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
        if 'mm' in display_units: return value_internal * 10.0   # cm -> mm
        if 'in' in display_units: return value_internal / 2.54   # cm -> in
        if 'cm' in display_units: return value_internal
        return value_internal

def format_depth(depth_val: float, display_units: str) -> str:
    unit = 'mm' if 'mm' in display_units else ('cm' if 'cm' in display_units else 'in')
    return f'{depth_val:.3f} {unit}'

# ---------------- sanity & misc ----------------

def is_positive(v) -> bool:
    try:
        return v is not None and float(v) > 0.0
    except:
        return False

def is_positive_or_zero(v) -> bool:
    try:
        return v is not None and float(v) >= 0.0
    except:
        return False

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
