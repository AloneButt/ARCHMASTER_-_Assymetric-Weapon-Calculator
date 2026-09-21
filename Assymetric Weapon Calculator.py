# Fusion 360 - Centre-of-mass balancer (single body)
#
# Drives one user parameter until the body's centre of mass lies on the
# centre of a sketch circle, measured in the sketch plane.
#
# Before solving it runs a pre-flight check: finds the sketch, body and target,
# reports the body thickness and extrude feature, and asks for confirmation.
#
# Thickness does not change the solution: for a constant-thickness extrusion the
# in-plane centre of mass equals the sketch-area centroid. It only sets where the
# centre of mass sits along the extrusion axis (mid-thickness), which does not
# affect balance about an axis perpendicular to the sketch.
#
# Root finder: Illinois (modified regula falsi), bracketed by an initial scan.

import adsk.core, adsk.fusion, traceback

APP = adsk.core.Application.get()
UI = APP.userInterface

# ============================ USER SETTINGS ============================

PARAM_NAME = 'RadiusS'

SKETCH_NAME = ''              # '' = the sketch with the most profiles
BODY_NAME = ''                # '' = the only solid body in the sketch's component

TARGET = 'circle'             # 'circle' = centre of a sketch circle, 'origin' = sketch origin
TARGET_CIRCLE_DIA_MM = 40.0   # which circle; None = the circle closest to the sketch origin

CONFIRM_BEFORE_RUN = True

# search range, in the parameter's display units (None = auto around current value)
R_MIN = None
R_MAX = None
RANGE_FRACTION = 0.45
SCAN_SAMPLES = 9

COM_TOL_MM = 1.0e-4           # stop when the in-plane offset is below this
R_TOL_FRACTION = 1.0e-7
MAX_EVALS = 40

RESTORE_ON_FAILURE = True

# =======================================================================

CM_TO_MM = 10.0
HIGH_ACC = adsk.fusion.CalculationAccuracy.HighCalculationAccuracy


def run(context):
    design = None
    param = None
    original_value = None
    try:
        design = adsk.fusion.Design.cast(APP.activeProduct)
        if not design:
            UI.messageBox('Open a Fusion design first.')
            return

        param = design.userParameters.itemByName(PARAM_NAME) or \
                design.allParameters.itemByName(PARAM_NAME)
        if not param:
            UI.messageBox('Parameter "{}" not found.'.format(PARAM_NAME))
            return

        original_value = param.value
        units = design.unitsManager

        def show(v):
            return units.formatInternalValue(v, param.unit, True)

        def to_internal(display_value):
            return units.evaluateExpression('{}'.format(display_value), param.unit)

        def set_param(v):
            param.value = v
            adsk.doEvents()
            if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
                design.computeAll()
            adsk.doEvents()

        # ---------------- lookup (re-run after every rebuild) ----------------

        def find_sketch():
            best, best_count = None, -1
            for comp in design.allComponents:
                for sk in comp.sketches:
                    if SKETCH_NAME:
                        if sk.name == SKETCH_NAME:
                            return sk
                    elif sk.profiles.count > best_count:
                        best, best_count = sk, sk.profiles.count
            if SKETCH_NAME:
                raise RuntimeError('Sketch "{}" not found.'.format(SKETCH_NAME))
            if not best:
                raise RuntimeError('No sketch found.')
            return best

        def find_body(sk):
            comp = sk.parentComponent
            solids = [b for b in comp.bRepBodies if b.isSolid]
            if BODY_NAME:
                for b in solids:
                    if b.name == BODY_NAME:
                        return b
                raise RuntimeError('Body "{}" not found in component "{}".'
                                   .format(BODY_NAME, comp.name))
            if len(solids) != 1:
                raise RuntimeError('Component "{}" has {} solid bodies ({}). Set BODY_NAME.'
                                   .format(comp.name, len(solids),
                                           ', '.join(b.name for b in solids) or 'none'))
            return solids[0]

        def target_point(sk):
            """Target in sketch space, mm."""
            if TARGET == 'origin':
                return 0.0, 0.0
            best, best_score = None, None
            for c in sk.sketchCurves.sketchCircles:
                g = c.centerSketchPoint.geometry
                if TARGET_CIRCLE_DIA_MM is not None:
                    score = abs(c.radius * 2.0 * CM_TO_MM - TARGET_CIRCLE_DIA_MM)
                else:
                    score = (g.x ** 2 + g.y ** 2) ** 0.5
                if best_score is None or score < best_score:
                    best, best_score = c, score
            if best is None:
                raise RuntimeError('Sketch "{}" has no circles.'.format(sk.name))
            if TARGET_CIRCLE_DIA_MM is not None and best_score > 0.01:
                raise RuntimeError('No circle of diameter {} mm in sketch "{}".'
                                   .format(TARGET_CIRCLE_DIA_MM, sk.name))
            g = best.centerSketchPoint.geometry
            return g.x * CM_TO_MM, g.y * CM_TO_MM

        def body_span(sk, body):
            """Extent of the body along the sketch normal, mm (sketch-space z)."""
            zs = [sk.modelToSketchSpace(v.geometry).z * CM_TO_MM for v in body.vertices]
            if not zs:
                raise RuntimeError('Body "{}" has no vertices to measure.'.format(body.name))
            return min(zs), max(zs)

        def extrude_info(sk, body):
            comp = sk.parentComponent
            for ef in comp.features.extrudeFeatures:
                try:
                    if not any(b.name == body.name for b in ef.bodies):
                        continue
                except Exception:
                    continue
                info = {'name': ef.name, 'type': 'unknown', 'expr': ''}
                try:
                    types = adsk.fusion.FeatureExtentTypes
                    info['type'] = {
                        types.OneSideFeatureExtentType: 'one side',
                        types.TwoSidesFeatureExtentType: 'two sides',
                        types.SymmetricFeatureExtentType: 'symmetric',
                    }.get(ef.extentType, 'other')
                except Exception:
                    pass
                try:
                    d = adsk.fusion.DistanceExtentDefinition.cast(ef.extentOne)
                    if d:
                        info['expr'] = d.distance.expression
                    s = adsk.fusion.SymmetricExtentDefinition.cast(ef.extentOne)
                    if s:
                        info['expr'] = s.distance.expression + \
                            (' (full length)' if s.isFullLength else ' (per side)')
                except Exception:
                    pass
                return info
            return None

        def measure():
            sk = find_sketch()
            body = find_body(sk)
            tx, ty = target_point(sk)
            if hasattr(body, 'getPhysicalProperties'):
                pp = body.getPhysicalProperties(HIGH_ACC)
            else:
                pp = body.physicalProperties
            com = pp.centerOfMass
            s = sk.modelToSketchSpace(com)
            return {
                'dx': s.x * CM_TO_MM - tx,
                'dy': s.y * CM_TO_MM - ty,
                'z': s.z * CM_TO_MM,
                'model': (com.x * CM_TO_MM, com.y * CM_TO_MM, com.z * CM_TO_MM),
                'mass_g': pp.mass * 1000.0,
                'volume_mm3': pp.volume * 1000.0,
                'body': body.name,
                'sketch': sk.name,
                'target': (tx, ty),
            }

        # ---------------- pre-flight ----------------

        sk = find_sketch()
        body = find_body(sk)
        tx, ty = target_point(sk)
        zmin, zmax = body_span(sk, body)
        thickness = zmax - zmin
        ext = extrude_info(sk, body)
        m0 = measure()

        pre = 'Pre-flight check\n'
        pre += '  Component : {}\n'.format(sk.parentComponent.name)
        pre += '  Sketch    : {}\n'.format(sk.name)
        pre += '  Body      : {}\n'.format(body.name)
        pre += '  Target    : sketch X {:+.4f}  Y {:+.4f} mm\n'.format(tx, ty)
        pre += '  Thickness : {:.4f} mm measured normal to the sketch\n'.format(thickness)
        pre += '              body spans {:+.4f} to {:+.4f} mm from the sketch plane\n'.format(zmin, zmax)
        if ext:
            pre += '  Extrude   : {} - {}{}\n'.format(
                ext['name'], ext['type'], ', ' + ext['expr'] if ext['expr'] else '')
        else:
            pre += '  Extrude   : feature not identified (reporting measured thickness only)\n'
        pre += '  {} now : {}\n'.format(PARAM_NAME, show(original_value))
        pre += '  COM offset now : dX {:+.4f}  dY {:+.4f} mm\n'.format(m0['dx'], m0['dy'])
        pre += '  Mass      : {:.2f} g\n'.format(m0['mass_g'])

        if CONFIRM_BEFORE_RUN:
            res = UI.messageBox(pre + '\nRun the balance solve?', 'Centre-of-mass balancer',
                                adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                                adsk.core.MessageBoxIconTypes.QuestionIconType)
            if res != adsk.core.DialogResults.DialogYes:
                return

        # ---------------- objective ----------------

        cache = {}

        def f(v):
            key = round(v, 12)
            if key not in cache:
                set_param(v)
                cache[key] = measure()['dy']
            return cache[key]

        # ---------------- bracket ----------------

        if R_MIN is None or R_MAX is None:
            span = abs(original_value) * RANGE_FRACTION or to_internal('10 mm')
            lo, hi = original_value - span, original_value + span
        else:
            lo, hi = to_internal(R_MIN), to_internal(R_MAX)
        if lo <= 0:
            lo = min(hi * 0.05, to_internal('0.1 mm'))

        samples = []
        for i in range(SCAN_SAMPLES):
            v = lo + (hi - lo) * i / (SCAN_SAMPLES - 1)
            try:
                samples.append((v, f(v)))
            except Exception:
                samples.append((v, None))

        scan_report = 'Scan of {} over [{}, {}]  (COM dY, mm):\n'.format(
            PARAM_NAME, show(lo), show(hi))
        for v, val in samples:
            scan_report += '  {:>12}  ->  {}\n'.format(
                show(v), 'rebuild failed' if val is None else '{:+.6f}'.format(val))

        valid = [s for s in samples if s[1] is not None]
        if len(valid) < 2:
            raise RuntimeError('The model failed to rebuild across the scanned range.')

        a = b = fa = fb = None
        for (v1, d1), (v2, d2) in zip(valid, valid[1:]):
            if d1 == 0.0 or d1 * d2 < 0.0:
                a, fa, b, fb = v1, d1, v2, d2
                break

        if a is None:
            if RESTORE_ON_FAILURE:
                set_param(original_value)
            UI.messageBox(scan_report +
                          '\nNo sign change in this range - the centre of mass never crosses '
                          'the target.\nWiden RANGE_FRACTION or set R_MIN / R_MAX.')
            return

        # ---------------- Illinois root finder ----------------

        r_tol = max(abs(original_value), 1e-6) * R_TOL_FRACTION
        side = 0
        root = a if fa == 0.0 else None

        while root is None and len(cache) < MAX_EVALS:
            c = 0.5 * (a + b) if fb == fa else b - fb * (b - a) / (fb - fa)
            span = abs(b - a)
            if not (min(a, b) + 0.01 * span <= c <= max(a, b) - 0.01 * span):
                c = 0.5 * (a + b)
            fc = f(c)

            if abs(fc) <= COM_TOL_MM or abs(b - a) <= r_tol:
                root = c
                break
            if fa * fc < 0.0:
                b, fb = c, fc
                if side == -1:
                    fa *= 0.5
                side = -1
            else:
                a, fa = c, fc
                if side == 1:
                    fb *= 0.5
                side = 1

        if root is None:
            root = 0.5 * (a + b)

        # ---------------- lock and report ----------------

        set_param(root)
        m = measure()
        mid = 0.5 * (zmin + zmax)
        converged = abs(m['dy']) <= COM_TOL_MM

        msg = scan_report + '\n'
        msg += ('Converged.' if converged else 'Stopped before full convergence.') + '\n'
        msg += '  {} = {}\n'.format(PARAM_NAME, show(root))
        msg += '  In-plane COM offset : dX {:+.6f}  dY {:+.6f} mm\n'.format(m['dx'], m['dy'])
        msg += '  Along extrusion     : {:+.4f} mm from sketch plane (mid-thickness {:+.4f})\n'.format(
            m['z'], mid)
        msg += '  Model COM (as in Properties): X {:+.4f}  Y {:+.4f}  Z {:+.4f} mm\n'.format(
            *m['model'])
        msg += '  Mass = {:.2f} g   Rebuilds = {}\n'.format(m['mass_g'], len(cache) + 1)

        if abs(m['z'] - mid) > 0.01:
            msg += ('\nNote: the centre of mass is not at mid-thickness, so the body is not a '
                    'plain constant-thickness extrusion (pockets, chamfers or one-sided features). '
                    'The in-plane result is still correct for the body as modelled.')
        if abs(m['dx']) > 10 * COM_TOL_MM:
            msg += ('\nWarning: X offset is not zero. The geometry is not symmetric about the '
                    'sketch Y axis, and {} alone cannot correct that.'.format(PARAM_NAME))

        UI.messageBox(msg)

    except:
        try:
            if RESTORE_ON_FAILURE and param and original_value is not None:
                param.value = original_value
                if design and design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
                    design.computeAll()
        except:
            pass
        if UI:
            UI.messageBox('Failed:\n{}'.format(traceback.format_exc()))