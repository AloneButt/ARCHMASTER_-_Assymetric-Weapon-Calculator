# Fusion 360 - Surface area equalizer
#
# Drives a user parameter until two regions have equal area.
#
# MODE 'profiles' (default): uses the closed profiles of a sketch, split by a
#   horizontal line. No bodies required. Profiles are re-found on every
#   iteration because they are invalidated by each rebuild.
# MODE 'bodies': uses the largest planar face of two named bodies.
#
# Root finder: Illinois (modified regula falsi). Bracketed like bisection but
# converges superlinearly, so ~8 rebuilds instead of ~160.

import adsk.core, adsk.fusion, traceback

APP = adsk.core.Application.get()
UI = APP.userInterface

# ============================ USER SETTINGS ============================

PARAM_NAME = 'RadiusS'      # parameter to solve for

MODE = 'profiles'           # 'profiles' or 'bodies'

# --- MODE 'profiles' ---
SKETCH_NAME = ''            # '' = auto-pick the sketch with the most profiles
SPLIT_Y_MM = 0.0            # sketch-space Y that separates top from bottom
PROFILE_PICK = 'SUM'        # was 'largest' = one profile per side, but due to the fact that there are now more than 2 profiles, switched tum 'SUM'
                            # 'sum'     = add up every profile on each side
MIN_PROFILE_AREA_MM2 = 1.0  # ignore slivers below this

# --- MODE 'bodies' ---
TOP_BODY_NAME = 'TopSurf'
BOT_BODY_NAME = 'BotSurf'

# --- search range, in the parameter's own display units ---
# Leave both None to auto-bracket at +/- RANGE_FRACTION around the current value.
R_MIN = None
R_MAX = None
RANGE_FRACTION = 0.45
SCAN_SAMPLES = 9            # samples used to locate a sign change

# --- convergence ---
AREA_TOL_MM2 = 1.0e-3       # stop when |top - bottom| is below this
R_TOL_FRACTION = 1.0e-6     # stop when the bracket is this fraction of the value
MAX_EVALS = 40

RESTORE_ON_FAILURE = True

# =======================================================================

CM2_TO_MM2 = 100.0          # Fusion internal area unit is cm^2
MM_TO_CM = 0.1

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

        param = design.userParameters.itemByName(PARAM_NAME)
        if not param:
            param = design.allParameters.itemByName(PARAM_NAME)
        if not param:
            UI.messageBox('Parameter "{}" not found.'.format(PARAM_NAME))
            return

        original_value = param.value          # internal units
        units = design.unitsManager

        def show(v_internal):
            return units.formatInternalValue(v_internal, param.unit, True)

        def to_internal(display_value):
            return units.evaluateExpression('{}'.format(display_value), param.unit)

        # ---------------- parameter driving ----------------

        def set_param(v_internal):
            param.value = v_internal
            adsk.doEvents()
            if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
                design.computeAll()
            adsk.doEvents()

        # ---------------- area measurement ----------------

        def find_sketch():
            best, best_count = None, -1
            for comp in design.allComponents:
                for sk in comp.sketches:
                    if SKETCH_NAME:
                        if sk.name == SKETCH_NAME:
                            return sk
                    else:
                        n = sk.profiles.count
                        if n > best_count:
                            best, best_count = sk, n
            if SKETCH_NAME:
                raise RuntimeError('Sketch "{}" not found.'.format(SKETCH_NAME))
            if not best or best_count < 2:
                raise RuntimeError('No sketch with at least two closed profiles was found.')
            return best

        def profile_areas():
            """Returns (top_mm2, bottom_mm2, detail_rows)."""
            sk = find_sketch()
            split_cm = SPLIT_Y_MM * MM_TO_CM
            tops, bots, rows = [], [], []

            for i in range(sk.profiles.count):
                prof = sk.profiles.item(i)
                props = prof.areaProperties(HIGH_ACC)
                area_mm2 = props.area * CM2_TO_MM2
                if area_mm2 < MIN_PROFILE_AREA_MM2:
                    continue
                # centroid comes back in model space -> convert to sketch space
                c = sk.modelToSketchSpace(props.centroid)
                rows.append((i, area_mm2, c.y / MM_TO_CM))
                if c.y > split_cm:
                    tops.append(area_mm2)
                else:
                    bots.append(area_mm2)

            if not tops or not bots:
                raise RuntimeError(
                    'Could not find profiles on both sides of Y = {} mm.\n'
                    'Sketch "{}" gave {} usable profile(s).'
                    .format(SPLIT_Y_MM, sk.name, len(rows)))

            if PROFILE_PICK == 'largest':
                return max(tops), max(bots), rows
            return sum(tops), sum(bots), rows

        def find_body(name):
            for comp in design.allComponents:
                for b in comp.bRepBodies:
                    if b.name == name:
                        return b
            return None

        def largest_planar_area_mm2(name):
            b = find_body(name)
            if not b:
                raise RuntimeError('Body "{}" not found.'.format(name))
            areas = []
            for f in b.faces:
                geom = f.geometry
                if geom and geom.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                    areas.append(f.area * CM2_TO_MM2)
            if not areas:
                raise RuntimeError('Body "{}" has no planar faces.'.format(name))
            return max(areas)

        def body_areas():
            return (largest_planar_area_mm2(TOP_BODY_NAME),
                    largest_planar_area_mm2(BOT_BODY_NAME),
                    [])

        measure = profile_areas if MODE == 'profiles' else body_areas

        # ---------------- objective ----------------

        cache = {}

        def f(v_internal):
            key = round(v_internal, 12)
            if key in cache:
                return cache[key]
            set_param(v_internal)
            top, bot, _ = measure()
            result = (top, bot, top - bot)
            cache[key] = result
            return result

        # ---------------- bracket ----------------

        if R_MIN is None or R_MAX is None:
            span = abs(original_value) * RANGE_FRACTION
            if span == 0.0:
                span = to_internal('10 mm')
            lo, hi = original_value - span, original_value + span
        else:
            lo, hi = to_internal(R_MIN), to_internal(R_MAX)

        if lo <= 0:
            lo = min(hi * 0.05, to_internal('0.1 mm'))

        samples = []
        for i in range(SCAN_SAMPLES):
            v = lo + (hi - lo) * i / (SCAN_SAMPLES - 1)
            try:
                top, bot, diff = f(v)
                samples.append((v, diff, top, bot))
            except Exception:
                samples.append((v, None, None, None))

        valid = [s for s in samples if s[1] is not None]
        if len(valid) < 2:
            raise RuntimeError('The model failed to rebuild across the scanned range.')

        scan_report = 'Scan of {} over [{}, {}]:\n'.format(
            PARAM_NAME, show(lo), show(hi))
        for v, diff, top, bot in samples:
            if diff is None:
                scan_report += '  {:>12}  ->  rebuild failed\n'.format(show(v))
            else:
                scan_report += '  {:>12}  ->  top {:10.4f}   bottom {:10.4f}   diff {:+10.4f}\n'.format(
                    show(v), top, bot, diff)

        a = b = None
        for i in range(len(valid) - 1):
            v1, d1 = valid[i][0], valid[i][1]
            v2, d2 = valid[i + 1][0], valid[i + 1][1]
            if d1 == 0.0:
                a, fa, b, fb = v1, d1, v1, d1
                break
            if d1 * d2 < 0.0:
                a, fa, b, fb = v1, d1, v2, d2
                break

        if a is None:
            if RESTORE_ON_FAILURE:
                set_param(original_value)
            UI.messageBox(
                scan_report +
                '\nNo sign change found - the areas never cross in this range.\n'
                'Widen RANGE_FRACTION or set R_MIN / R_MAX explicitly.')
            return

        # ---------------- Illinois root finder ----------------

        r_tol = max(abs(original_value), 1e-6) * R_TOL_FRACTION
        side = 0
        evals = len(cache)
        root, froot = None, None

        if fa == 0.0:
            root, froot = a, 0.0

        while root is None and evals < MAX_EVALS:
            if fb == fa:
                c = 0.5 * (a + b)
            else:
                c = b - fb * (b - a) / (fb - fa)
            span = abs(b - a)
            if not (min(a, b) + 0.01 * span <= c <= max(a, b) - 0.01 * span):
                c = 0.5 * (a + b)

            top, bot, fc = f(c)
            evals = len(cache)

            if abs(fc) <= AREA_TOL_MM2 or abs(b - a) <= r_tol:
                root, froot = c, fc
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
            top, bot, froot = f(root)

        # lock the model at the solution
        set_param(root)
        top, bot, diff = measure()[0], measure()[1], 0.0
        top, bot, _ = measure()
        diff = top - bot

        detail = ''
        if MODE == 'profiles':
            _, _, rows = profile_areas()
            detail = '\nProfiles at the solution (index, area mm^2, centroid Y mm):\n'
            for idx, area, cy in rows:
                detail += '  #{}  {:10.4f}  {:+9.3f}\n'.format(idx, area, cy)

        converged = abs(diff) <= AREA_TOL_MM2 or abs(b - a) <= r_tol
        header = 'Areas equalized.' if converged else 'Stopped before full convergence.'

        UI.messageBox(
            scan_report + '\n' + header + '\n'
            '  {} = {}\n'
            '  Top area    = {:.6f} mm^2\n'
            '  Bottom area = {:.6f} mm^2\n'
            '  Difference  = {:+.6f} mm^2\n'
            '  Rebuilds    = {}\n'
            '  Bracket     = {} wide'
            .format(PARAM_NAME, show(root), top, bot, diff,
                    len(cache), show(abs(b - a))) + detail)

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