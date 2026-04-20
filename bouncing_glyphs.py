#!/usr/bin/env python3
"""
bouncing_glyphs.py  --  Rigid-body physics simulation of font glyphs
=====================================================================

Extracts actual vector outlines from TrueType / OpenType fonts, computes each
glyph's convex hull, area, centroid, mass, and moment of inertia, then runs a
real-time 2-D rigid-body simulation with impulse-based collision response
(including rotational dynamics and Coulomb friction) -- all derived from each
glyph's specific shape.

Dependencies:
    pip install pygame-ce fonttools        (or: pip install pygame fonttools)

Usage:
    python bouncing_glyphs.py
    python bouncing_glyphs.py --font "C:/Windows/Fonts/impact.ttf" --count 25
    python bouncing_glyphs.py --gravity 500 --restitution 0.9
    python bouncing_glyphs.py --unicode --colorful --count 30
    python bouncing_glyphs.py --friction 0.3 --gravity 500    (realistic friction + gravity)
    python bouncing_glyphs.py --color "255,100,0" --font-size 100 --debug

Interactive keys:
    ESC / close window  --  quit
    SPACE               --  pause / resume
    D                   --  toggle convex-hull debug overlay
    G                   --  toggle gravity (0 <-> 500 px/s^2)
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys

import pygame
import pygame.freetype
from fontTools.ttLib import TTFont
from fontTools.pens.recordingPen import RecordingPen


# =====================================================================
#  2-D vector helpers  (pure tuples -- no numpy dependency)
# =====================================================================

def dot2(a, b):
    return a[0] * b[0] + a[1] * b[1]

def cross2(a, b):
    """Scalar 2-D cross product: a x b = ax*by - ay*bx."""
    return a[0] * b[1] - a[1] * b[0]

def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1])

def vscale(v, s):
    return (v[0] * s, v[1] * s)

def vlength(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])

def vnorm(v):
    L = vlength(v)
    return (v[0] / L, v[1] / L) if L > 1e-12 else (0.0, 0.0)

def vdist2(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return dx * dx + dy * dy


# =====================================================================
#  Bezier-curve flattening
# =====================================================================

_QS = 8    # sample count per quadratic segment
_CS = 10   # sample count per cubic segment

def _flat_quad(p0, p1, p2):
    """Flatten one quadratic Bezier into line-segment samples (excludes p0)."""
    out = []
    for i in range(1, _QS + 1):
        t = i / _QS
        u = 1.0 - t
        out.append((u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0],
                     u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1]))
    return out

def _flat_cubic(p0, p1, p2, p3):
    """Flatten one cubic Bezier into line-segment samples (excludes p0)."""
    out = []
    for i in range(1, _CS + 1):
        t = i / _CS
        u = 1.0 - t
        u2 = u * u
        t2 = t * t
        out.append((u2 * u * p0[0] + 3 * u2 * t * p1[0] + 3 * u * t2 * p2[0] + t2 * t * p3[0],
                     u2 * u * p0[1] + 3 * u2 * t * p1[1] + 3 * u * t2 * p2[1] + t2 * t * p3[1]))
    return out

def _flatten_qcurve(cur, args):
    """Flatten a fontTools *qCurveTo* (TrueType quadratic spline).

    TrueType allows multiple consecutive off-curve points; implied on-curve
    midpoints sit between each consecutive pair.  The final point in *args*
    is the true on-curve endpoint (or None for an implicit close).
    """
    if not args:
        return []
    a = list(args)
    if a[-1] is None:
        a[-1] = cur  # implicit close back to contour start
    if len(a) < 2:
        return [a[0]]
    off = a[:-1]
    end = a[-1]
    pts = []
    start = cur
    for i, ctrl in enumerate(off):
        if i < len(off) - 1:
            nxt = off[i + 1]
            on = ((ctrl[0] + nxt[0]) * 0.5, (ctrl[1] + nxt[1]) * 0.5)
        else:
            on = end
        pts.extend(_flat_quad(start, ctrl, on))
        start = on
    return pts

def _flatten_curve(cur, args):
    """Flatten a fontTools *curveTo* (PostScript cubic Bezier)."""
    if len(args) == 3:
        return _flat_cubic(cur, args[0], args[1], args[2])
    # Non-standard arity -- just keep the endpoint
    return [args[-1]] if args else []


# =====================================================================
#  Glyph-outline extraction  (via fontTools)
# =====================================================================

def _pen_to_points(ops, scale):
    """RecordingPen operations -> list[(x, y)] in pixel coords, Y-down."""
    pts = []
    cur = (0.0, 0.0)
    start = (0.0, 0.0)   # contour start (for None-endpoint handling)
    for op, args in ops:
        if op == "moveTo":
            cur = start = args[0]
            pts.append((cur[0] * scale, -cur[1] * scale))
        elif op == "lineTo":
            cur = args[0]
            pts.append((cur[0] * scale, -cur[1] * scale))
        elif op == "qCurveTo":
            a = list(args)
            if a[-1] is None:
                a[-1] = start
            for p in _flatten_qcurve(cur, a):
                pts.append((p[0] * scale, -p[1] * scale))
            cur = a[-1]
        elif op == "curveTo":
            for p in _flatten_curve(cur, args):
                pts.append((p[0] * scale, -p[1] * scale))
            cur = args[-1]
        # closePath / endPath -- nothing to record
    return pts


def load_outlines(font_path: str, chars: str | None, size: int) -> dict[str, list]:
    """Extract glyph outlines for characters from a font.

    *chars*: a string of characters to extract, or ``None`` to scan the
    font's entire cmap (all Unicode codepoints the font supports).

    Returns ``{char: [(x, y), ...]}`` in pixel coordinates (Y-down),
    scaled to *size* pixels-per-em.  Characters whose outlines are
    missing or degenerate (< 3 points) are silently skipped.
    """
    kw = {}
    if font_path.lower().endswith((".ttc", ".otc")):
        kw["fontNumber"] = 0
    try:
        tt = TTFont(font_path, **kw)
    except Exception as exc:
        print(f"fontTools cannot open {font_path}: {exc}", file=sys.stderr)
        return {}

    cmap = tt.getBestCmap()
    if cmap is None:
        tt.close()
        return {}
    gs = tt.getGlyphSet()
    sc = size / tt["head"].unitsPerEm

    # Build the list of (codepoint, glyph_name) pairs to try
    if chars is not None:
        pairs = [(ord(ch), ch) for ch in chars]
    else:
        pairs = [(code, chr(code)) for code in cmap]

    out = {}
    for code, ch in pairs:
        if code not in cmap:
            continue
        name = cmap[code]
        if name not in gs:
            continue
        pen = RecordingPen()
        try:
            gs[name].draw(pen)
        except Exception:
            continue
        pts = _pen_to_points(pen.value, sc)
        if len(pts) >= 3:
            out[ch] = pts
    tt.close()
    return out


# =====================================================================
#  Convex hull  (Andrew's monotone-chain algorithm, O(n log n))
# =====================================================================

def convex_hull(points):
    """Return the convex hull as a list of (x, y) tuples, CCW order."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return list(pts)
    lo = []
    for p in pts:
        while len(lo) >= 2 and cross2(vsub(lo[-1], lo[-2]), vsub(p, lo[-2])) <= 0:
            lo.pop()
        lo.append(p)
    hi = []
    for p in reversed(pts):
        while len(hi) >= 2 and cross2(vsub(hi[-1], hi[-2]), vsub(p, hi[-2])) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


# =====================================================================
#  Polygon physical properties  (area, centroid, moment of inertia)
# =====================================================================

DENSITY = 0.01   # mass = DENSITY * area   (px^2 -> mass-units)

def polygon_props(verts):
    """Compute physical properties for a simple polygon with uniform density.

    Returns ``(area, (cx, cy), moment_of_inertia, mass)``.

    The moment of inertia is about the centroid for uniform surface density
    ``DENSITY``.  The formula decomposes the polygon into triangular wedges
    and sums the second-moment contributions of each.
    """
    n = len(verts)
    if n < 3:
        cx = sum(v[0] for v in verts) / max(n, 1)
        cy = sum(v[1] for v in verts) / max(n, 1)
        return 1.0, (cx, cy), 1.0, DENSITY

    # Signed area and centroid (Green's theorem)
    sa = cx = cy = 0.0
    for i in range(n):
        j = (i + 1) % n
        d = verts[i][0] * verts[j][1] - verts[j][0] * verts[i][1]
        sa += d
        cx += (verts[i][0] + verts[j][0]) * d
        cy += (verts[i][1] + verts[j][1]) * d
    sa *= 0.5
    area = abs(sa)
    if area < 1e-8:
        cx = sum(v[0] for v in verts) / n
        cy = sum(v[1] for v in verts) / n
        return area, (cx, cy), 1.0, max(DENSITY * area, 0.01)
    cx /= 6.0 * sa
    cy /= 6.0 * sa

    # Second moments of area about the centroid
    Ixx = Iyy = 0.0
    for i in range(n):
        j = (i + 1) % n
        xi, yi = verts[i][0] - cx, verts[i][1] - cy
        xj, yj = verts[j][0] - cx, verts[j][1] - cy
        d = xi * yj - xj * yi
        Ixx += d * (yi * yi + yi * yj + yj * yj)
        Iyy += d * (xi * xi + xi * xj + xj * xj)
    Ixx /= 12.0
    Iyy /= 12.0

    mass = DENSITY * area
    # Polar moment of inertia = Ixx + Iyy, scaled by density
    inertia = DENSITY * abs(Ixx + Iyy)
    return area, (cx, cy), max(inertia, 0.01), mass


# =====================================================================
#  Rigid body
# =====================================================================

class Body:
    """A single glyph treated as a 2-D rigid body."""
    __slots__ = (
        "char", "px", "py", "vx", "vy", "angle", "omega",
        "mass", "inv_mass", "inertia", "inv_inertia",
        "local_verts", "radius",
        "base_surf", "color", "restitution",
    )

    def __init__(self, char, pos, vel, local_verts, mass, inertia,
                 base_surf, color, restitution):
        self.char = char
        self.px, self.py = pos
        self.vx, self.vy = vel
        self.angle = 0.0
        self.omega = 0.0
        self.mass = mass
        self.inv_mass = 1.0 / mass
        self.inertia = inertia
        self.inv_inertia = 1.0 / inertia
        self.local_verts = local_verts          # hull verts relative to centroid
        self.radius = max((vlength(v) for v in local_verts), default=0.0)
        self.base_surf = base_surf              # unrotated; centroid == surface centre
        self.color = color
        self.restitution = restitution

    def world_verts(self):
        """Return hull vertices transformed to world coordinates."""
        c = math.cos(self.angle)
        s = math.sin(self.angle)
        px, py = self.px, self.py
        return [(lx * c - ly * s + px,
                 lx * s + ly * c + py)
                for lx, ly in self.local_verts]


# =====================================================================
#  SAT collision detection  (convex-convex)
# =====================================================================

def _edge_normals(vs):
    """Outward-pointing unit normals for each edge of a convex polygon."""
    ns = []
    n = len(vs)
    for i in range(n):
        j = (i + 1) % n
        ex = vs[j][0] - vs[i][0]
        ey = vs[j][1] - vs[i][1]
        L = math.sqrt(ex * ex + ey * ey)
        if L > 1e-12:
            ns.append((-ey / L, ex / L))
    return ns


def _proj(vs, ax):
    """Project polygon onto axis -> (min, max)."""
    dots = [v[0] * ax[0] + v[1] * ax[1] for v in vs]
    return min(dots), max(dots)


def sat_test(va, vb):
    """Separating Axis Theorem overlap test for two convex polygons.

    Returns ``(colliding, normal, depth)`` where *normal* points from
    centroid-A toward centroid-B.
    """
    best_d = float("inf")
    best_ax = None
    for ax in _edge_normals(va) + _edge_normals(vb):
        la, ha = _proj(va, ax)
        lb, hb = _proj(vb, ax)
        ov = min(ha, hb) - max(la, lb)
        if ov <= 0:
            return False, None, 0.0
        if ov < best_d:
            best_d = ov
            best_ax = ax
    if best_ax is None:
        return False, None, 0.0
    # Orient normal so it points A -> B
    ca = (sum(v[0] for v in va) / len(va), sum(v[1] for v in va) / len(va))
    cb = (sum(v[0] for v in vb) / len(vb), sum(v[1] for v in vb) / len(vb))
    if dot2(vsub(cb, ca), best_ax) < 0:
        best_ax = (-best_ax[0], -best_ax[1])
    return True, best_ax, best_d


def _contact_pt(va, vb, n):
    """Approximate contact point between two overlapping convex hulls.

    We pick the deepest-penetrating vertex from each polygon and average
    them.  This is the standard single-point approximation used in many
    real-time physics engines.
    """
    # Vertex of A furthest along +n  (deepest into B)
    ba = max(va, key=lambda v: dot2(v, n))
    # Vertex of B furthest along -n  (deepest into A)
    bb = min(vb, key=lambda v: dot2(v, n))
    return ((ba[0] + bb[0]) * 0.5, (ba[1] + bb[1]) * 0.5)


# =====================================================================
#  Impulse-based collision response  (with rotational dynamics)
#
#  Core formula (rigid-body impulse in 2-D):
#
#    j = (1+e) * v_n  /  (1/m_a + 1/m_b + (r_a x n)^2/I_a + (r_b x n)^2/I_b)
#
#  where  r = contact - centre_of_mass,
#         v_rel = (v_a + w_a x r_a) - (v_b + w_b x r_b)  at the contact point,
#         v_n   = v_rel . n  (positive when approaching, since n points A->B),
#         j     = impulse magnitude (always >= 0).
#
#  Body A receives impulse  -j*n  (pushed away from B),
#  Body B receives impulse  +j*n  (pushed away from A).
#
#  After the normal impulse, Coulomb friction is applied along the
#  tangent with coefficient MU_FRICTION.
# =====================================================================

MU_FRICTION = 0.0    # Coulomb friction coefficient (set from CLI)


def _apply(body, impulse, r):
    """Apply a linear + angular impulse at lever arm *r* from the centroid."""
    body.vx += impulse[0] * body.inv_mass
    body.vy += impulse[1] * body.inv_mass
    body.omega += cross2(r, impulse) * body.inv_inertia


def _velocity_at(body, r):
    """Velocity at a point offset *r* from body centroid."""
    return (body.vx - body.omega * r[1],
            body.vy + body.omega * r[0])


def resolve_pair(a, b, n, depth, cp):
    """Full impulse-based collision response between two Bodies."""

    # --- Positional correction  (Baumgarte stabilisation) ---
    total_inv = a.inv_mass + b.inv_mass
    if total_inv < 1e-12:
        return
    slop = 0.5                # allow tiny overlap to reduce jitter
    percent = 0.6             # correction strength
    correction = max(depth - slop, 0.0) / total_inv * percent
    a.px -= n[0] * correction * a.inv_mass
    a.py -= n[1] * correction * a.inv_mass
    b.px += n[0] * correction * b.inv_mass
    b.py += n[1] * correction * b.inv_mass

    # --- Normal impulse ---
    ra = (cp[0] - a.px, cp[1] - a.py)
    rb = (cp[0] - b.px, cp[1] - b.py)

    v_rel = vsub(_velocity_at(a, ra), _velocity_at(b, rb))
    vn = dot2(v_rel, n)
    # n points A->B, v_rel = vA-vB, so vn > 0 means approaching.
    if vn < 0:     # already separating
        return

    e = min(a.restitution, b.restitution)
    rn_a = cross2(ra, n)
    rn_b = cross2(rb, n)
    denom = (total_inv
             + rn_a * rn_a * a.inv_inertia
             + rn_b * rn_b * b.inv_inertia)
    if abs(denom) < 1e-12:
        return
    j = (1.0 + e) * vn / denom          # positive impulse magnitude
    J = vscale(n, -j)                    # A gets pushed in -n (away from B)
    _apply(a, J, ra)
    _apply(b, vscale(J, -1), rb)         # B gets +j*n (away from A)

    # --- Tangential (friction) impulse ---
    v_rel = vsub(_velocity_at(a, ra), _velocity_at(b, rb))
    vt_vec = vsub(v_rel, vscale(n, dot2(v_rel, n)))
    vt_len = vlength(vt_vec)
    if vt_len < 1e-8:
        return
    tangent = vscale(vt_vec, -1.0 / vt_len)
    rt_a = cross2(ra, tangent)
    rt_b = cross2(rb, tangent)
    f_denom = (total_inv
               + rt_a * rt_a * a.inv_inertia
               + rt_b * rt_b * b.inv_inertia)
    if abs(f_denom) < 1e-12:
        return
    jt = vt_len / f_denom
    jt = min(jt, j * MU_FRICTION)        # Coulomb clamp
    Jt = vscale(tangent, jt)
    _apply(a, Jt, ra)
    _apply(b, vscale(Jt, -1), rb)


# =====================================================================
#  Wall (screen-edge) collision
# =====================================================================

def _resolve_wall(body, cp, n, depth):
    """Impulse response against an immovable wall (infinite mass, I=inf)."""
    body.px += n[0] * depth
    body.py += n[1] * depth

    r = (cp[0] - body.px, cp[1] - body.py)
    vc = _velocity_at(body, r)
    vn = dot2(vc, n)
    if vn >= 0:
        return

    e = body.restitution
    rn = cross2(r, n)
    denom = body.inv_mass + rn * rn * body.inv_inertia
    if abs(denom) < 1e-12:
        return
    j = -(1.0 + e) * vn / denom
    _apply(body, vscale(n, j), r)

    # Friction against wall
    vc = _velocity_at(body, r)
    vt_vec = vsub(vc, vscale(n, dot2(vc, n)))
    vt_len = vlength(vt_vec)
    if vt_len < 1e-8:
        return
    tangent = vscale(vt_vec, -1.0 / vt_len)
    rt = cross2(r, tangent)
    fd = body.inv_mass + rt * rt * body.inv_inertia
    if abs(fd) < 1e-12:
        return
    jt = min(vt_len / fd, j * MU_FRICTION)
    _apply(body, vscale(tangent, jt), r)


def handle_walls(body, W, H):
    """Detect and resolve wall penetration (deepest vertex per wall)."""
    verts = body.world_verts()
    # Left wall  (normal = +x)
    worst, cp = 0.0, None
    for v in verts:
        d = -v[0]
        if d > worst:
            worst, cp = d, v
    if cp:
        _resolve_wall(body, cp, (1.0, 0.0), worst)

    # Right wall  (normal = -x)
    worst, cp = 0.0, None
    for v in verts:
        d = v[0] - W
        if d > worst:
            worst, cp = d, v
    if cp:
        _resolve_wall(body, cp, (-1.0, 0.0), worst)

    # Top wall  (normal = +y)
    worst, cp = 0.0, None
    for v in verts:
        d = -v[1]
        if d > worst:
            worst, cp = d, v
    if cp:
        _resolve_wall(body, cp, (0.0, 1.0), worst)

    # Bottom wall  (normal = -y)
    worst, cp = 0.0, None
    for v in verts:
        d = v[1] - H
        if d > worst:
            worst, cp = d, v
    if cp:
        _resolve_wall(body, cp, (0.0, -1.0), worst)


# =====================================================================
#  Body factory
# =====================================================================

def _bright_color():
    """Random saturated colour via HSV  (S in 0.6-0.9, V=1)."""
    h = random.random()
    s = random.uniform(0.6, 0.9)
    v = 1.0
    i = int(h * 6)
    f = h * 6 - i
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    r, g, b = [(v, t, p), (q, v, p), (p, v, t),
               (p, q, v), (t, p, v), (v, p, q)][i % 6]
    return (int(r * 255), int(g * 255), int(b * 255))


def make_body(ch, outline, pg_font, pos, vel, restitution, color):
    """Build a Body: convex hull, physical properties, centred surface."""
    hull = convex_hull(outline)
    if len(hull) < 3:
        return None
    area, (cx, cy), inertia, mass = polygon_props(hull)
    # Hull vertices in local (centroid-relative) coordinates
    local = [(x - cx, y - cy) for x, y in hull]

    surf, rect = pg_font.render(ch, fgcolor=color)
    sw, sh = surf.get_width(), surf.get_height()
    if sw < 1 or sh < 1:
        return None

    # Map the physics centroid into the rendered-surface pixel grid.
    # rect.x = horizontal bearing (pen origin to surface left edge)
    # rect.y = ascent (baseline to surface top edge — positive upward)
    # Outline coords: pen origin at (0,0), Y-down.
    # Surface coords: top-left at (rect.x, -rect.y) relative to pen origin.
    #   surface_x = outline_x - rect.x
    #   surface_y = outline_y + rect.y
    csx = cx - rect.x      # centroid x within rendered surface
    csy = cy + rect.y      # centroid y within rendered surface

    # Build a square surface with the centroid exactly at the centre
    # so that pygame.transform.rotate pivots around the centroid.
    pad = 4
    hw = max(abs(csx), abs(sw - csx)) + pad
    hh = max(abs(csy), abs(sh - csy)) + pad
    half = int(math.ceil(max(hw, hh)))
    side = 2 * half
    base = pygame.Surface((side, side), pygame.SRCALPHA)
    bx = int(round(half - csx))
    by = int(round(half - csy))
    base.blit(surf, (bx, by))

    return Body(
        char=ch, pos=pos, vel=vel,
        local_verts=local, mass=mass, inertia=inertia,
        base_surf=base, color=color, restitution=restitution,
    )


# =====================================================================
#  Default-font search
# =====================================================================

def _find_font():
    """Try to locate a usable .ttf on the system."""
    pygame.font.init()
    for name in ("arial", "segoeui", "helvetica", "dejavusans",
                 "liberationsans", "freesans", "noto", "roboto"):
        p = pygame.font.match_font(name)
        if p and os.path.isfile(p):
            return p
    # Fall back to the font bundled with pygame
    p = os.path.join(os.path.dirname(pygame.__file__), "freesansbold.ttf")
    return p if os.path.isfile(p) else None


# =====================================================================
#  Main
# =====================================================================

DEFAULT_CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789&@#?!$*%+"


def main():
    ap = argparse.ArgumentParser(
        description="Bouncing Glyphs -- rigid-body font physics simulation")
    ap.add_argument("--font",        default=None, type=str,
                    help="Path to a .ttf / .otf font file (default: auto-detect)")
    ap.add_argument("--count",       default=15,   type=int,
                    help="Number of glyphs to spawn (default: 15)")
    ap.add_argument("--width",       default=1280, type=int,
                    help="Window width in pixels (default: 1280)")
    ap.add_argument("--height",      default=720,  type=int,
                    help="Window height in pixels (default: 720)")
    ap.add_argument("--speed-min",   default=100,  type=float,
                    help="Minimum initial speed in px/s (default: 100)")
    ap.add_argument("--speed-max",   default=400,  type=float,
                    help="Maximum initial speed in px/s (default: 400)")
    ap.add_argument("--font-size",   default=72,   type=int,
                    help="Font size in pixels (default: 72)")
    ap.add_argument("--gravity",     default=0,    type=float,
                    help="Downward gravity in px/s^2 (default: 0, try 500)")
    ap.add_argument("--restitution", default=1.0,  type=float,
                    help="Coefficient of restitution 0..1 (default: 1.0)")
    ap.add_argument("--friction",    default=0.0,  type=float,
                    help="Coulomb friction coefficient (default: 0 = frictionless; "
                         "try 0.3 for realistic sliding friction)")
    ap.add_argument("--chars",       default=DEFAULT_CHARS, type=str,
                    help="Character pool to draw from")
    ap.add_argument("--unicode",     action="store_true",
                    help="Pick from the font's entire Unicode range instead of --chars")
    ap.add_argument("--colorful",    action="store_true",
                    help="Random colour per glyph (default: all white)")
    ap.add_argument("--color",       default="white", type=str,
                    help="Glyph colour name or R,G,B (default: white)")
    ap.add_argument("--debug",       action="store_true",
                    help="Draw convex-hull wireframes")
    ap.add_argument("--seed",        default=None, type=int,
                    help="RNG seed for reproducible runs")
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    # ---- Apply friction coefficient globally ----
    global MU_FRICTION
    MU_FRICTION = args.friction

    # ---- Initialise pygame ----
    pygame.init()
    screen = pygame.display.set_mode((args.width, args.height))
    clock = pygame.time.Clock()

    # ---- Load font ----
    font_path = args.font or _find_font()
    if not font_path:
        sys.exit("Error: no font found.  Use --font to specify a .ttf file.")
    font_name = os.path.basename(font_path)
    pygame.display.set_caption(f"Bouncing Glyphs  [{font_name}]")
    print(f"Font : {font_path}")

    # ---- Extract glyph outlines (fontTools) ----
    char_source = None if args.unicode else args.chars
    outlines = load_outlines(font_path, char_source, args.font_size)
    if not outlines:
        sys.exit("Error: could not extract any glyph outlines from this font.")
    usable = list(outlines)
    preview = " ".join(usable[:40])
    print(f"Glyphs available : {len(usable)}  ({preview}{'...' if len(usable) > 40 else ''})")

    # ---- pygame.freetype font for surface rendering ----
    pg_font = pygame.freetype.Font(font_path, args.font_size)

    # ---- Parse base colour ----
    def _parse_color(s):
        """Parse 'R,G,B' or a pygame colour name."""
        if "," in s:
            parts = [int(x.strip()) for x in s.split(",")]
            return tuple(parts[:3])
        try:
            return pygame.Color(s)[:3]
        except ValueError:
            return (255, 255, 255)

    base_color = _parse_color(args.color)

    # ---- Spawn bodies ----
    bodies: list[Body] = []
    margin = args.font_size * 1.2
    attempts_per_body = 30
    for _ in range(args.count):
        ch = random.choice(usable)
        # Try to avoid spawning on top of existing bodies
        for _try in range(attempts_per_body):
            px = random.uniform(margin, max(args.width - margin, margin + 1))
            py = random.uniform(margin, max(args.height - margin, margin + 1))
            if all(vdist2((px, py), (o.px, o.py)) > (args.font_size * 0.8) ** 2
                   for o in bodies):
                break
        speed = random.uniform(args.speed_min, args.speed_max)
        ang = random.uniform(0, math.tau)
        color = _bright_color() if args.colorful else base_color
        b = make_body(ch, outlines[ch], pg_font,
                      (px, py),
                      (speed * math.cos(ang), speed * math.sin(ang)),
                      args.restitution, color)
        if b is not None:
            b.omega = random.uniform(-3.0, 3.0)
            bodies.append(b)
    print(f"Bodies spawned   : {len(bodies)}")
    print("Controls: ESC=quit  SPACE=pause  D=debug  G=gravity toggle")

    # ---- HUD font ----
    try:
        hud_font = pygame.freetype.SysFont("consolas,mono,monospace", 14)
    except Exception:
        hud_font = pygame.freetype.Font(None, 14)

    # ---- Main loop ----
    running = True
    paused = False
    debug = args.debug
    gravity = args.gravity

    while running:
        dt = min(clock.tick(60) / 1000.0, 1.0 / 30.0)   # cap large steps

        # ---- Events ----
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    paused = not paused
                elif ev.key == pygame.K_d:
                    debug = not debug
                elif ev.key == pygame.K_g:
                    gravity = 0.0 if gravity else 500.0

        if paused:
            # Still draw, but skip physics
            pass
        else:
            # ---- Integrate ----
            TAU = math.tau
            for b in bodies:
                b.vy += gravity * dt
                b.px += b.vx * dt
                b.py += b.vy * dt
                b.angle = (b.angle + b.omega * dt) % TAU

            # ---- Body-body collisions ----
            #   Broad phase: bounding-circle distance check
            #   Narrow phase: SAT on rotated convex hulls
            nb = len(bodies)
            wv = [b.world_verts() for b in bodies]   # cache world verts
            for i in range(nb):
                a = bodies[i]
                for j in range(i + 1, nb):
                    bb = bodies[j]
                    rr = a.radius + bb.radius
                    if vdist2((a.px, a.py), (bb.px, bb.py)) > rr * rr:
                        continue
                    hit, n, d = sat_test(wv[i], wv[j])
                    if hit:
                        cp = _contact_pt(wv[i], wv[j], n)
                        resolve_pair(a, bb, n, d, cp)

            # ---- Wall collisions (fresh verts after body-body resolution) ----
            for b in bodies:
                handle_walls(b, args.width, args.height)

        # ---- Render ----
        screen.fill((12, 12, 20))

        for b in bodies:
            # Rotate the pre-built surface (centroid is at surface centre)
            rot = pygame.transform.rotate(b.base_surf, -math.degrees(b.angle))
            rr = rot.get_rect(center=(int(b.px), int(b.py)))
            screen.blit(rot, rr)

            if debug:
                wv_b = b.world_verts()
                if len(wv_b) >= 3:
                    ipts = [(int(x), int(y)) for x, y in wv_b]
                    pygame.draw.polygon(screen, (255, 255, 0), ipts, 1)
                    # Draw centroid cross
                    cx, cy = int(b.px), int(b.py)
                    pygame.draw.line(screen, (255, 80, 80), (cx - 4, cy), (cx + 4, cy))
                    pygame.draw.line(screen, (255, 80, 80), (cx, cy - 4), (cx, cy + 4))

        # ---- HUD ----
        fps = clock.get_fps()
        status = "PAUSED" if paused else f"{fps:.0f} fps"
        hud = f"{status}  |  {len(bodies)} glyphs  |  gravity={'ON' if gravity else 'OFF'}"
        if debug:
            hud += "  |  DEBUG"
        hud_font.render_to(screen, (8, 6), hud, fgcolor=(90, 90, 110))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
