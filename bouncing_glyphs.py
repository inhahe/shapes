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
    python bouncing_glyphs.py --goal --sound --gravity 400    (goal mode with sounds)
    python bouncing_glyphs.py --color "255,100,0" --font-size 100 --debug

Interactive keys:
    ESC / close window  --  quit
    SPACE               --  pause / resume
    D                   --  toggle convex-hull debug overlay
    G                   --  toggle gravity (0 <-> 500 px/s^2)
    S                   --  toggle collision sound (when --sound is available)

Interactive mouse:
    Left-click + drag   --  grab and fling any glyph; it collides with others
"""

from __future__ import annotations

import argparse
import array
import math
import os
import random
import sys
import time

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

def _pen_to_contours(ops, scale):
    """RecordingPen operations -> list of contours, each a list of (x, y).

    Preserves individual contours (outer boundaries and holes) so that
    mass / inertia can be computed from the true ink area.
    """
    contours = []
    cur_contour: list[tuple[float, float]] = []
    cur = (0.0, 0.0)
    start = (0.0, 0.0)

    def _finish_contour():
        if len(cur_contour) >= 3:
            contours.append(cur_contour[:])
        cur_contour.clear()

    for op, args in ops:
        if op == "moveTo":
            _finish_contour()
            cur = start = args[0]
            cur_contour.append((cur[0] * scale, -cur[1] * scale))
        elif op == "lineTo":
            cur = args[0]
            cur_contour.append((cur[0] * scale, -cur[1] * scale))
        elif op == "qCurveTo":
            a = list(args)
            if a[-1] is None:
                a[-1] = start
            for p in _flatten_qcurve(cur, a):
                cur_contour.append((p[0] * scale, -p[1] * scale))
            cur = a[-1]
        elif op == "curveTo":
            for p in _flatten_curve(cur, args):
                cur_contour.append((p[0] * scale, -p[1] * scale))
            cur = args[-1]
        elif op in ("closePath", "endPath"):
            _finish_contour()

    _finish_contour()   # in case the last contour had no explicit close
    return contours


def load_outlines(font_path: str, chars: str | None, size: int) -> dict[str, list]:
    """Extract glyph outlines for characters from a font.

    *chars*: a string of characters to extract, or ``None`` to scan the
    font's entire cmap (all Unicode codepoints the font supports).

    Returns ``{char: [contour, ...]}`` where each contour is a list of
    ``(x, y)`` tuples in pixel coordinates (Y-down), scaled to *size*
    pixels-per-em.  Separate contours are preserved so that mass and
    inertia can be computed from the true ink area (holes subtracted).
    Characters with no usable contours are silently skipped.
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
        contours = _pen_to_contours(pen.value, sc)
        if contours:
            out[ch] = contours
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


def _signed_area_of(contour):
    """Signed area of a single closed contour (shoelace formula)."""
    sa = 0.0
    n = len(contour)
    for i in range(n):
        j = (i + 1) % n
        sa += contour[i][0] * contour[j][1] - contour[j][0] * contour[i][1]
    return sa * 0.5


def _point_in_polygon(px, py, polygon):
    """Ray-casting point-in-polygon test."""
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and \
           (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _contour_nesting(contours):
    """Return the nesting depth of each contour (0 = outermost).

    Depth is the number of other contours that contain a sample point
    of this contour.  Even depth = filled region, odd depth = hole.
    """
    nc = len(contours)
    depths = [0] * nc
    for i in range(nc):
        pt = contours[i][0]
        for j in range(nc):
            if i != j and _point_in_polygon(pt[0], pt[1], contours[j]):
                depths[i] += 1
    return depths


def multi_contour_props(contours):
    """Physical properties for a glyph with arbitrarily nested contours.

    Determines each contour's nesting depth (how many other contours
    enclose it) and corrects its winding so that even-depth contours
    (filled regions) add material and odd-depth contours (holes)
    subtract it.  This handles all cases correctly:

    - Simple outlines (``I``, ``l``): depth 0, adds material.
    - Letters with holes (``O``, ``A``, ``B``): outer at depth 0,
      hole at depth 1 — hole subtracts.
    - Nested fills (bullseye, ``@``): solid inside a hole at depth 2
      — adds material back.

    Returns ``(area, (cx, cy), moment_of_inertia, mass)``.
    """
    if not contours:
        return 1.0, (0.0, 0.0), 1.0, DENSITY

    # ── Determine correction factor for each contour ──
    depths = _contour_nesting(contours)
    flips = []
    for k, contour in enumerate(contours):
        sa = _signed_area_of(contour)
        # Even depth = filled (want positive contribution)
        # Odd depth  = hole   (want negative contribution)
        want_positive = (depths[k] % 2 == 0)
        is_positive = (sa > 0)
        flips.append(1 if want_positive == is_positive else -1)

    # ── Accumulate Green's theorem sums with corrected winding ──
    total_sa = cx_num = cy_num = 0.0
    for k, contour in enumerate(contours):
        f = flips[k]
        n = len(contour)
        for i in range(n):
            j = (i + 1) % n
            d = (contour[i][0] * contour[j][1]
                 - contour[j][0] * contour[i][1]) * f
            total_sa += d
            cx_num += (contour[i][0] + contour[j][0]) * d
            cy_num += (contour[i][1] + contour[j][1]) * d

    total_sa *= 0.5
    area = abs(total_sa)
    if area < 1e-8:
        all_pts = [p for c in contours for p in c]
        nn = max(len(all_pts), 1)
        cx = sum(p[0] for p in all_pts) / nn
        cy = sum(p[1] for p in all_pts) / nn
        return max(area, 1.0), (cx, cy), 1.0, max(DENSITY * area, 0.01)

    cx = cx_num / (6.0 * total_sa)
    cy = cy_num / (6.0 * total_sa)

    # ── Second moments of area about the centroid ──
    Ixx = Iyy = 0.0
    for k, contour in enumerate(contours):
        f = flips[k]
        n = len(contour)
        for i in range(n):
            j = (i + 1) % n
            xi, yi = contour[i][0] - cx, contour[i][1] - cy
            xj, yj = contour[j][0] - cx, contour[j][1] - cy
            d = (xi * yj - xj * yi) * f
            Ixx += d * (yi * yi + yi * yj + yj * yj)
            Iyy += d * (xi * xi + xi * xj + xj * xj)
    Ixx /= 12.0
    Iyy /= 12.0

    mass = DENSITY * area
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


_BAUMGARTE_REF_DT = 1.0 / 60.0   # reference dt the constants were tuned for
_BAUMGARTE_SLOP   = 0.5          # overlap tolerance at reference dt
_BAUMGARTE_PERCENT = 0.6         # correction strength at reference dt


def resolve_pair(a, b, n, depth, cp, sdt=_BAUMGARTE_REF_DT):
    """Full impulse-based collision response between two Bodies.

    *sdt* is the substep duration, used to scale the Baumgarte positional
    correction so it behaves identically at any physics rate.

    Returns the normal impulse magnitude *j* (>= 0), or 0 if no
    impulse was applied (e.g. bodies already separating).
    """
    # --- Positional correction  (Baumgarte stabilisation) ---
    #     Scale slop and percent proportionally to dt so that the
    #     correction converges at the same real-time rate regardless
    #     of how many substeps we run.
    total_inv = a.inv_mass + b.inv_mass
    if total_inv < 1e-12:
        return 0.0
    dt_ratio = sdt / _BAUMGARTE_REF_DT
    slop = _BAUMGARTE_SLOP * dt_ratio
    percent = 1.0 - (1.0 - _BAUMGARTE_PERCENT) ** dt_ratio
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
        return 0.0

    e = min(a.restitution, b.restitution)
    rn_a = cross2(ra, n)
    rn_b = cross2(rb, n)
    denom = (total_inv
             + rn_a * rn_a * a.inv_inertia
             + rn_b * rn_b * b.inv_inertia)
    if abs(denom) < 1e-12:
        return 0.0
    j = (1.0 + e) * vn / denom          # positive impulse magnitude
    J = vscale(n, -j)                    # A gets pushed in -n (away from B)
    _apply(a, J, ra)
    _apply(b, vscale(J, -1), rb)         # B gets +j*n (away from A)

    # --- Tangential (friction) impulse ---
    v_rel = vsub(_velocity_at(a, ra), _velocity_at(b, rb))
    vt_vec = vsub(v_rel, vscale(n, dot2(v_rel, n)))
    vt_len = vlength(vt_vec)
    if vt_len < 1e-8:
        return j
    tangent = vscale(vt_vec, -1.0 / vt_len)
    rt_a = cross2(ra, tangent)
    rt_b = cross2(rb, tangent)
    f_denom = (total_inv
               + rt_a * rt_a * a.inv_inertia
               + rt_b * rt_b * b.inv_inertia)
    if abs(f_denom) < 1e-12:
        return j
    jt = vt_len / f_denom
    jt = min(jt, j * MU_FRICTION)        # Coulomb clamp
    Jt = vscale(tangent, jt)
    _apply(a, Jt, ra)
    _apply(b, vscale(Jt, -1), rb)
    return j


# =====================================================================
#  Wall (screen-edge) collision
# =====================================================================

def _resolve_wall(body, cp, n, depth):
    """Impulse response against an immovable wall (infinite mass, I=inf).

    Returns the normal impulse magnitude *j*, or 0.
    """
    body.px += n[0] * depth
    body.py += n[1] * depth

    r = (cp[0] - body.px, cp[1] - body.py)
    vc = _velocity_at(body, r)
    vn = dot2(vc, n)
    if vn >= 0:
        return 0.0

    e = body.restitution
    rn = cross2(r, n)
    denom = body.inv_mass + rn * rn * body.inv_inertia
    if abs(denom) < 1e-12:
        return 0.0
    j = -(1.0 + e) * vn / denom
    _apply(body, vscale(n, j), r)

    # Friction against wall
    vc = _velocity_at(body, r)
    vt_vec = vsub(vc, vscale(n, dot2(vc, n)))
    vt_len = vlength(vt_vec)
    if vt_len < 1e-8:
        return j
    tangent = vscale(vt_vec, -1.0 / vt_len)
    rt = cross2(r, tangent)
    fd = body.inv_mass + rt * rt * body.inv_inertia
    if abs(fd) < 1e-12:
        return j
    jt = min(vt_len / fd, j * MU_FRICTION)
    _apply(body, vscale(tangent, jt), r)
    return j


def handle_walls(body, W, H, goal=None):
    """Detect and resolve wall penetration (deepest vertex per wall).

    *goal*: ``None`` or ``(left_x, right_x)`` — a gap in the top wall
    where glyphs can pass through.

    Returns the largest wall impulse magnitude this frame (for sound).
    """
    verts = body.world_verts()
    max_j = 0.0

    # Left wall  (normal = +x)
    worst, cp = 0.0, None
    for v in verts:
        d = -v[0]
        if d > worst:
            worst, cp = d, v
    if cp:
        max_j = max(max_j, _resolve_wall(body, cp, (1.0, 0.0), worst))

    # Right wall  (normal = -x)
    worst, cp = 0.0, None
    for v in verts:
        d = v[0] - W
        if d > worst:
            worst, cp = d, v
    if cp:
        max_j = max(max_j, _resolve_wall(body, cp, (-1.0, 0.0), worst))

    # Top wall  (normal = +y) — skip vertices inside goal gap
    worst, cp = 0.0, None
    for v in verts:
        if goal and goal[0] <= v[0] <= goal[1]:
            continue   # gap — let it through
        d = -v[1]
        if d > worst:
            worst, cp = d, v
    if cp:
        max_j = max(max_j, _resolve_wall(body, cp, (0.0, 1.0), worst))

    # Goal posts — vertical wall segments at x=goal_left and x=goal_right,
    # extending from y=0 down to POST_H into the play area.  Which face
    # pushes the glyph depends on which side of the post the body centre is.
    if goal:
        gl, gr = goal
        POST_H = 40.0
        # Broad phase: skip bodies whose bounding circle is below the posts
        if body.py - body.radius < POST_H:
            # -- Left post at x = gl --
            worst, cp = 0.0, None
            if body.px < gl:
                # Body is left of goal → push stray vertices LEFT
                for v in verts:
                    if 0 < v[1] < POST_H:
                        d = v[0] - gl
                        if d > worst:
                            worst, cp = d, v
                if cp:
                    max_j = max(max_j,
                                _resolve_wall(body, cp, (-1.0, 0.0), worst))
            else:
                # Body is inside/right of goal → push stray vertices RIGHT
                for v in verts:
                    if 0 < v[1] < POST_H:
                        d = gl - v[0]
                        if d > worst:
                            worst, cp = d, v
                if cp:
                    max_j = max(max_j,
                                _resolve_wall(body, cp, (1.0, 0.0), worst))

            # -- Right post at x = gr --
            worst, cp = 0.0, None
            if body.px > gr:
                # Body is right of goal → push stray vertices RIGHT
                for v in verts:
                    if 0 < v[1] < POST_H:
                        d = gr - v[0]
                        if d > worst:
                            worst, cp = d, v
                if cp:
                    max_j = max(max_j,
                                _resolve_wall(body, cp, (1.0, 0.0), worst))
            else:
                # Body is inside/left of goal → push stray vertices LEFT
                for v in verts:
                    if 0 < v[1] < POST_H:
                        d = v[0] - gr
                        if d > worst:
                            worst, cp = d, v
                if cp:
                    max_j = max(max_j,
                                _resolve_wall(body, cp, (-1.0, 0.0), worst))

    # Bottom wall  (normal = -y)
    worst, cp = 0.0, None
    for v in verts:
        d = v[1] - H
        if d > worst:
            worst, cp = d, v
    if cp:
        max_j = max(max_j, _resolve_wall(body, cp, (0.0, -1.0), worst))

    return max_j


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


def make_body(ch, contours, pg_font, pos, vel, restitution, color):
    """Build a Body from glyph contours.

    *contours* is a list of contour polylines (as returned by
    ``load_outlines``).  Mass and inertia are computed from the true ink
    area (holes subtracted).  The convex hull of all points is used for
    collision detection.
    """
    all_pts = [p for c in contours for p in c]
    hull = convex_hull(all_pts)
    if len(hull) < 3:
        return None
    area, (cx, cy), inertia, mass = multi_contour_props(contours)
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
    ap.add_argument("--ascii",       action="store_true",
                    help="Use printable ASCII (codepoints 32-126)")
    ap.add_argument("--cp437",       action="store_true",
                    help="Use CP437 (DOS/BBS) character set (codepoints 1-255)")
    ap.add_argument("--colorful",    action="store_true",
                    help="Random colour per glyph (default: all white)")
    ap.add_argument("--color",       default="white", type=str,
                    help="Glyph colour name or R,G,B (default: white)")
    ap.add_argument("--sound",       action="store_true",
                    help="Play a click on every collision (volume ~ impact force)")
    ap.add_argument("--goal",        action="store_true",
                    help="Add a goal opening in the top wall. Glyphs that pass "
                         "through disappear and respawn as new random glyphs.")
    ap.add_argument("--goal-width",  default=150, type=int,
                    help="Width of the goal opening in pixels (default: 150)")
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

    # ---- Initialise pygame (with vsync) ----
    pygame.init()
    use_vsync = False
    try:
        screen = pygame.display.set_mode((args.width, args.height), vsync=1)
        use_vsync = True
    except Exception:
        screen = pygame.display.set_mode((args.width, args.height))
    clock = pygame.time.Clock()

    # ---- Sound ----
    click_sound = None
    goal_exit_sound = None
    goal_enter_sound = None
    want_sound = args.sound or args.goal   # goal implies sound for its events
    if want_sound:
        try:
            pygame.mixer.init(frequency=44100, size=-16, channels=1, buffer=512)
            pygame.mixer.set_num_channels(16)
            _sr = 44100

            # Collision click (~12 ms damped sine)
            _ns = int(_sr * 0.012)
            _buf = array.array("h", (
                int(8000 * math.exp(-i / _sr * 500) *
                    math.sin(2 * math.pi * 900 * i / _sr))
                for i in range(_ns)
            ))
            click_sound = pygame.mixer.Sound(buffer=_buf)

            # Goal exit: "cha-ching" — two quick ascending metallic tones
            _buf2 = array.array("h")
            for i in range(int(_sr * 0.05)):        # tone 1: 880 Hz, 50 ms
                t = i / _sr
                _buf2.append(int(6000 * math.exp(-t * 40) *
                                 math.sin(2 * math.pi * 880 * t)))
            for _ in range(int(_sr * 0.03)):         # 30 ms gap
                _buf2.append(0)
            for i in range(int(_sr * 0.07)):        # tone 2: 1320 Hz, 70 ms
                t = i / _sr
                _buf2.append(int(8000 * math.exp(-t * 30) *
                                 math.sin(2 * math.pi * 1320 * t)))
            goal_exit_sound = pygame.mixer.Sound(buffer=_buf2)
            goal_exit_sound.set_volume(0.6)

            # Goal enter: gentle rising "bwoop" (60 ms, 300 -> 700 Hz)
            _buf3 = array.array("h")
            _nd = int(_sr * 0.06)
            for i in range(_nd):
                t = i / _sr
                f = 300 + 400 * (i / _nd)
                _buf3.append(int(5000 * math.exp(-t * 35) *
                                 math.sin(2 * math.pi * f * t)))
            goal_enter_sound = pygame.mixer.Sound(buffer=_buf3)
            goal_enter_sound.set_volume(0.5)

        except Exception as exc:
            print(f"Sound init failed ({exc}); continuing without sound.")

    # ---- Load font ----
    font_path = args.font or _find_font()
    if not font_path:
        sys.exit("Error: no font found.  Use --font to specify a .ttf file.")
    font_name = os.path.basename(font_path)
    pygame.display.set_caption(f"Bouncing Glyphs  [{font_name}]")
    print(f"Font : {font_path}")

    # ---- Character set selection ----
    if args.unicode:
        char_source = None  # scan entire cmap
    elif args.cp437:
        char_source = bytes(range(1, 256)).decode("cp437")
    elif args.ascii:
        char_source = "".join(chr(c) for c in range(32, 127))
    else:
        char_source = args.chars
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

    # ---- Goal setup ----
    goal = None
    respawn_queue: list[list] = []   # [[delay, vx, vy, omega], ...]
    if args.goal:
        gl = args.width / 2 - args.goal_width / 2
        gr = args.width / 2 + args.goal_width / 2
        goal = (gl, gr)
        print(f"Goal             : top wall, x=[{gl:.0f}, {gr:.0f}] "
              f"({args.goal_width}px wide)")

    vsync_str = "vsync" if use_vsync else "60fps"
    sound_str = "  S=sound toggle" if click_sound else ""
    print(f"Display          : {args.width}x{args.height} {vsync_str}")
    print(f"Physics          : continuous (variable dt, as fast as CPU allows)")
    print(f"Controls: ESC=quit  SPACE=pause  D=debug  G=gravity  LMB=drag{sound_str}")

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
    goal_score = 0

    # ---- Mouse drag state ----
    dragged_body = None
    drag_offset_x = 0.0
    drag_offset_y = 0.0
    drag_vel_x = 0.0
    drag_vel_y = 0.0

    # ---- Physics timing ----
    last_phys = time.perf_counter()

    def _physics_step(sdt):
        """One fixed-timestep physics substep."""
        nonlocal goal_score

        # ---- Integrate ----
        TAU = math.tau
        for b in bodies:
            if b is dragged_body:
                continue
            b.vy += gravity * sdt
            b.px += b.vx * sdt
            b.py += b.vy * sdt
            b.angle = (b.angle + b.omega * sdt) % TAU

        # ---- Body-body collisions ----
        #   Broad phase: bounding-circle distance check
        #   Narrow phase: SAT on rotated convex hulls
        nb = len(bodies)
        wv = [b.world_verts() for b in bodies]
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
                    imp = resolve_pair(a, bb, n, d, cp, sdt)
                    if imp > 0 and click_sound and args.sound:
                        vol = min(1.0, max(0.05, math.log1p(imp) / 7.0))
                        click_sound.set_volume(vol)
                        click_sound.play()

        # ---- Wall collisions (fresh verts after body-body resolution) ----
        for b in bodies:
            if b is dragged_body:
                continue
            wj = handle_walls(b, args.width, args.height, goal)
            if wj > 0 and click_sound and args.sound:
                vol = min(1.0, max(0.05, math.log1p(wj) / 7.0))
                click_sound.set_volume(vol)
                click_sound.play()

        # ---- Goal: exit detection ----
        if goal:
            gl, gr = goal
            exited = [i for i, b in enumerate(bodies)
                      if b is not dragged_body and gl < b.px < gr and b.py < 0]
            for i in reversed(exited):
                b = bodies.pop(i)
                respawn_queue.append([random.uniform(1.0, 4.0),
                                     b.vx, b.vy, b.omega,
                                     b.mass, b.inertia])
                goal_score += 1
                if goal_exit_sound:
                    goal_exit_sound.play()

        # ---- Goal: respawn queue ----
        if respawn_queue:
            for entry in respawn_queue[:]:
                entry[0] -= sdt
                if entry[0] <= 0:
                    respawn_queue.remove(entry)
                    _, evx, evy, eomega, emass, einertia = entry
                    ch = random.choice(usable)
                    gx = random.uniform(goal[0] + 20, goal[1] - 20)
                    color = _bright_color() if args.colorful else base_color
                    newb = make_body(ch, outlines[ch], pg_font,
                                     (gx, args.font_size * 0.6),
                                     (0, 0),
                                     args.restitution, color)
                    if newb is not None:
                        vs = math.sqrt(emass / newb.mass)
                        ws = math.sqrt(einertia / newb.inertia)
                        vy_in = abs(evy) if evy < 0 else evy
                        newb.vx = evx * vs
                        newb.vy = vy_in * vs
                        newb.omega = -eomega * ws
                        bodies.append(newb)
                        if goal_enter_sound:
                            goal_enter_sound.play()

    while running:
        # vsync: flip() already blocked, just measure dt.
        # no vsync: cap at 60 fps via clock.tick.
        if use_vsync:
            dt = min(clock.tick() / 1000.0, 1.0 / 30.0)
        else:
            dt = min(clock.tick(60) / 1000.0, 1.0 / 30.0)
        frame_start = time.perf_counter()

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
                elif ev.key == pygame.K_s:
                    if click_sound:
                        args.sound = not args.sound
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                mx, my = ev.pos
                # Check bodies in reverse order (topmost rendered last)
                for idx in range(len(bodies) - 1, -1, -1):
                    b = bodies[idx]
                    wv_b = b.world_verts()
                    xs = [v[0] for v in wv_b]
                    ys = [v[1] for v in wv_b]
                    if min(xs) <= mx <= max(xs) and min(ys) <= my <= max(ys):
                        dragged_body = b
                        drag_offset_x = b.px - mx
                        drag_offset_y = b.py - my
                        drag_vel_x = b.vx
                        drag_vel_y = b.vy
                        # Move to end so it renders on top
                        bodies.pop(idx)
                        bodies.append(b)
                        # Infinite mass while dragging -- other bodies
                        # bounce off but the dragged body stays put
                        b.inv_mass = 0.0
                        b.inv_inertia = 0.0
                        break
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                if dragged_body is not None:
                    # Restore real mass
                    dragged_body.inv_mass = 1.0 / dragged_body.mass
                    dragged_body.inv_inertia = 1.0 / dragged_body.inertia
                    # Cap release speed to prevent physics explosions
                    spd = math.sqrt(drag_vel_x ** 2 + drag_vel_y ** 2)
                    max_spd = 2000.0
                    if spd > max_spd:
                        s = max_spd / spd
                        drag_vel_x *= s
                        drag_vel_y *= s
                    dragged_body.vx = drag_vel_x
                    dragged_body.vy = drag_vel_y
                    dragged_body = None

        # ---- Update dragged body position (runs even when paused) ----
        if dragged_body is not None:
            # Safety: release if body was removed (goal exit) or mouse
            # button released while window was unfocused
            if dragged_body not in bodies or not pygame.mouse.get_pressed()[0]:
                if dragged_body in bodies:
                    dragged_body.inv_mass = 1.0 / dragged_body.mass
                    dragged_body.inv_inertia = 1.0 / dragged_body.inertia
                    dragged_body.vx = drag_vel_x
                    dragged_body.vy = drag_vel_y
                dragged_body = None
            else:
                mx, my = pygame.mouse.get_pos()
                new_px = mx + drag_offset_x
                new_py = my + drag_offset_y
                if dt > 1e-6:
                    inst_vx = (new_px - dragged_body.px) / dt
                    inst_vy = (new_py - dragged_body.py) / dt
                    # Time-based exponential smoothing (~100ms window)
                    alpha = min(1.0, dt / 0.1)
                    drag_vel_x = drag_vel_x * (1.0 - alpha) + inst_vx * alpha
                    drag_vel_y = drag_vel_y * (1.0 - alpha) + inst_vy * alpha
                dragged_body.px = new_px
                dragged_body.py = new_py
                # Set velocity so collision response uses the drag speed
                dragged_body.vx = drag_vel_x
                dragged_body.vy = drag_vel_y
                dragged_body.omega = 0.0

        # ---- Render (before physics so we know exactly how much
        #      time remains -- no estimation or prediction needed) ----
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

        # ---- Goal rendering ----
        if goal:
            gl, gr = goal
            post_h = 35   # visual post height (cosmetic only)
            # Subtle glow in the goal opening
            glow = pygame.Surface((int(gr - gl), 4), pygame.SRCALPHA)
            glow.fill((80, 180, 80, 40))
            screen.blit(glow, (int(gl), 0))
            # Posts
            pygame.draw.line(screen, (80, 200, 80),
                             (int(gl), 0), (int(gl), post_h), 3)
            pygame.draw.line(screen, (80, 200, 80),
                             (int(gr), 0), (int(gr), post_h), 3)
            # Waiting indicator: dots for queued respawns
            for qi in range(len(respawn_queue)):
                dx = int(gl + (gr - gl) * (qi + 1) / (len(respawn_queue) + 1))
                pygame.draw.circle(screen, (80, 200, 80), (dx, 12), 3)

        # ---- HUD ----
        fps = clock.get_fps()
        status = "PAUSED" if paused else f"{fps:.0f} fps"
        hud = f"{status}  |  {len(bodies)} glyphs  |  gravity={'ON' if gravity else 'OFF'}"
        if goal:
            hud += f"  |  goals: {goal_score}  queue: {len(respawn_queue)}"
        if debug:
            hud += "  |  DEBUG"
        hud_font.render_to(screen, (8, 6), hud, fgcolor=(90, 90, 110))

        # ---- Cursor feedback for draggable bodies ----
        if dragged_body is not None:
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_SIZEALL)
        else:
            mx, my = pygame.mouse.get_pos()
            hover = False
            for b in reversed(bodies):
                wv_b = b.world_verts()
                xs = [v[0] for v in wv_b]
                ys = [v[1] for v in wv_b]
                if min(xs) <= mx <= max(xs) and min(ys) <= my <= max(ys):
                    hover = True
                    break
            pygame.mouse.set_cursor(
                pygame.SYSTEM_CURSOR_HAND if hover
                else pygame.SYSTEM_CURSOR_ARROW)

        # ---- Physics (fills all remaining time before vsync) ----
        if paused:
            # Reset the physics clock so unpause doesn't see a huge dt.
            last_phys = time.perf_counter()
        else:
            # Continuous variable-dt physics: rendering is already done,
            # so we know exactly how much time is left in this frame.
            # Run as many steps as the CPU can fit.  Each step uses the
            # real nanosecond-precision elapsed time since the last step.
            # The first step each frame naturally absorbs the render/flip
            # gap from the previous frame, keeping sim-time synchronised
            # with wall-clock time.  The budget is capped to one vsync
            # interval so a single missed vsync can't cascade.
            #
            # The 2 ms margin covers two things that sit between the
            # vsync signal and our flip() call:
            #   - frame_start is stamped AFTER the vsync (flip-return
            #     + clock.tick + perf_counter), so frame_start + budget
            #     overshoots the next vsync by that gap (~0.1-1 ms).
            #   - The physics loop can overshoot by up to one step
            #     (a step that passes the deadline check but finishes
            #     after it, ~0.1-0.5 ms in CPython).
            frame_budget = min(dt, 1.0 / 60.0)
            phys_deadline = frame_start + frame_budget - 0.002
            while time.perf_counter() < phys_deadline:
                now = time.perf_counter()
                sdt = now - last_phys
                last_phys = now
                if sdt > 1e-7:
                    # Cap individual steps so fast/spinning bodies
                    # don't penetrate deeply before collision fires.
                    # At 2000 px/s and 4 ms, max movement = 8 px —
                    # well within the smallest glyph radius.
                    while sdt > 0.004:
                        _physics_step(0.004)
                        sdt -= 0.004
                    if sdt > 1e-7:
                        _physics_step(sdt)

        pygame.display.flip()                  # vsync wait

    pygame.quit()


if __name__ == "__main__":
    main()
