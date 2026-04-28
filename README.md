# Bouncing Glyphs

Rigid-body physics simulation of font glyphs in the browser. Rasterizes
characters, traces their silhouette contours via marching squares, decomposes
them into convex parts (Bayazit algorithm with hole bridging), then simulates
them as rigid bodies bouncing off each other and the walls -- in both 2D and 3D.

All collision math -- impulse magnitudes, rotational velocity changes, friction
-- is derived from each glyph's specific shape, not a bounding circle or box.
Mass and inertia are computed from the true ink area (holes in letters like O,
A, B are subtracted, nested fills are added back).

## Live demo

**<http://inhahe.com/bouncing_glyphs.html>**

## Quick start

Open `bouncing_glyphs.html` in any modern browser. No build step, no
dependencies, no server required -- it's a single self-contained HTML file.

Click **Start** to launch the simulation. The default glyph count scales
automatically to your screen size.

## Features

- **2D and 3D modes** -- switch freely via the sidebar toggle
- **Accurate shape collision** -- convex decomposition (Bayazit) gives precise
  body-body collisions using the actual glyph outline, not a bounding hull
- **WebGL 3D rendering** -- extruded glyphs with Phong lighting, smooth vertex
  normals on curved contour edges, textured cap quads with alpha discard for
  correct holes (O, @, B, etc.)
- **Stereo 3D** -- anaglyph (red/cyan), cross-eyed, and parallel (wall-eyed)
  modes with configurable IPD, screen distance, and DPI
- **Inter-glyph attraction** -- optional gravitational pull between glyphs,
  with configurable strength and choice of 1/r (2-D law) or 1/r² (3-D law)
  distance falloff
- **Goal mode** -- glyphs exit through a goal opening and respawn as new
  characters after a random delay
- **Collision sound** -- impact-proportional click sounds via Web Audio
- **Mobile-friendly** -- responsive layout with slide-out control panel and full
  touch support (tap + drag to grab and fling glyphs)
- **Configurable everything** -- physics, appearance, font, characters, and 3D
  settings all adjustable from the sidebar

## Controls

### Keyboard

| Key | Action |
|---|---|
| **Space** | Pause / resume |
| **D** | Toggle debug overlay (convex decomposition shapes + centroids) |
| **G** | Toggle gravity on/off (0 or 500 px/s^2) |
| **S** | Toggle collision sound on/off |
| **A** | Toggle inter-glyph attraction on/off |

### Mouse / Touch

| Action | Effect |
|---|---|
| **Click/tap + drag** | Grab a glyph and drag it around. While held, the glyph has infinite mass -- other glyphs bounce off it based on your drag speed. |
| **Release** | Fling the glyph with the velocity you were dragging at (smoothed, capped at 2000 px/s). |

Dragging works even while paused -- reposition glyphs and they launch with your
drag velocity when you unpause.

## Settings

All settings are in the sidebar (on mobile, tap the ☰ button).

### Physics

| Setting | Default | Description |
|---|---|---|
| Count | auto | Number of glyphs. Auto-scales to screen area (2D) or room volume (3D). Manually setting it disables auto-scaling on Reset. |
| Init. Speed Min/Max | 100 / 400 | Initial speed range (px/s). Each glyph gets a random speed at a random angle. |
| Font Size | 72 | Em height in pixels. Larger = heavier glyphs with more inertia. |
| Gravity | 0 | Downward acceleration (px/s^2). Try 500 for a tumbling effect. |
| Restitution | 1.0 | Bounciness. 1.0 = perfectly elastic, 0.0 = perfectly inelastic. |
| Friction | 0.0 | Coulomb friction coefficient. 0 = frictionless, 0.3 = realistic. |
| Attraction | 0 | Inter-glyph gravitational constant G. 0 = off, try 5e4 for visible pull. Glyphs attract each other proportional to their masses. |
| Gravity Falloff | square | Distance law for attraction: `linear` (1/r, correct for a 2-D universe) or `square` (1/r², 3-D-style inverse-square law). |

### Font

Choose a system font from the dropdown, type any installed font name, or load a
`.ttf` / `.otf` / `.woff` / `.woff2` file directly. Novelty fonts like Wingdings
work fine (they're TrueType vector fonts).

### Characters

| Char Set | Contents |
|---|---|
| Default | A-Z, a-z, 0-9, and symbols `&@#?!$*%+` |
| ASCII | Printable codepoints 32-126 |
| CP437 | DOS/BBS character set (box-drawing, block elements, card suits, etc.) |
| Uppercase / Digits | Filtered subsets |
| Custom | Any string you type |

### Appearance

- **Color** -- base glyph colour (hex or colour picker)
- **Colorful** -- each glyph gets a random saturated colour
- **Background** -- canvas background colour

### Goal

Enable a goal opening in the top wall (2D) or back wall (3D). Glyphs that pass
through disappear and respawn as new characters after a random delay.

### 3D Settings

| Setting | Default | Description |
|---|---|---|
| Depth | auto | Room depth in pixels (Z axis). Auto-scales to window size; manually setting it disables auto-scaling on Reset. |
| Glyph Depth | 20 | Thickness of extruded glyphs. |
| Contour Detail | 0.8 | Douglas-Peucker epsilon for silhouette simplification. Lower = smoother curves, more vertices. Requires Reset. |
| Perspective | 60 | Vertical field of view in degrees. |
| Stereo 3D | None | Anaglyph, cross-eyed, or parallel stereoscopic rendering. |

## How the physics works

Each glyph goes through this pipeline:

1. **Rasterization** -- the character is drawn to a canvas at the configured font
   size, and pixels with alpha > 64 are classified as filled.

2. **Contour tracing** -- marching squares on the rasterized bitmap extracts
   closed boundary loops. Tiny contours (antialiasing artifacts) and thin/flat
   contours are filtered out. Douglas-Peucker simplification smooths the
   stair-step edges.

3. **Convex decomposition** -- contour nesting is determined via even-odd
   containment. Holes are bridged into their parent outer contour to form simple
   polygons, then each polygon is split into convex parts using the Bayazit
   algorithm. Body-body collisions test all convex part pairs; wall collisions
   still use the overall convex hull (correct since walls are flat).

4. **Physical properties** -- computed from the actual multi-contour outline:
   - **Area** and **centroid** via Green's theorem, with holes subtracting and
     nested fills adding back.
   - **Mass** = density x ink area (the hole in `O` is not counted).
   - **Moment of inertia** about the centroid, from the second moment of area.

5. **Continuous variable-dt physics** -- decoupled from the display refresh rate.
   Each frame renders first, then a tight loop runs as many physics steps as the
   CPU can fit in the remaining time before the next vsync. Each step uses
   nanosecond-precision elapsed time (`performance.now()`) -- no fixed rate, no
   accumulator.

6. **Collision detection** -- every sub-step, each pair of glyphs is tested:
   - *Broad phase*: bounding-circle distance check (skip far-apart pairs).
   - *Narrow phase*: Separating Axis Theorem on the rotated convex parts (2D) or
     extruded prisms (3D).

7. **Collision response** -- impulse-based rigid-body physics:
   ```
   j = (1+e) * v_n / (1/m_a + 1/m_b + (r_a x n)^2/I_a + (r_b x n)^2/I_b)
   ```
   The impulse changes both linear and angular velocity. Off-centre hits make
   glyphs spin. Glancing hits transfer energy between translation and rotation.

8. **Friction** -- Coulomb model applied tangentially at the contact point,
   clamped to the friction cone `mu * j_normal`.

9. **Wall collisions** -- same impulse math, treating each wall/edge as a body
   with infinite mass. Off-centre wall hits impart spin.

10. **Inter-glyph attraction** (optional) -- every pair of bodies experiences a
    gravitational pull along the line between their centroids. The force is
    `F = G * m_a * m_b / r^n` where *n* = 1 (linear, physically correct in a
    2-D universe where field lines spread across a circle, not a sphere) or
    *n* = 2 (inverse-square, the familiar 3-D law). A softening distance of
    ~10 px prevents the force from diverging when centroids overlap.

## 3D rendering

Glyphs are extruded into prisms and rendered with WebGL:

- **Side faces** -- contour edges extruded from front to back. Smooth vertex
  normals are computed by averaging adjacent edge normals where the angle between
  them is less than ~70 degrees, giving curved surfaces (like O, S) a smooth
  cylinder-like appearance while preserving hard edges at sharp corners (L, T).

- **Front/back caps** -- textured quads using the glyph's rasterized texture
  (rendered with Canvas 2D even-odd fill). Alpha discard in the fragment
  shader makes holes transparent, so characters like @, O, and B render
  correctly at any rotation angle without stencil tricks or special
  triangulation.

- **Lighting** -- Phong shading with diffuse + specular + ambient terms.
  Back-face normals are flipped in the fragment shader (`gl_FrontFacing`) so
  inner surfaces of holes are lit correctly.

## Two kinds of "friction"

| Parameter | What it controls | Physics term |
|---|---|---|
| Restitution | Energy lost on the **bounce** (normal direction). 1.0 = perfect bounce, 0.0 = stick. | Coefficient of restitution |
| Friction | How much an object **slips** at the contact point (tangential direction). Affects spin transfer. 0.0 = ice, 0.3 = rubber. | Coulomb friction coefficient |
