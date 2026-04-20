# Bouncing Glyphs

Rigid-body physics simulation of font glyphs. Extracts actual vector outlines
from TrueType/OpenType fonts, computes each glyph's convex hull, area,
centroid, mass, and moment of inertia, then simulates them as 2-D rigid bodies
bouncing off each other and the screen edges.

All collision math -- impulse magnitudes, rotational velocity changes, friction
-- is derived from each glyph's specific shape, not a bounding circle or box.

![demo concept](https://img.shields.io/badge/python-3.8%2B-blue)

## Install

```
pip install pygame-ce fonttools
```

(or `pip install pygame fonttools` -- either works)

## Quick start

```bash
python bouncing_glyphs.py
```

Auto-detects a system font, spawns 15 white glyphs, perfectly elastic
collisions, no gravity, no friction.

## Examples

```bash
# Big chunky Impact letters with gravity
python bouncing_glyphs.py --font "C:/Windows/Fonts/impact.ttf" --count 25 --gravity 500

# Random Unicode glyphs in rainbow colours
python bouncing_glyphs.py --unicode --colorful --count 30

# Realistic: friction slows sliding, gravity pulls down, slight energy loss
python bouncing_glyphs.py --friction 0.3 --gravity 500 --restitution 0.9

# Huge glyphs, fast, orange, with debug hulls visible
python bouncing_glyphs.py --font-size 120 --speed-max 600 --color "255,140,0" --debug

# Reproducible run
python bouncing_glyphs.py --seed 42

# Just the word "HELLO"
python bouncing_glyphs.py --chars "HELLO" --count 5 --font-size 100
```

## CLI options

| Option | Default | Description |
|---|---|---|
| `--font PATH` | auto-detect | Path to a `.ttf` / `.otf` / `.ttc` font file. Falls back to Arial, Helvetica, DejaVu Sans, or pygame's bundled font. |
| `--count N` | `15` | Number of glyphs to spawn. |
| `--width W` | `1280` | Window width in pixels. |
| `--height H` | `720` | Window height in pixels. |
| `--speed-min V` | `100` | Minimum initial speed (px/s). Each glyph gets a random speed in `[min, max]` at a random angle. |
| `--speed-max V` | `400` | Maximum initial speed (px/s). |
| `--font-size S` | `72` | Font size in pixels (em height). Larger = heavier glyphs with more inertia. |
| `--gravity G` | `0` | Downward gravitational acceleration (px/s^2). `0` = zero-g. Try `500` for a nice tumbling effect. |
| `--restitution E` | `1.0` | Coefficient of restitution (bounciness). `1.0` = perfectly elastic, no energy lost on normal impact. `0.0` = perfectly inelastic, bodies stick. |
| `--friction MU` | `0.0` | Coulomb friction coefficient. `0` = frictionless (energy conserved perfectly). `0.3` = realistic sliding friction (collisions bleed tangential kinetic energy, glyphs gradually slow down and stop spinning). |
| `--chars CHARS` | `A-Z a-z 0-9 &@#...` | Character pool to draw from. Each spawned glyph picks a random character from this string. |
| `--unicode` | off | Pick from the font's entire Unicode character map instead of `--chars`. Gives you accented letters, symbols, Greek, Cyrillic -- whatever the font supports. |
| `--colorful` | off | Each glyph gets a random saturated colour. Without this flag, all glyphs are the same colour. |
| `--color C` | `white` | Base colour for all glyphs (when `--colorful` is not set). Accepts a pygame colour name (`red`, `cyan`, `gold`, ...) or `R,G,B` like `"255,100,0"`. |
| `--debug` | off | Draw convex-hull wireframes (yellow) and centroid crosses (red) over each glyph. Useful for seeing exactly what the physics engine "sees". |
| `--seed N` | random | RNG seed for reproducible runs. Same seed = same initial positions, velocities, characters, and colours. |

## Interactive keys

| Key | Action |
|---|---|
| **ESC** | Quit |
| **Space** | Pause / resume |
| **D** | Toggle debug overlay (convex hulls + centroids) |
| **G** | Toggle gravity on/off (0 or 500 px/s^2) |

## How the physics works

Each glyph goes through this pipeline:

1. **Outline extraction** -- fontTools reads the glyph's Bezier curves from the
   font file and flattens them into a polyline.

2. **Convex hull** -- Andrew's monotone-chain algorithm computes the tightest
   convex polygon enclosing the outline.

3. **Physical properties** -- from the hull polygon, computed exactly:
   - **Area** and **centroid** via Green's theorem.
   - **Mass** = density x area (bigger glyphs are heavier).
   - **Moment of inertia** about the centroid, from the polygon's second moment
     of area. A thin `I` has very different rotational inertia than a round `O`.

4. **Collision detection** -- every frame, each pair of glyphs is tested:
   - *Broad phase*: bounding-circle distance check (skip far-apart pairs).
   - *Narrow phase*: Separating Axis Theorem on the rotated convex hulls.

5. **Collision response** -- impulse-based rigid-body physics:
   ```
   j = (1+e) * v_n / (1/m_a + 1/m_b + (r_a x n)^2/I_a + (r_b x n)^2/I_b)
   ```
   - `j` = impulse magnitude
   - `e` = coefficient of restitution (`--restitution`)
   - `v_n` = relative approach velocity along the collision normal
   - `r` = lever arm from centroid to contact point
   - `I` = moment of inertia

   The impulse changes both **linear** and **angular** velocity. An off-centre
   hit makes the glyph spin. A glancing hit transfers energy between translation
   and rotation.

6. **Friction** -- Coulomb model applied tangentially at the contact point:
   - Opposes sliding at the contact.
   - Clamped to `mu * j_normal` (the friction cone).
   - With `--friction 0` (default): perfectly frictionless, energy is conserved
     exactly.
   - With `--friction 0.3`: realistic surface friction. Glyphs gradually lose
     kinetic energy and stop spinning.

7. **Wall collisions** -- same impulse math, treating each screen edge as a
   body with infinite mass. Off-centre wall hits impart spin.
