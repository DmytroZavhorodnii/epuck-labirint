"""
Generates a Webots R2023b .wbt file for a 10x10 maze with E-puck robots.
Coordinate system: Z-up (ENU default in R2023b).
  X, Y → horizontal (maze floor plane)
  Z    → up (height)

Run once: python generate_world.py
"""
import random

ROWS = 10
COLS = 10
CELL = 0.2        # cell size in meters  (corridor ~0.18 m ≈ 2.4× E-puck diameter)
WTHK = 0.02       # wall thickness
WHGT = 0.15       # wall height
SEED = 42

# ── Maze generation (recursive backtracking DFS) ──────────────────────────────

def generate_maze(rows, cols, seed=SEED):
    rng = random.Random(seed)
    walls = [[{'N', 'S', 'E', 'W'} for _ in range(cols)] for _ in range(rows)]
    visited = [[False] * cols for _ in range(rows)]
    opposite = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
    delta    = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}

    def dfs(r, c):
        visited[r][c] = True
        dirs = list(delta.keys())
        rng.shuffle(dirs)
        for d in dirs:
            dr, dc = delta[d]
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                walls[r][c].discard(d)
                walls[nr][nc].discard(opposite[d])
                dfs(nr, nc)

    dfs(0, 0)
    return walls

# ── Webots geometry helpers (Z-up: X-Y = floor, Z = height) ──────────────────

def box_solid(name, tx, ty, tz, sx, sy, sz, color=(0.7, 0.7, 0.7)):
    r, g, b = color
    return f"""Solid {{
  translation {tx:.4f} {ty:.4f} {tz:.4f}
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {r} {g} {b}
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {sx:.4f} {sy:.4f} {sz:.4f}
      }}
    }}
  ]
  boundingObject Box {{
    size {sx:.4f} {sy:.4f} {sz:.4f}
  }}
}}
"""

# ── Build wall list ───────────────────────────────────────────────────────────

def build_walls(walls):
    solids = []
    total_w = COLS * CELL   # X extent
    total_h = ROWS * CELL   # Y extent
    cz = WHGT / 2           # wall center Z (half height)

    # ── Outer North wall (Y=0) — gap at col 0 (entrance 1) and col 5 (entrance 2) ──
    for c in range(COLS):
        if c == 0 or c == 5:
            continue
        cx = c * CELL + CELL / 2
        solids.append(box_solid(f"wN_{c}", cx, 0.0, cz, CELL, WTHK, WHGT))

    # ── Outer South wall (Y=total_h) — gap at col 9 (exit) ──────────────────
    for c in range(COLS):
        if c == COLS - 1:
            continue
        cx = c * CELL + CELL / 2
        solids.append(box_solid(f"wS_{c}", cx, total_h, cz, CELL, WTHK, WHGT))

    # ── Outer West wall (X=0) — full ─────────────────────────────────────────
    solids.append(box_solid("wW", 0.0, total_h / 2, cz, WTHK, total_h, WHGT))

    # ── Outer East wall (X=total_w) — full ───────────────────────────────────
    solids.append(box_solid("wE", total_w, total_h / 2, cz, WTHK, total_h, WHGT))

    # ── Inner horizontal walls (S edge of each cell, along X axis) ───────────
    for r in range(ROWS - 1):
        for c in range(COLS):
            if 'S' in walls[r][c]:
                cx = c * CELL + CELL / 2
                cy = (r + 1) * CELL
                solids.append(box_solid(f"wh_{r}_{c}", cx, cy, cz, CELL, WTHK, WHGT))

    # ── Inner vertical walls (E edge of each cell, along Y axis) ─────────────
    for r in range(ROWS):
        for c in range(COLS - 1):
            if 'E' in walls[r][c]:
                cx = (c + 1) * CELL
                cy = r * CELL + CELL / 2
                solids.append(box_solid(f"wv_{r}_{c}", cx, cy, cz, WTHK, CELL, WHGT))

    return solids

# ── Node templates ────────────────────────────────────────────────────────────

HEADER = '''#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/gctronic/e-puck/protos/E-puck.proto"

WorldInfo {
  title "Labirynt E-puck 10x10"
  info [ "10x10 maze navigation - E-puck robots" ]
  coordinateSystem "ENU"
}
Viewpoint {
  orientation -0.577 -0.577 -0.577 2.094
  position 3.0 -2.0 4.0
}
TexturedBackground {
}
TexturedBackgroundLight {
}
'''


def floor_node(total_w, total_h):
    cx, cy = total_w / 2, total_h / 2
    return f"""Solid {{
  translation {cx:.4f} {cy:.4f} -0.0025
  name "floor"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.9 0.85 0.75
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {total_w:.4f} {total_h:.4f} 0.005
      }}
    }}
  ]
  boundingObject Box {{
    size {total_w:.4f} {total_h:.4f} 0.005
  }}
}}
"""


def goal_node():
    # Exit gap: col=9, row=9, South wall → goal just past Y = 10*CELL
    cx = 9 * CELL + CELL / 2
    cy = ROWS * CELL + 0.06
    return f"""Solid {{
  translation {cx:.4f} {cy:.4f} 0.05
  name "goal"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0.8 0
        roughness 0.4
        metalness 0
        emissiveColor 0 0.5 0
      }}
      geometry Box {{
        size 0.16 0.05 0.10
      }}
    }}
  ]
}}
"""


def epuck_node(name, controller, tx, ty, rot_z=0.0, with_comm=False):
    slots = ""
    if with_comm:
        slots = """
  turretSlot [
    GPS {
      name "gps"
    }
  ]"""
    return f"""E-puck {{
  translation {tx:.4f} {ty:.4f} 0
  rotation 0 0 1 {rot_z:.4f}
  name "{name}"
  controller "{controller}"{slots}
}}
"""


# ── Assemble world file ───────────────────────────────────────────────────────

def generate_wbt(walls):
    total_w = COLS * CELL
    total_h = ROWS * CELL
    lines = [HEADER]

    lines.append(floor_node(total_w, total_h))
    lines.append(goal_node())

    # Robot 1 — entrance at col=0, North gap → placed at cell (0,0)
    lines.append(epuck_node("e-puck", "epuck_maze2",
                             0 * CELL + CELL / 2, 0 * CELL + CELL / 2,
                             rot_z=0.0, with_comm=True))

    # Robot 2 — entrance at col=5, North gap → placed at cell (0,5)
    lines.append(epuck_node("e-puck2", "epuck_maze2",
                             5 * CELL + CELL / 2, 0 * CELL + CELL / 2,
                             rot_z=3.14159, with_comm=True))

    for s in build_walls(walls):
        lines.append(s)

    return "\n".join(lines)


# ── Run ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    walls = generate_maze(ROWS, COLS)
    wbt   = generate_wbt(walls)
    out   = "worlds/labirynt.wbt"
    with open(out, "w", encoding="utf-8") as f:
        f.write(wbt)
    print(f"World written -> {out}")

    print("\nMaze preview  (S=robot1 start, 2=robot2 start, E=exit):")
    for r in range(ROWS):
        row_str = ""
        for c in range(COLS):
            if r == 0 and c == 0:
                ch = "S"
            elif r == 0 and c == 5:
                ch = "2"
            elif r == ROWS - 1 and c == COLS - 1:
                ch = "E"
            else:
                ch = "."
            row_str += ch
            row_str += " " if "E" not in walls[r][c] or c == COLS - 1 else "|"
        print(row_str)
        if r < ROWS - 1:
            sep = ""
            for c in range(COLS):
                sep += "-" if "S" in walls[r][c] else " "
                sep += "+"
            print(sep)
