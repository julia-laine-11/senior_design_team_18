"""
Generate two flowchart PNGs for wevibe3.py:
1. High-level overview
2. Defense logic detail
Uses matplotlib.
"""
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

# ============================================================
# SHARED HELPERS
# ============================================================
colors = {
    'start': '#4CAF50',
    'process': '#2196F3',
    'decision': '#FF9800',
    'subprocess': '#9C27B0',
    'io': '#00BCD4',
    'stop': '#F44336',
    'thread': '#795548',
}

def draw_box(ax, x, y, w, h, text, color, fontsize=8, bold=False):
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                         boxstyle="round,pad=0.02,rounding_size=0.2",
                         facecolor=color, edgecolor='black', linewidth=1.5, alpha=0.9)
    ax.add_patch(box)
    weight = 'bold' if bold else 'normal'
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize,
            color='white', wrap=True, fontweight=weight, linespacing=1.2)
    return box

def draw_diamond(ax, x, y, w, h, text, color, fontsize=8):
    diamond = plt.Polygon([
        [x, y + h/2], [x + w/2, y], [x, y - h/2], [x - w/2, y]
    ], facecolor=color, edgecolor='black', linewidth=1.5, alpha=0.9)
    ax.add_patch(diamond)
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize,
            color='white', wrap=True, fontweight='bold', linespacing=1.1)
    return diamond

def draw_arrow(ax, x1, y1, x2, y2, label='', color='black', lw=1.5):
    style = "Simple, tail_width=4, head_width=12, head_length=8"
    kw = dict(arrowstyle=style, color=color, linewidth=lw)
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mx + 0.2, my + 0.15, label, fontsize=8, color='black', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white', edgecolor='none', alpha=0.8))
    a = FancyArrowPatch((x1, y1), (x2, y2), connectionstyle="arc3,rad=0", **kw)
    ax.add_patch(a)

def draw_label(ax, x, y, text, fontsize=8, color='black'):
    ax.text(x, y, text, fontsize=fontsize, color=color, fontweight='bold',
            ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', edgecolor='none', alpha=0.8))

def setup_ax(figsize):
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    ax.set_xlim(0, figsize[0])
    ax.set_ylim(0, figsize[1])
    ax.axis('off')
    return fig, ax

# ============================================================
# 1. HIGH-LEVEL OVERVIEW
# ============================================================
fig, ax = setup_ax((20, 28))

ax.text(10, 27.5, 'wevibe3.py – High-Level Architecture',
        ha='center', va='center', fontsize=18, fontweight='bold', color='#1a237e')

y = 26.5
draw_box(ax, 10, y, 10, 0.8, 'START: main()', colors['start'], 11, bold=True)

y -= 1.3
draw_box(ax, 10, y, 14, 1.0,
         'Initialize GameState\n(Load JSON config: HSV, ROI, bounds, goal, home)',
         colors['process'], 10)

y -= 1.5
# Three parallel threads
draw_box(ax, 4, y, 5, 1.2,
         'Camera Thread\n(V4L2 capture,\n90 FPS frame buffer)',
         colors['thread'], 9)
draw_box(ax, 10, y, 6, 1.2,
         'Tracking Thread\n(Vision → Defense → Motors)',
         colors['thread'], 9)
draw_box(ax, 16, y, 5, 1.2,
         'GUI Thread\n(Tkinter Control Panel\n4 tabs: Game/Bounds/Vision/Status)',
         colors['thread'], 9)
draw_arrow(ax, 10, y + 1.3 - 0.6, 10, y + 0.6, color='black')
draw_arrow(ax, 10, y + 0.6, 4, y + 0.6, color='black')
draw_arrow(ax, 10, y + 0.6, 16, y + 0.6, color='black')

y -= 2.0
draw_box(ax, 10, y, 16, 1.4,
         'VISION PIPELINE (per frame)\n'
         'Resize → Blur → HSV → inRange masks (puck green, mallet orange) → findContours',
         colors['subprocess'], 10)
draw_arrow(ax, 10, y + 1.5 - 0.7, 10, y + 0.7, color='black')

y -= 1.8
draw_box(ax, 10, y, 16, 1.4,
         'OBJECT TRACKING\n'
         'Kalman predict → find_circle (circularity + radius + Kalman hint) → Kalman correct',
         colors['subprocess'], 10)
draw_arrow(ax, 10, y + 1.4 - 0.7, 10, y + 0.7, color='black')

y -= 1.8
draw_box(ax, 10, y, 14, 1.2,
         'DEFENSE LOGIC\nSafety checks → Game/Manual/Clear modes → Trajectory prediction → Intercept/Guard/Homing',
         colors['decision'], 10)
draw_arrow(ax, 10, y + 1.2 - 0.6, 10, y + 0.6, color='black')

y -= 1.6
draw_box(ax, 10, y, 14, 1.0,
         'MOTOR CONTROL\nAcceleration ramping → CoreXY kinematics → Red zone enforcement',
         colors['process'], 10)
draw_arrow(ax, 10, y + 1.0 - 0.5, 10, y + 0.5, color='black')

y -= 1.5
draw_box(ax, 10, y, 16, 1.2,
         'VISUALIZATION & STATE UPDATE\n'
         'OpenCV overlays + state.update_tracking() → GUI reads shared state',
         colors['io'], 10)
draw_arrow(ax, 10, y + 1.2 - 0.6, 10, y + 0.6, color='black')

# Loop arrow back to vision
ax.annotate('', xy=(18, 21.7), xytext=(18, 8.8),
            arrowprops=dict(arrowstyle='->', color='#607D8B', lw=2.5,
                            connectionstyle='arc3,rad=0.25'))
ax.text(19.3, 15.5, 'LOOP\n@ 90 FPS', ha='center', va='center',
        fontsize=10, fontweight='bold', color='#607D8B',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='#607D8B', alpha=0.9))

# Shared state block
y = 6.0
draw_box(ax, 10, y, 12, 1.2,
         'Thread-Safe Shared State (RLock)\n'
         'GameState: tracking results, config, HSV, bounds, STM32 scores',
         colors['thread'], 9)

# Key inputs / outputs
y = 4.5
draw_box(ax, 3.5, y, 5, 0.9, 'Camera\nInput', colors['io'], 9)
draw_box(ax, 10, y, 5, 0.9, 'STM32 UART\nRX/TX', colors['io'], 9)
draw_box(ax, 16.5, y, 5, 0.9, 'User\nInput\n(WASD/G/A)', colors['io'], 9)

# Legend
legend_y = 2.8
legend_items = [
    ('Process', colors['process']),
    ('Decision/Logic', colors['decision']),
    ('Sub-process', colors['subprocess']),
    ('I/O', colors['io']),
    ('Thread', colors['thread']),
]
for i, (name, color) in enumerate(legend_items):
    lx = 2.5 + i * 3.5
    box = FancyBboxPatch((lx - 0.4, legend_y - 0.25), 0.8, 0.5,
                         boxstyle="round,pad=0.02", facecolor=color, edgecolor='black', alpha=0.9)
    ax.add_patch(box)
    ax.text(lx + 0.6, legend_y, name, ha='left', va='center', fontsize=9, color='black')

plt.tight_layout()
plt.savefig('wevibe3_highlevel.png', dpi=200, bbox_inches='tight',
            facecolor='white', edgecolor='none')
print("Saved wevibe3_highlevel.png")

# ============================================================
# 2. DEFENSE LOGIC DETAIL
# ============================================================
fig, ax = setup_ax((20, 32))

ax.text(10, 31.5, 'wevibe3.py – Defense Logic Detail',
        ha='center', va='center', fontsize=18, fontweight='bold', color='#1a237e')

y = 30.5
draw_box(ax, 10, y, 14, 0.9,
         'DEFENSE LOGIC ENTRY\n(mallet x,y, puck x,y,vx,vy, game_on, clear_on, home/goal/bounds)',
         colors['start'], 10, bold=True)

y -= 1.3
draw_diamond(ax, 10, y, 7, 1.0, 'Mallet\nDetected?', colors['decision'], 10)

draw_box(ax, 3.5, y - 1.5, 6, 1.0,
         'SAFETY STOP\nSet target_v = 0\nctrl.stop()', colors['stop'], 10, bold=True)
draw_arrow(ax, 6.5, y, 3.5, y - 1.0, color='black')
draw_label(ax, 4.8, y - 0.8, 'NO', 9, colors['stop'])

# Merge line from safety back
ax.plot([3.5, 3.5, 10], [y - 2.0, y - 3.3, y - 3.3], 'k-', lw=1.5)
ax.annotate('', xy=(10, y - 3.3), xytext=(3.5, y - 3.3),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

y -= 2.3
draw_diamond(ax, 10, y, 6.5, 1.0, 'Game\nMode ON?', colors['decision'], 10)
draw_arrow(ax, 10, y + 1.0 - 0.5, 10, y + 0.5, color='black')
draw_label(ax, 10.8, y + 0.85, 'YES', 9, 'green')

# Manual branch
draw_box(ax, 17, y - 1.5, 5.5, 1.0,
         'MANUAL MODE\nSet target_v from\nWASD/Arrow/Numpad keys',
         colors['process'], 10)
draw_arrow(ax, 13.2, y, 17, y - 1.0, color='black')
draw_label(ax, 15.2, y - 0.8, 'NO', 9, 'black')

# Merge manual back
ax.plot([17, 17, 10], [y - 2.0, y - 3.8, y - 3.8], 'k-', lw=1.5)
ax.annotate('', xy=(10, y - 3.8), xytext=(17, y - 3.8),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

y -= 2.3
draw_diamond(ax, 10, y, 9, 1.2,
             'Clear Mode ON\nAND Puck on our side\nAND Puck stopped\n(speed < 0.05)?',
             colors['decision'], 9)
draw_arrow(ax, 10, y + 1.2 - 0.6, 10, y + 0.6, color='black')
draw_label(ax, 11.5, y + 0.9, 'YES', 9, 'green')

# Clearing branch
y_clear = y - 1.8
draw_box(ax, 3.5, y_clear, 7, 1.6,
         'CLEARING MODE\n'
         '• target_x = puck_x - 50 (get behind)\n'
         '• target_y = puck_y\n'
         '• If already behind & aligned:\n'
         '  target_x = puck_x + 150 (strike right)',
         colors['subprocess'], 9)
draw_arrow(ax, 5.5, y, 3.5, y_clear + 0.8, color='black')
draw_label(ax, 4.5, y - 1.0, 'YES', 9, 'green')

# Merge clearing back
ax.plot([3.5, 3.5, 10], [y_clear - 0.8, y - 3.8, y - 3.8], 'k-', lw=1.5)
ax.annotate('', xy=(10, y - 3.8), xytext=(3.5, y - 3.8),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

# Continue game mode
y -= 2.5
draw_diamond(ax, 10, y, 7, 1.0, 'Puck\nDetected?', colors['decision'], 10)
draw_arrow(ax, 10, y + 1.0 - 0.5, 10, y + 0.5, color='black')
draw_label(ax, 10.8, y + 0.85, 'YES', 9, 'green')

# Homing branch
y_home = y - 1.8
draw_box(ax, 3.5, y_home, 7, 1.4,
         'HOMING\n'
         'target = (home_x, home_y)\n'
         '_drive_to(home)\n'
         'If at home → HOME state',
         colors['process'], 9)
draw_arrow(ax, 6.5, y, 3.5, y_home + 0.7, color='black')
draw_label(ax, 4.8, y - 1.0, 'NO', 9, 'black')

# Merge homing back
ax.plot([3.5, 3.5, 10], [y_home - 0.7, y - 5.3, y - 5.3], 'k-', lw=1.5)
ax.annotate('', xy=(10, y - 5.3), xytext=(3.5, y - 5.3),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

y -= 2.8
draw_box(ax, 10, y, 14, 1.4,
         'TRAJECTORY PREDICTION\n'
         'predict_intercept(px, py, pvx, pvy, goal_x, ... )\n'
         '→ Simulate bounces, return (cross_x, cross_y) if puck will cross goal',
         colors['subprocess'], 9)

y -= 2.0
draw_diamond(ax, 10, y, 8.5, 1.2,
             'Puck attacking?\n(pvx < -7.0)\nAND goal_result\nis not None?',
             colors['decision'], 9)
draw_arrow(ax, 10, y + 1.2 - 0.6, 10, y + 0.6, color='black')
draw_label(ax, 11.5, y + 0.9, 'YES', 9, 'green')

# Intercept branch
y_int = y - 2.2
draw_box(ax, 3.5, y_int, 8, 2.2,
         'INTERCEPT MODE\n'
         '1. V-shape forward push:\n'
         '   push = min(60, |cross_y - goal_y| * 0.5)\n'
         '   target_x = guard_x + push\n'
         '2. Re-predict to mallet face line\n'
         '3. Smooth target_y (stable frames)\n'
         '4. _drive_to(target_x, target_y)\n'
         'State: INTERCEPT / INTERCEPT HOLD',
         colors['subprocess'], 9)
draw_arrow(ax, 6, y, 3.5, y_int + 1.1, color='black')
draw_label(ax, 4.6, y - 1.2, 'YES', 9, 'green')

# Merge intercept back
ax.plot([3.5, 3.5, 10], [y_int - 1.1, y - 6.5, y - 6.5], 'k-', lw=1.5)
ax.annotate('', xy=(10, y - 6.5), xytext=(3.5, y - 6.5),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

# Guard branch
draw_box(ax, 17, y - 1.8, 5.5, 1.6,
         'GUARD CENTER\n'
         'target = (guard_x, goal_y)\n'
         '_drive_to(guard_x, goal_y)\n'
         'State: GUARD_CENTER / WAITING',
         colors['process'], 9)
draw_arrow(ax, 14.2, y, 17, y - 1.0, color='black')
draw_label(ax, 15.8, y - 1.0, 'NO', 9, 'black')

# Merge guard back
ax.plot([17, 17, 10], [y - 2.6, y - 6.5, y - 6.5], 'k-', lw=1.5)
ax.annotate('', xy=(10, y - 6.5), xytext=(17, y - 6.5),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

# All branches converge here
y = 15.5
ax.text(10, y + 0.5, '=== MOTOR OUTPUT ===',
        ha='center', va='center', fontsize=12, fontweight='bold', color='#1a237e')

y -= 1.3
draw_box(ax, 10, y, 14, 1.2,
         'ACCELERATION RAMPING\n'
         'If |target_v| > |prev_v| and same sign:\n'
         '  v = prev_v*(0.75) + target_v*(0.25)\n'
         'Else: v = target_v (snap on stop/reversal)',
         colors['process'], 10)

y -= 1.6
draw_diamond(ax, 10, y, 6.5, 1.0, 'Mallet\nDetected?', colors['decision'], 10)

y -= 1.4
draw_box(ax, 3.5, y, 6, 0.9,
         'STOP\nctrl.stop()', colors['stop'], 10, bold=True)
draw_arrow(ax, 6.8, y + 1.0 - 0.5, 3.5, y + 0.45, color='black')
draw_label(ax, 5, y + 0.85, 'NO', 9, colors['stop'])

# Merge stop back
ax.plot([3.5, 3.5, 10], [y - 0.45, y - 1.3, y - 1.3], 'k-', lw=1.5)
ax.annotate('', xy=(10, y - 1.3), xytext=(3.5, y - 1.3),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

y -= 1.4
draw_box(ax, 10, y, 14, 1.2,
         'DRIVE MOTOR (CoreXY)\n'
         'ctrl.drive(vx, vy, mallet_x, mallet_y, radius)\n'
         'Red zone enforcement (hard stop at margins)\n'
         'Returns: actual_vx, actual_vy, in_red',
         colors['subprocess'], 10)
draw_arrow(ax, 10, y + 1.2 - 0.6, 10, y + 0.6, color='black')
draw_label(ax, 10.8, y + 0.85, 'YES', 9, 'green')

y -= 1.5
draw_box(ax, 10, y, 14, 1.0,
         'Compute motor % display\nA/B motor percentages + direction (F/R)',
         colors['process'], 10)

y -= 1.3
draw_box(ax, 10, y, 14, 0.9,
         'state.update_tracking(...) + OpenCV overlay draw',
         colors['io'], 10)

# Legend
legend_y = 4.0
legend_items = [
    ('Entry', colors['start']),
    ('Process', colors['process']),
    ('Decision', colors['decision']),
    ('Sub-process', colors['subprocess']),
    ('I/O', colors['io']),
    ('Stop', colors['stop']),
]
for i, (name, color) in enumerate(legend_items):
    lx = 2.5 + i * 3.0
    box = FancyBboxPatch((lx - 0.4, legend_y - 0.25), 0.8, 0.5,
                         boxstyle="round,pad=0.02", facecolor=color, edgecolor='black', alpha=0.9)
    ax.add_patch(box)
    ax.text(lx + 0.6, legend_y, name, ha='left', va='center', fontsize=9, color='black')

plt.tight_layout()
plt.savefig('wevibe3_defense_logic.png', dpi=200, bbox_inches='tight',
            facecolor='white', edgecolor='none')
print("Saved wevibe3_defense_logic.png")
