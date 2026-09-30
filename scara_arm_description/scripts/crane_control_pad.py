#!/usr/bin/env python3
"""
crane_control_pad.py  –  Full-featured SCARA crane control panel.

Features:
  • Jog buttons with HOLD-TO-REPEAT for all joints
  • Live telemetry panel (position, heading, lift, arm angles)
  • Distance-to-cargo meter + READY-TO-GRASP indicator
  • Lift progress bar + distance bar with colour coding
  • Preset poses: STOW / GROUND PICK / DELIVER
  • Speed mode: SLOW / NORMAL / FAST  (keys 1/2/3)
  • AUTO PICK & PLACE button
  • Full keyboard capture on the window

Publishes:
  /crane_pad_keys  (std_msgs/String) - single-char key codes -> teleop node
  /crane_preset    (std_msgs/String) - preset/command name   -> teleop node

Subscribes:
  /crane_telemetry (std_msgs/String) - JSON state from teleop node

Run in its own terminal:
    ros2 run scara_arm_description crane_control_pad.py
"""

import json
import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── colours ──────────────────────────────────────────────────────────────────
BG        = '#1e1e2e'
FG        = '#cdd6f4'
BTN_BG    = '#313244'
BTN_FG    = '#cdd6f4'
BTN_ACT   = '#585b70'
ACCENT_1  = '#f38ba8'   # drive / danger
ACCENT_2  = '#a6e3a1'   # lift / safe/green
ACCENT_3  = '#fab387'   # shoulder / elbow / orange
ACCENT_4  = '#89b4fa'   # slider / wrist / blue
ACCENT_5  = '#f5c2e7'   # action buttons
ACCENT_6  = '#cba6f7'   # presets / purple
ACCENT_7  = '#f9e2af'   # speed / yellow
TEAL      = '#94e2d5'
BORDER    = '#45475a'
DIM       = '#6c7086'
DARK_BTN  = '#45475a'

BTN_W         = 7
BTN_H         = 2
PAD_PX        = 4
JOG_DELAY_MS  = 300   # initial hold delay before repeat starts
JOG_REPEAT_MS = 70    # repeat interval while held


class CraneNode(Node):
    """Publishes key strings and preset commands; subscribes to telemetry."""

    def __init__(self):
        super().__init__('crane_control_pad')
        self.pub      = self.create_publisher(String, '/crane_pad_keys', 10)
        self.pre_pub  = self.create_publisher(String, '/crane_preset',   10)
        self.tele     = {}
        self._tele_gui_cb = None   # set after GUI is built

        self.create_subscription(String, '/crane_telemetry', self._on_tele, 10)

    def send(self, ch: str):
        msg = String()
        msg.data = ch
        self.pub.publish(msg)

    def send_preset(self, name: str):
        msg = String()
        msg.data = name
        self.pre_pub.publish(msg)

    def _on_tele(self, msg: String):
        try:
            self.tele = json.loads(msg.data)
            if self._tele_gui_cb:
                self._tele_gui_cb(self.tele)
        except Exception:
            pass


# ── helpers ───────────────────────────────────────────────────────────────────

def _make_btn(parent, text, command, *, bg=BTN_BG, fg=BTN_FG,
              width=BTN_W, height=BTN_H, bfont=None):
    b = tk.Button(
        parent, text=text, command=command,
        bg=bg, fg=fg, activebackground=BTN_ACT, activeforeground=fg,
        relief='flat', bd=0, highlightthickness=1, highlightbackground=BORDER,
        width=width, height=height, font=bfont,
    )
    return b


def _jog_btn(parent, text, send_fn, ch, *, bg=DARK_BTN, fg=BTN_FG,
             width=BTN_W, height=BTN_H, bfont=None):
    """Button that fires immediately and repeats while held down."""
    btn = tk.Button(
        parent, text=text,
        bg=bg, fg=fg, activebackground=BTN_ACT, activeforeground=fg,
        relief='flat', bd=0, highlightthickness=1, highlightbackground=BORDER,
        width=width, height=height, font=bfont,
    )
    _after_id = [None]

    def _repeat():
        send_fn(ch)
        _after_id[0] = btn.after(JOG_REPEAT_MS, _repeat)

    def _press(_event=None):
        send_fn(ch)
        if _after_id[0]:
            btn.after_cancel(_after_id[0])
        _after_id[0] = btn.after(JOG_DELAY_MS, _repeat)

    def _release(_event=None):
        if _after_id[0]:
            btn.after_cancel(_after_id[0])
            _after_id[0] = None

    btn.bind('<ButtonPress-1>',   _press)
    btn.bind('<ButtonRelease-1>', _release)
    btn.configure(command=lambda: None)
    return btn


def _section(parent, title, accent, row, col, colspan=1, rowspan=1):
    frm = tk.LabelFrame(
        parent, text=f'  {title}  ', fg=accent, bg=BG,
        font=('Monospace', 9, 'bold'), bd=1, relief='groove',
        highlightbackground=accent, highlightcolor=accent,
        highlightthickness=1, padx=7, pady=5,
    )
    frm.grid(row=row, column=col, columnspan=colspan, rowspan=rowspan,
             padx=PAD_PX, pady=PAD_PX, sticky='nsew')
    return frm


# ── GUI builder ───────────────────────────────────────────────────────────────

def build_gui(node: CraneNode):
    root = tk.Tk()
    root.title('🏗  SCARA Crane Control Centre')
    root.configure(bg=BG)
    root.resizable(False, False)

    bold  = tkfont.Font(family='Monospace', size=10, weight='bold')
    small = tkfont.Font(family='Monospace', size=8)
    tiny  = tkfont.Font(family='Monospace', size=8)
    big   = tkfont.Font(family='Monospace', size=11, weight='bold')

    send   = node.send
    preset = node.send_preset

    # ── TITLE ─────────────────────────────────────────────────────────────────
    tk.Label(root, text='🏗  SCARA CRANE CONTROL CENTRE',
             bg=BG, fg=FG, font=big, pady=7
             ).grid(row=0, column=0, columnspan=6, sticky='ew')

    # ── DRIVE ─────────────────────────────────────────────────────────────────
    drv = _section(root, 'DRIVE', ACCENT_1, 1, 0, colspan=2)
    _jog_btn(drv, '▲ W', send, 'w', bfont=bold).grid(row=0, column=1, pady=(0, 2))
    _jog_btn(drv, '◀ A', send, 'a', bfont=bold).grid(row=1, column=0, padx=(0, 2))
    tk.Label(drv, text='✛', bg=BG, fg=ACCENT_1, font=bold).grid(row=1, column=1)
    _jog_btn(drv, 'D ▶', send, 'd', bfont=bold).grid(row=1, column=2, padx=(2, 0))
    _jog_btn(drv, '▼ S', send, 's', bfont=bold).grid(row=2, column=1, pady=(2, 0))

    # ── CRANE LIFT ────────────────────────────────────────────────────────────
    lft = _section(root, 'LIFT  I/K', ACCENT_2, 1, 2, colspan=1)
    _jog_btn(lft, '▲ I', send, 'i', bfont=bold, width=6).grid(row=0, column=0, pady=(0, 3))
    tk.Label(lft, text='↕', bg=BG, fg=ACCENT_2, font=bold).grid(row=1, column=0)
    _jog_btn(lft, '▼ K', send, 'k', bfont=bold, width=6).grid(row=2, column=0, pady=(3, 0))

    # ── SHOULDER ──────────────────────────────────────────────────────────────
    sh = _section(root, 'SHOULDER  J/L', ACCENT_3, 2, 0, colspan=2)
    _jog_btn(sh, '◀ J', send, 'j', bfont=bold).grid(row=0, column=0, padx=(0, 4))
    _jog_btn(sh, 'L ▶', send, 'l', bfont=bold).grid(row=0, column=1, padx=(4, 0))

    # ── ELBOW ─────────────────────────────────────────────────────────────────
    el = _section(root, 'ELBOW  U/O', ACCENT_3, 2, 2, colspan=1)
    _jog_btn(el, '◀ U', send, 'u', bfont=bold, width=6).grid(row=0, column=0, pady=2)
    _jog_btn(el, 'O ▶', send, 'o', bfont=bold, width=6).grid(row=1, column=0, pady=2)

    # ── SLIDER ────────────────────────────────────────────────────────────────
    sl = _section(root, 'SLIDER  T/G', ACCENT_4, 3, 0, colspan=2)
    _jog_btn(sl, '▲ T', send, 't', bfont=bold).grid(row=0, column=0, padx=(0, 4))
    _jog_btn(sl, 'G ▼', send, 'g', bfont=bold).grid(row=0, column=1, padx=(4, 0))

    # ── WRIST ─────────────────────────────────────────────────────────────────
    wr = _section(root, 'WRIST  R/F', ACCENT_4, 3, 2, colspan=1)
    _jog_btn(wr, '↺ R', send, 'r', bfont=bold, width=6).grid(row=0, column=0, pady=2)
    _jog_btn(wr, 'F ↻', send, 'f', bfont=bold, width=6).grid(row=1, column=0, pady=2)

    # ── TELEMETRY PANEL ───────────────────────────────────────────────────────
    tele_frm = _section(root, 'LIVE TELEMETRY', TEAL, 1, 3, colspan=3, rowspan=3)
    tele_frm.grid_columnconfigure(0, weight=1)

    def _lbl(text, fg=FG, f=small):
        return tk.Label(tele_frm, text=text, bg=BG, fg=fg, font=f, anchor='w')

    tk.Label(tele_frm, text='── POSITION ─────────', bg=BG, fg=TEAL,
             font=small, anchor='w').grid(row=0, column=0, sticky='w')
    lbl_xy   = _lbl('X:  ---    Y:  ---');  lbl_xy.grid(row=1, column=0, sticky='w')
    lbl_head = _lbl('Heading:    --- °');    lbl_head.grid(row=2, column=0, sticky='w')
    lbl_lift = _lbl('Lift:   --- m');        lbl_lift.grid(row=3, column=0, sticky='w')

    tk.Label(tele_frm, text='── ARM JOINTS ───────', bg=BG, fg=TEAL,
             font=small, anchor='w').grid(row=4, column=0, sticky='w', pady=(5, 0))
    lbl_j1 = _lbl('J1 shoulder:  --- °'); lbl_j1.grid(row=5, column=0, sticky='w')
    lbl_j2 = _lbl('J2 elbow:     --- °'); lbl_j2.grid(row=6, column=0, sticky='w')
    lbl_j3 = _lbl('J3 slider:    --- m'); lbl_j3.grid(row=7, column=0, sticky='w')
    lbl_j4 = _lbl('J4 wrist:     --- °'); lbl_j4.grid(row=8, column=0, sticky='w')

    tk.Label(tele_frm, text='── CARGO ────────────', bg=BG, fg=TEAL,
             font=small, anchor='w').grid(row=9, column=0, sticky='w', pady=(5, 0))
    lbl_dist  = _lbl('Dist to cargo:  --- cm'); lbl_dist.grid(row=10, column=0, sticky='w')
    lbl_cargo = _lbl('Cargo: ---');              lbl_cargo.grid(row=11, column=0, sticky='w')
    lbl_grip  = _lbl('Gripper:  OPEN');          lbl_grip.grid(row=12, column=0, sticky='w')

    ready_var = tk.StringVar(value='  ○ ALIGN GRIPPER')
    ready_lbl = tk.Label(tele_frm, textvariable=ready_var,
                         bg=BG, fg=DIM, font=('Monospace', 9, 'bold'), anchor='w')
    ready_lbl.grid(row=13, column=0, sticky='w', pady=(4, 0))

    speed_var = tk.StringVar(value='SPEED:  NORMAL')
    speed_lbl = tk.Label(tele_frm, textvariable=speed_var,
                         bg=BG, fg=ACCENT_7, font=('Monospace', 9, 'bold'), anchor='w')
    speed_lbl.grid(row=14, column=0, sticky='w', pady=(4, 0))

    # Lift bar
    tk.Label(tele_frm, text='── LIFT HEIGHT ──────', bg=BG, fg=TEAL,
             font=small, anchor='w').grid(row=15, column=0, sticky='w', pady=(6, 0))
    lift_canvas = tk.Canvas(tele_frm, width=180, height=12, bg='#181825',
                            highlightthickness=0)
    lift_canvas.grid(row=16, column=0, sticky='w')
    lift_bar = lift_canvas.create_rectangle(0, 0, 0, 12, fill=ACCENT_2, outline='')
    lift_pct_lbl = tk.Label(tele_frm, text='0 %', bg=BG, fg=ACCENT_2,
                             font=tiny, anchor='w')
    lift_pct_lbl.grid(row=17, column=0, sticky='w')

    # Distance bar
    tk.Label(tele_frm, text='── DIST TO CARGO ────', bg=BG, fg=TEAL,
             font=small, anchor='w').grid(row=18, column=0, sticky='w', pady=(5, 0))
    dist_canvas = tk.Canvas(tele_frm, width=180, height=12, bg='#181825',
                            highlightthickness=0)
    dist_canvas.grid(row=19, column=0, sticky='w')
    dist_bar = dist_canvas.create_rectangle(0, 0, 180, 12, fill=ACCENT_1, outline='')

    # ── PRESET POSES ──────────────────────────────────────────────────────────
    pre_frm = _section(root, 'PRESET POSES', ACCENT_6, 4, 3, colspan=3)
    _make_btn(pre_frm, '📦 STOW\n(travel)',  lambda: preset('stow'),
              bg=DARK_BTN, fg=ACCENT_6, width=10, bfont=small
              ).grid(row=0, column=0, padx=3)
    _make_btn(pre_frm, '⬇ PICK\n(ground)',   lambda: preset('pick'),
              bg=DARK_BTN, fg=ACCENT_6, width=10, bfont=small
              ).grid(row=0, column=1, padx=3)
    _make_btn(pre_frm, '📍 DELIVER\n(pad)',  lambda: preset('deliver'),
              bg=DARK_BTN, fg=ACCENT_6, width=10, bfont=small
              ).grid(row=0, column=2, padx=3)

    # ── SPEED MODE ────────────────────────────────────────────────────────────
    spd_frm = _section(root, 'SPEED MODE  [1/2/3]', ACCENT_7, 5, 3, colspan=3)
    _make_btn(spd_frm, '🐢 SLOW',   lambda: send('1'),
              bg=DARK_BTN, fg=ACCENT_2, width=9, bfont=small).grid(row=0, column=0, padx=3)
    _make_btn(spd_frm, '🚀 NORMAL', lambda: send('2'),
              bg=DARK_BTN, fg=ACCENT_7, width=9, bfont=small).grid(row=0, column=1, padx=3)
    _make_btn(spd_frm, '⚡ FAST',   lambda: send('3'),
              bg=DARK_BTN, fg=ACCENT_1, width=9, bfont=small).grid(row=0, column=2, padx=3)

    # ── ACTION BUTTONS ────────────────────────────────────────────────────────
    act = tk.Frame(root, bg=BG)
    act.grid(row=6, column=0, columnspan=6, pady=(6, 4))

    _make_btn(act, '⏚ GRIPPER\n[SPACE]', lambda: send(' '),
              bg=DARK_BTN, fg=ACCENT_5, width=11, bfont=small).grid(row=0, column=0, padx=3)
    _make_btn(act, '⌂ HOME\n[H]',        lambda: send('h'),
              bg=DARK_BTN, fg=ACCENT_2, width=11, bfont=small).grid(row=0, column=1, padx=3)
    _make_btn(act, '↻ RESPAWN\n[N]',     lambda: send('n'),
              bg=DARK_BTN, fg=ACCENT_3, width=11, bfont=small).grid(row=0, column=2, padx=3)
    _make_btn(act, '🤖 AUTO\nPICK&PLACE', lambda: preset('auto'),
              bg=DARK_BTN, fg=TEAL,     width=11, bfont=small).grid(row=0, column=3, padx=3)
    _make_btn(act, '✕ QUIT\n[Q]',        lambda: send('q'),
              bg=DARK_BTN, fg=ACCENT_1, width=11, bfont=small).grid(row=0, column=4, padx=3)

    # ── STATUS BAR ────────────────────────────────────────────────────────────
    status_var = tk.StringVar(value='Ready  –  hold jog buttons for continuous movement')
    tk.Label(root, textvariable=status_var, bg='#181825', fg=DIM,
             font=tiny, anchor='w', padx=8, pady=3
             ).grid(row=7, column=0, columnspan=6, sticky='ew')

    # ── KEYBOARD ──────────────────────────────────────────────────────────────
    KEYMAP = {
        'w': 'w', 's': 's', 'a': 'a', 'd': 'd',
        'i': 'i', 'k': 'k',
        'j': 'j', 'l': 'l',
        'u': 'u', 'o': 'o',
        't': 't', 'g': 'g',
        'r': 'r', 'f': 'f',
        'space': ' ',
        'h': 'h', 'n': 'n', 'q': 'q',
        '1': '1', '2': '2', '3': '3',
    }

    def _on_key(event):
        ch = KEYMAP.get(event.keysym.lower())
        if ch:
            send(ch)

    root.bind('<KeyPress>', _on_key)

    # ── TELEMETRY UPDATER ─────────────────────────────────────────────────────
    def _tele_ros_cb(t: dict):
        root.after(0, lambda: _apply_tele(t))

    def _apply_tele(t: dict):
        try:
            x    = t.get('x', 0.0)
            y    = t.get('y', 0.0)
            head = t.get('heading_deg', 0.0)
            lift = t.get('lift', 0.0)
            j1   = t.get('j1_deg', 0.0)
            j2   = t.get('j2_deg', 0.0)
            j3   = t.get('j3', 0.0)
            j4   = t.get('j4_deg', 0.0)
            dist = t.get('dist_to_cargo', 999.0)
            grip = t.get('gripper', 'open')
            held = t.get('holding', False)
            near = t.get('near_grasp', False)
            spd  = t.get('speed_mode', 'NORMAL')
            cargo_name = t.get('cargo_name', '---')

            lbl_xy.config(   text=f'X: {x:+.2f} m   Y: {y:+.2f} m')
            lbl_head.config( text=f'Heading:  {head:+5.1f} °')
            lbl_lift.config( text=f'Lift:  {lift:+.3f} m')
            lbl_j1.config(   text=f'J1 shoulder:  {j1:+6.1f} °')
            lbl_j2.config(   text=f'J2 elbow:     {j2:+6.1f} °')
            lbl_j3.config(   text=f'J3 slider:    {j3:+.3f} m')
            lbl_j4.config(   text=f'J4 wrist:     {j4:+6.1f} °')

            dist_cm = dist * 100.0
            lbl_dist.config( text=f'Dist to cargo:  {dist_cm:.1f} cm')
            lbl_cargo.config(text=f'Cargo: {cargo_name}')

            if held:
                lbl_grip.config(text='Gripper:  HOLDING ✓', fg=ACCENT_2)
            elif grip == 'closing':
                lbl_grip.config(text='Gripper:  CLOSING...', fg=ACCENT_1)
            else:
                lbl_grip.config(text='Gripper:  OPEN', fg=FG)

            speed_var.set(f'SPEED:  {spd}')
            speed_lbl.config(fg=(ACCENT_2 if spd == 'SLOW' else
                                 ACCENT_1 if spd == 'FAST' else ACCENT_7))

            if held:
                ready_var.set('  ● CARGO ATTACHED  ✓')
                ready_lbl.config(fg=ACCENT_2)
            elif near:
                ready_var.set('  ● READY TO GRASP! ✓')
                ready_lbl.config(fg=ACCENT_2)
            elif dist_cm < 20:
                ready_var.set(f'  ◐ ALIGN...  {dist_cm:.0f} cm')
                ready_lbl.config(fg=ACCENT_3)
            else:
                ready_var.set('  ○ ALIGN GRIPPER')
                ready_lbl.config(fg=DIM)

            # Lift progress bar  (-0.14 bottom → 0 → +0.10 top)
            total_range = 0.24
            lift_pct = max(0.0, min(1.0, (lift + 0.14) / total_range))
            bar_w = int(180 * lift_pct)
            lift_canvas.coords(lift_bar, 0, 0, bar_w, 12)
            lcol = ACCENT_2 if lift >= 0 else ACCENT_4
            lift_canvas.itemconfig(lift_bar, fill=lcol)
            lift_pct_lbl.config(text=f'{lift_pct*100:.0f}%  ({lift:+.3f} m)', fg=lcol)

            # Distance bar (0 cm = green/close, ≥60 cm = red/far)
            d_pct  = max(0.0, min(1.0, dist_cm / 60.0))
            bar_w2 = int(180 * d_pct)
            dist_canvas.coords(dist_bar, 0, 0, bar_w2, 12)
            dcol = (ACCENT_2 if d_pct < 0.25 else
                    ACCENT_3 if d_pct < 0.6  else ACCENT_1)
            dist_canvas.itemconfig(dist_bar, fill=dcol)

            status_var.set(
                f'x={x:+.2f} y={y:+.2f} | lift={lift:+.3f} | '
                f'j1={j1:+.0f}° j2={j2:+.0f}° | cargo={cargo_name} | spd={spd}'
            )
        except Exception as e:
            status_var.set(f'Telemetry parse error: {e}')

    node._tele_gui_cb = _tele_ros_cb
    return root


def main():
    rclpy.init()
    node = CraneNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = build_gui(node)

    def on_close():
        node.send('q')
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
