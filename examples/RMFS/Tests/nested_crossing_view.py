"""Top-view replay of the committed events in the nested crossing example."""

from bisect import bisect_right
from time import perf_counter
import tkinter as tk


LABELS = ('Immediate admission + FIFO', 'Lookahead depth 1', 'Lookahead depth 2')
COLORS = dict(R1='#b92d38', R2='#047349', R3='#2458af', R4='#7037a0', blocker='#526174')


def show(results, depth, *, root=None):
    root = tk.Tk() if root is None else root
    root.title('Nested crossing lookahead')
    root.resizable(False, False)
    toolbar = tk.Frame(root, bg='white')
    toolbar.pack(fill='x')
    canvas = tk.Canvas(root, width=900, height=370, bg='white', highlightthickness=0)
    canvas.pack()
    selected = tk.IntVar(root, value=depth)
    time = 0.
    paused = False
    last_tick = perf_counter()
    times = {key: [frame['time'] for frame in result['frames']] for key, result in results.items()}

    def draw():
        result = results[selected.get()]
        index = bisect_right(times[selected.get()], time) - 1
        state = result['frames'][index]
        canvas.delete('all')
        canvas.create_text(32, 25, anchor='w', font=('Segoe UI', 14, 'bold'),
                           text=f'{time:04.1f} / {result["makespan"]:.1f} s')
        canvas.create_text(868, 26, anchor='e', font=('Segoe UI', 10),
                           text='Top-view schematic · event replay')
        for x, label in ((188, 'Rear crossing'), (390, 'Front crossing'), (665, 'Downstream crossing')):
            canvas.create_rectangle(x-35, 129, x+35, 199, fill='#eaf1f6', outline='#7894a8', width=2)
            canvas.create_text(x, 105, text=label, font=('Segoe UI', 11, 'bold'))
        canvas.create_line(45, 164, 850, 164, fill='#7894a8', dash=(5, 5), arrow='last')
        canvas.create_line(188, 295, 188, 202, 630, 202, fill='#7894a8', dash=(5, 5), arrow='last')
        canvas.create_line(390, 235, 390, 200, fill='#7894a8', dash=(5, 5), arrow='last')
        finished = state['finished']
        positions = {'R1': (138, 164), 'R2': (188, 260), 'R3': (665, 164),
                     'R4': (390, 235), 'blocker': (465, 164)}
        if state['r1_at_front'] or state['rear_clear'] is not None:
            positions['R1'] = (515, 146)
        if state['rear_clear'] is not None:
            positions['R2'] = (510, 202)
        if state['front']:
            positions[state['front']] = (390, 164)
        for i, robot in enumerate(state['waiting']):
            positions[robot] = (608 - i * 50, 164)
        if state['downstream']:
            positions[state['downstream']] = (665, 164)
        for robot in state['tail']:
            positions[robot] = (746, 143 if robot == 'R1' else 184)
        for i, robot in enumerate(positions):
            if robot in finished:
                positions[robot] = (565 + 68 * i, 268)
            x, y = positions[robot]
            half_width = 25 if robot == 'blocker' else 18
            canvas.create_rectangle(x-half_width, y-12, x+half_width, y+12,
                                    fill=COLORS[robot], outline='')
            canvas.create_text(x, y, text=robot, fill='white', font=('Segoe UI', 10, 'bold'))
        canvas.create_text(542, 242, text='Completed robots', anchor='w', font=('Segoe UI', 10))
        notes = []
        if state['decisions']:
            notes.append(f'Push · R1 waits {state["decisions"][0][1]}')
        if len(state['decisions']) > 1:
            stamp, robot = state['decisions'][1]
            notes.append(f'{stamp:g} s · {robot} granted downstream passage')
        elif state['downstream'] in ('R1', 'R2'):
            notes.append(f'{state["downstream"]} crossing downstream')
        canvas.create_text(32, 326, anchor='w', text='    |    '.join(notes), font=('Segoe UI', 11))
        canvas.create_text(32, 350, anchor='w', text='1–3  Select mode     Space  Pause     R  Replay',
                           font=('Segoe UI', 10), fill='#526174')

    def tick():
        nonlocal time, last_tick
        now = perf_counter()
        if not paused:
            time = min(results[selected.get()]['makespan'], time + now - last_tick)
        last_tick = now
        draw()
        root.after(33, tick)

    def restart():
        nonlocal time, paused, last_tick
        time, paused, last_tick = 0., False, perf_counter()
        draw()

    def select(key):
        selected.set(key)
        restart()

    def toggle():
        nonlocal paused
        paused = not paused

    for key, label in enumerate(LABELS):
        tk.Radiobutton(toolbar, text=label, variable=selected, value=key, command=restart,
                       bg='white', activebackground='white', padx=10, pady=8).pack(side='left')
        root.bind(str(key+1), lambda event, key=key: select(key))
    tk.Button(toolbar, text='Replay', command=restart).pack(side='right', padx=8)
    tk.Button(toolbar, text='Pause / Resume', command=toggle).pack(side='right')
    root.bind('<space>', lambda event: toggle())
    root.bind('<r>', lambda event: restart())
    tick()
    root.mainloop()
