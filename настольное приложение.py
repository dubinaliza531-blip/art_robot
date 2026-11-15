import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import PathPatch
from matplotlib.path import Path
import serial
import serial.tools.list_ports
import threading
import time
import xml.etree.ElementTree as ET
from svgpath2mpl import parse_path
from datetime import datetime


class VEXPlotterController:
    def __init__(self, root):
        self.root = root
        self.root.title("VEX V5 Plotter - Debug & Streaming")
        self.root.geometry("1300x850")

        self.x_max = 500
        self.y_max = 180

        self.raw_gcode = []
        self.current_paths = []
        self.svg_height = 0

        self.serial_port = None
        self.is_sending = False

        self.setup_ui()

    def setup_ui(self):
        main_paned = ttk.PanedWindow(self.root, orient='horizontal')
        main_paned.pack(fill='both', expand=True, padx=10, pady=10)

        control_frame = ttk.Frame(main_paned)
        main_paned.add(control_frame, weight=1)

        right_frame = ttk.Frame(main_paned)
        main_paned.add(right_frame, weight=2)

        ttk.Label(control_frame, text="VEX V5 Control Panel", font=('Arial', 14, 'bold')).pack(pady=5)

        f_conn = ttk.LabelFrame(control_frame, text="1. Подключение", padding=8)
        f_conn.pack(fill='x', pady=5)

        p_frame = ttk.Frame(f_conn)
        p_frame.pack(fill='x', pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(p_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side='left', padx=2)
        ttk.Button(p_frame, text="⟳", width=3, command=self.refresh_ports).pack(side='left')

        ttk.Button(f_conn, text="🔌 Подключиться", command=self.connect_robot).pack(fill='x', pady=2)

        f_debug = ttk.LabelFrame(control_frame, text="Тест связи (Отправка вручную)", padding=8)
        f_debug.pack(fill='x', pady=5)

        debug_btn_frame = ttk.Frame(f_debug)
        debug_btn_frame.pack(fill='x', pady=2)
        ttk.Button(debug_btn_frame, text="Отправить G28 (Home)", command=lambda: self.send_single_command("G28")).pack(
            side='left', expand=True, fill='x', padx=1)
        ttk.Button(debug_btn_frame, text="Отправить M3 (Перо)", command=lambda: self.send_single_command("M3")).pack(
            side='left', expand=True, fill='x', padx=1)
        ttk.Button(debug_btn_frame, text="Отправить M5 (Поднять)", command=lambda: self.send_single_command("M5")).pack(
            side='left', expand=True, fill='x', padx=1)

        custom_cmd_frame = ttk.Frame(f_debug)
        custom_cmd_frame.pack(fill='x', pady=5)
        self.custom_cmd_var = tk.StringVar()
        ttk.Entry(custom_cmd_frame, textvariable=self.custom_cmd_var).pack(side='left', fill='x', expand=True,
                                                                           padx=(0, 5))
        ttk.Button(custom_cmd_frame, text="➤ Отправить", command=self.send_custom_command).pack(side='left')

        f1 = ttk.LabelFrame(control_frame, text="2. SVG Файл", padding=8)
        f1.pack(fill='x', pady=5)
        self.file_path = tk.StringVar()
        ttk.Entry(f1, textvariable=self.file_path, state='readonly').pack(fill='x', pady=2)
        ttk.Button(f1, text="📂 Открыть SVG", command=self.load_svg).pack(fill='x', pady=2)

        f2 = ttk.LabelFrame(control_frame, text="3. Масштаб и смещение", padding=8)
        f2.pack(fill='x', pady=5)

        ttk.Label(f2, text="Масштаб (%):").pack(anchor='w')
        self.scale_var = tk.IntVar(value=100)
        ttk.Scale(f2, from_=1, to=200, variable=self.scale_var, command=self.update_preview).pack(fill='x')

        ttk.Label(f2, text="Смещение X (мм):").pack(anchor='w')
        self.offset_x_var = tk.IntVar(value=10)
        ttk.Scale(f2, from_=0, to=self.x_max, variable=self.offset_x_var, command=self.update_preview).pack(fill='x')

        ttk.Label(f2, text="Смещение Y (мм):").pack(anchor='w')
        self.offset_y_var = tk.IntVar(value=10)
        ttk.Scale(f2, from_=0, to=self.y_max, variable=self.offset_y_var, command=self.update_preview).pack(fill='x')

        f3 = ttk.LabelFrame(control_frame, text="4. Автоматическая печать", padding=8)
        f3.pack(fill='x', pady=5)
        self.send_btn = ttk.Button(f3, text="▶ НАПЕЧАТАТЬ SVG НА РОБОТЕ", command=self.start_streaming)
        self.send_btn.pack(fill='x', pady=5)

        self.status_lbl = ttk.Label(control_frame, text="Статус: Ожидание подключения", font=('Arial', 10, 'bold'))
        self.status_lbl.pack(pady=5)

        preview_frame = ttk.LabelFrame(right_frame, text="Предпросмотр (500x180 мм)", padding=5)
        preview_frame.pack(fill='both', expand=True)

        self.figure = plt.Figure(figsize=(6, 4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, preview_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        console_frame = ttk.LabelFrame(right_frame, text="Консоль VEX Serial (Логи и ответы 'ok')", padding=5)
        console_frame.pack(fill='x', pady=(5, 0))
        self.console_text = tk.Text(console_frame, height=8, font=('Courier', 9))
        self.console_text.pack(fill='both', expand=True)

        self.init_preview()
        self.refresh_ports()

    def init_preview(self):
        self.ax.clear()
        self.ax.set_xlim(0, self.x_max)
        self.ax.set_ylim(0, self.y_max)
        self.ax.grid(True, alpha=0.3)
        rect = plt.Rectangle((0, 0), self.x_max, self.y_max, fill=False, edgecolor='red', linestyle='--')
        self.ax.add_patch(rect)
        self.ax.set_aspect('equal')
        self.canvas.draw()

    def load_svg(self):
        filename = filedialog.askopenfilename(filetypes=[('SVG файлы', '*.svg')])
        if not filename:
            return
        self.file_path.set(filename)
        try:
            tree = ET.parse(filename)
            root = tree.getroot()
            ns = {'svg': 'http://www.w3.org/2000/svg'}
            vb = root.get('viewBox')
            self.svg_height = float(vb.split()[3]) if vb else float(root.get('height', '100').replace('px', ''))

            self.current_paths = []
            for tag in ['path', 'circle', 'rect', 'polygon', 'polyline', 'line']:
                found = root.findall(f'.//svg:{tag}', ns) or root.findall(f'.//{tag}')
                for elem in found:
                    d = elem.get('d')
                    if tag == 'circle':
                        cx, cy, r = float(elem.get('cx', 0)), float(elem.get('cy', 0)), float(elem.get('r', 0))
                        d = f"M {cx - r} {cy} A {r} {r} 0 1 0 {cx + r} {cy} A {r} {r} 0 1 0 {cx - r} {cy}"
                    elif tag == 'rect':
                        x, y, w, h = float(elem.get('x', 0)), float(elem.get('y', 0)), float(
                            elem.get('width', 0)), float(elem.get('height', 0))
                        d = f"M {x} {y} L {x + w} {y} L {x + w} {y + h} L {x} {y + h} Z"
                    if d:
                        polys = parse_path(d).to_polygons()
                        for p in polys:
                            if len(p) > 0:
                                self.current_paths.append(Path(p))

            self.update_preview()
            self.log(f"SVG загружен. Контуров: {len(self.current_paths)}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось прочитать SVG: {e}")

    def update_preview(self, *args):
        scale = self.scale_var.get() / 100.0
        ox, oy = self.offset_x_var.get(), self.offset_y_var.get()

        self.ax.clear()
        self.ax.set_xlim(0, self.x_max)
        self.ax.set_ylim(0, self.y_max)
        self.ax.grid(True, alpha=0.3)
        rect = plt.Rectangle((0, 0), self.x_max, self.y_max, fill=False, edgecolor='red', linestyle='--')
        self.ax.add_patch(rect)

        self.raw_gcode = ["G28", "G00 X0 Y0", "M05"]

        for path in self.current_paths:
            verts = path.vertices.copy()
            if self.svg_height > 0:
                verts[:, 1] = self.svg_height - verts[:, 1]
            verts[:, 0] = verts[:, 0] * scale + ox
            verts[:, 1] = verts[:, 1] * scale + oy

            self.ax.add_patch(PathPatch(Path(verts), facecolor='none', edgecolor='blue', linewidth=1.2))

            sx = max(0.0, min(self.x_max, verts[0][0]))
            sy = max(0.0, min(self.y_max, verts[0][1]))
            self.raw_gcode.append(f"G00 X{sx:.1f} Y{sy:.1f}")
            self.raw_gcode.append("M03")

            for i in range(1, len(verts)):
                x = max(0.0, min(self.x_max, verts[i][0]))
                y = max(0.0, min(self.y_max, verts[i][1]))
                self.raw_gcode.append(f"G01 X{x:.1f} Y{y:.1f}")

            self.raw_gcode.append("M05")

        self.raw_gcode.append("G00 X0 Y0")
        self.ax.set_aspect('equal')
        self.canvas.draw()

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_combo['values'] = [p.device for p in ports]
        if ports:
            self.port_combo.set(ports[0].device)

    def connect_robot(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("Ошибка", "Выберите COM-порт")
            return

        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            self.serial_port = serial.Serial(port, 115200, timeout=1)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.serial_port.write(b"\n")
            time.sleep(0.1)

            self.status_lbl.config(text=f"Подключен к {port}", foreground="green")
            self.log(f"Открыт порт {port}. Проверьте реакцию отправкой тестовой команды.")

        except Exception as e:
            self.serial_port = None
            self.status_lbl.config(text="Ошибка подключения", foreground="red")
            messagebox.showerror("Ошибка порта", f"Не удалось открыть {port}:\n{e}")

    def send_single_command(self, cmd):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Ошибка", "Сначала нажмите 'Подключиться'!")
            return
        threading.Thread(target=self._send_single_thread, args=(cmd,), daemon=True).start()

    def send_custom_command(self):
        cmd = self.custom_cmd_var.get().strip()
        if cmd:
            self.send_single_command(cmd)

    def _send_single_thread(self, cmd):
        self.log(f"Тест → Отправка: {cmd}")
        try:
            self.serial_port.write((cmd + '\n').encode())

            start = time.time()
            resp_found = False
            while time.time() - start < 5.0:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.log(f"← Ответ робота: {line}")
                        if 'ok' in line.lower():
                            resp_found = True
                            break
                time.sleep(0.01)

            if not resp_found:
                self.log("⚠️ Робот принял команду, но не прислал 'ok'")
        except Exception as e:
            self.log(f"❌ Ошибка отправки: {e}")

    def start_streaming(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Ошибка", "Подключитесь к VEX V5!")
            return
        if not self.raw_gcode:
            messagebox.showwarning("Ошибка", "Загрузите SVG рисунок!")
            return

        self.is_sending = True
        self.send_btn.config(state='disabled')
        threading.Thread(target=self._stream_thread, daemon=True).start()

    def _stream_thread(self):
        total = len(self.raw_gcode)
        self.log(f"Старт потоковой печати ({total} команд)...")

        for i, cmd in enumerate(self.raw_gcode):
            if not self.is_sending:
                break

            self.serial_port.write((cmd + '\n').encode())
            self.log(f"[{i + 1}/{total}] → {cmd}")

            start_time = time.time()
            ok_received = False
            while time.time() - start_time < 10.0:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.log(f"   [RX]: {line}")
                        if 'ok' in line.lower():
                            ok_received = True
                            break
                time.sleep(0.01)

            if not ok_received:
                self.log("⚠️ Таймаут (нет 'ok'), продолжаем...")

        self.log("✅ Печать завершена!")
        self.root.after(0, lambda: self.send_btn.config(state='normal'))

    def log(self, msg):
        def _append():
            ts = datetime.now().strftime("%H:%M:%S")
            self.console_text.insert(tk.END, f"[{ts}] {msg}\n")
            self.console_text.see(tk.END)

        self.root.after(0, _append)


if __name__ == "__main__":
    root = tk.Tk()
    app = VEXPlotterController(root)
    root.mainloop()