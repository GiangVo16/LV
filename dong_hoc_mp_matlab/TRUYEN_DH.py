import tkinter as tk
from tkinter import ttk
import socket

# === Thiết lập TCP client ===
SERVER_IP = "192.168.1.5"
SERVER_PORT = 8888

def send_tcp_message(message):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((SERVER_IP, SERVER_PORT))
            s.sendall(message.encode())
            print("Đã gửi:", message.strip())
    except Exception as e:
        print("Lỗi khi gửi TCP:", e)

# === Gửi động học thuận ===
def gui_dht():
    try:
        cmd = f'dht t0 {entry_t0.get()} t1 {entry_t1.get()} t2 {entry_t2.get()} t3 {entry_t3.get()}\n'
        send_tcp_message(cmd)
    except Exception as e:
        print("Lỗi DHT:", e)

# === Gửi động học nghịch ===
def gui_dhn():
    try:
        cmd = f'dhn x {entry_x.get()} y {entry_y.get()} z {entry_z.get()}\n'
        send_tcp_message(cmd)
    except Exception as e:
        print("Lỗi DHN:", e)

# === Giao diện GUI ===
root = tk.Tk()
root.title("Điều khiển Robot qua WiFi TCP/IP")
root.geometry("400x300")
root.resizable(False, False)

style = ttk.Style()
style.configure("TLabel", font=("Arial", 11))
style.configure("TEntry", font=("Arial", 11))
style.configure("TButton", font=("Arial", 11, "bold"))

# --- Khung DHT ---
frame_dht = ttk.LabelFrame(root, text="Động học thuận (DHT)")
frame_dht.pack(padx=10, pady=10, fill="x")

entry_t0 = ttk.Entry(frame_dht, width=8)
entry_t1 = ttk.Entry(frame_dht, width=8)
entry_t2 = ttk.Entry(frame_dht, width=8)
entry_t3 = ttk.Entry(frame_dht, width=8)

ttk.Label(frame_dht, text="t0").grid(row=0, column=0, padx=5, pady=5)
entry_t0.grid(row=0, column=1)
ttk.Label(frame_dht, text="t1").grid(row=0, column=2)
entry_t1.grid(row=0, column=3)

ttk.Label(frame_dht, text="t2").grid(row=1, column=0, padx=5, pady=5)
entry_t2.grid(row=1, column=1)
ttk.Label(frame_dht, text="t3").grid(row=1, column=2)
entry_t3.grid(row=1, column=3)

ttk.Button(frame_dht, text="Gửi DHT", command=gui_dht).grid(row=2, column=0, columnspan=4, pady=10)

# --- Khung DHN ---
frame_dhn = ttk.LabelFrame(root, text="Động học nghịch (DHN)")
frame_dhn.pack(padx=10, pady=10, fill="x")

entry_x = ttk.Entry(frame_dhn, width=8)
entry_y = ttk.Entry(frame_dhn, width=8)
entry_z = ttk.Entry(frame_dhn, width=8)

ttk.Label(frame_dhn, text="x").grid(row=0, column=0, padx=5, pady=5)
entry_x.grid(row=0, column=1)
ttk.Label(frame_dhn, text="y").grid(row=0, column=2)
entry_y.grid(row=0, column=3)
ttk.Label(frame_dhn, text="z").grid(row=1, column=0, padx=5, pady=5)
entry_z.grid(row=1, column=1)

ttk.Button(frame_dhn, text="Gửi DHN", command=gui_dhn).grid(row=2, column=0, columnspan=4, pady=10)

root.mainloop()
