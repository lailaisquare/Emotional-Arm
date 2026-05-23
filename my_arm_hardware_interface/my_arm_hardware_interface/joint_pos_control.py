from arm_hardware_interface.motors.feetech import FeetechMotorsBus
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


def read_all_data(motors_bus, motor_names, data_name):
    data = {}
    for name in motor_names:
        data[name] = motors_bus.read(data_name, name)
    return data


def toggle_torque(motors_bus, motor_names, torque_state, sliders):
    new_state = 0 if torque_state.get() else 1
    for name in motor_names:
        motors_bus.write("Torque_Enable", new_state, name)
    torque_state.set(new_state)
    print("Torque Enabled" if new_state else "Torque Disabled")

    if new_state == 1:
        data = read_all_data(motors_bus, motor_names, "Present_Position")
        for name in motor_names:
            sliders[name].set(int(data[name][0]))


def update_plot(motors_bus, motor_names, data_name, lines, ax, data_history, sliders, torque_state):
    data = read_all_data(motors_bus, motor_names, data_name.get())

    for name in motor_names:
        data_history[name].append(data[name])
        if len(data_history[name]) > 100:
            data_history[name].pop(0)
        lines[name].set_ydata(data_history[name])
        lines[name].set_xdata(range(len(data_history[name])))

    ax.relim()
    ax.autoscale_view()
    global canvas
    canvas.draw()
    t = 10
    root.after(t, update_plot, motors_bus, motor_names, data_name, lines, ax, data_history, sliders, torque_state)


def on_data_menu_change(data_name, data_history):
    for name in data_history:
        data_history[name].clear()


def on_slider_change(motors_bus, motor_name, value):
    if motors_bus.is_connected:
        motors_bus.write("Goal_Position", int(value), motor_name)


def configure_motors(motors_bus, motor_names):
    for name in motor_names:
        motors_bus.write("Goal_Speed", 100, name)
        motors_bus.write("Acceleration", 50, name)


def main():
    motors_bus = FeetechMotorsBus(
        port="/dev/ttyARM0",
        motors={},
    )
    motors_bus.connect()

    try:
        motors_bus.motors = {"temp_motor": (0, "sts3215")}

        connected_indices = motors_bus.find_motor_indices(possible_ids=range(15))
        motor_names = [f"motor_{idx}" for idx in connected_indices]
        motors_bus.motors = {name: (idx, "sts3215") for name, idx in zip(motor_names, connected_indices)}

        configure_motors(motors_bus, motor_names)

        global root
        root = tk.Tk()
        root.title("Motor Data Viewer")

        torque_state = tk.IntVar(value=0)
        toggle_button = tk.Button(
            root, text="Toggle Torque", command=lambda: toggle_torque(motors_bus, motor_names, torque_state, sliders)
        )
        toggle_button.pack(pady=10)

        data_name = tk.StringVar(value="Present_Position")
        data_options = ["Present_Position", "Present_Speed", "Present_Load", "Present_Voltage", "Present_Temperature"]
        data_menu = ttk.Combobox(root, textvariable=data_name, values=data_options)
        data_menu.pack(pady=10)

        fig, ax = plt.subplots()
        lines = {}
        data_history = {}
        sliders = {}
        for name in motor_names:
            line, = ax.plot([], [], label=name)
            lines[name] = line
            data_history[name] = []
            slider = tk.Scale(root, from_=0, to=4095, orient=tk.HORIZONTAL, label=name, command=lambda value, name=name: on_slider_change(motors_bus, name, value))
            slider.pack(pady=5, fill=tk.X, expand=True)
            sliders[name] = slider
        ax.legend()

        global canvas
        canvas = FigureCanvasTkAgg(fig, master=root)
        canvas.get_tk_widget().pack(pady=20)

        data_menu.bind("<<ComboboxSelected>>", lambda event: on_data_menu_change(data_name, data_history))

        update_plot(motors_bus, motor_names, data_name, lines, ax, data_history, sliders, torque_state)

        root.mainloop()

    finally:
        motors_bus.disconnect()


if __name__ == "__main__":
    main()
