import tkinter as tk
from tkinter import font, ttk

class SliderController:
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("Parameter Adjustment")
        
        # Define a larger font for the sliders
        large_font = font.Font(size=16, weight="bold")

        # Calculating the window height based on the number of sliders and spacing
        num_sliders = 12  # Total sliders including additional ones and method selector
        slider_height = 140  # Approximate height per slider including padding
        window_height = (num_sliders + 3) * slider_height + 50  # +3 for new master sliders
        
        # Set window width and position to the right of the screen
        screen_width = self.root.winfo_screenwidth()
        window_width = 600
        x_offset = screen_width - window_width - 50  # Offset from the right edge
        self.root.geometry(f"{window_width}x{window_height}+{x_offset}+50")

        # Master Slider for Position Gains
        self.pos_gains_master_slider = self.create_slider("Master Pos Gains", 0, 10, 0.1, self.controller.pos_gains[0], self.update_pos_gains_master)

        # Individual Position Gains Sliders
        self.pos_gains_sliders = []
        for i in range(3):
            slider = tk.Scale(self.root, from_=0, to=10, resolution=0.1, label=f"Pos Gain {i+1}",
                              orient="horizontal", length=500, command=self.update_pos_gains)
            slider.config(font=large_font)
            slider.set(self.controller.pos_gains[i])
            slider.pack(pady=10)
            self.pos_gains_sliders.append(slider)

        # Master Slider for Orientation Gains
        self.ori_gains_master_slider = self.create_slider("Master Ori Gains", 0, 10, 0.1, self.controller.ori_gains[0], self.update_ori_gains_master)

        # Individual Orientation Gains Sliders
        self.ori_gains_sliders = []
        for i in range(3):
            slider = tk.Scale(self.root, from_=0, to=10, resolution=0.1, label=f"Ori Gain {i+1}",
                              orient="horizontal", length=500, command=self.update_ori_gains)
            slider.config(font=large_font)
            slider.set(self.controller.ori_gains[i])
            slider.pack(pady=10)
            self.ori_gains_sliders.append(slider)

        # Additional sliders for various parameters
        self.damping_ratio_slider = self.create_slider("Damping Ratio", 0, 5, 0.1, self.controller.damping_ratio, self.update_damping_ratio)
        self.error_tolerance_pos_slider = self.create_slider("Error Tolerance Pos", 0, 0.1, 0.001, self.controller.error_tolerance_pos, self.update_error_tolerance_pos)
        self.error_tolerance_ori_slider = self.create_slider("Error Tolerance Ori", 0, 0.1, 0.001, self.controller.error_tolerance_ori, self.update_error_tolerance_ori)
        self.max_pos_error_slider = self.create_slider("Max Pos Error", 0, 5, 0.1, self.controller.max_pos_error or 0, self.update_max_pos_error)
        self.max_ori_error_slider = self.create_slider("Max Ori Error", 0, 5, 0.1, self.controller.max_ori_error or 0, self.update_max_ori_error)
        self.integration_dt_slider = self.create_slider("Integration DT", 0, 1, 0.01, self.controller.integration_dt, self.update_integration_dt)

        # Method Selector
        self.method_var = tk.StringVar(value=self.controller.method)
        method_selector = ttk.Combobox(self.root, textvariable=self.method_var, values=["dynamics", "pinv", "svd", "trans", "dls"], font=large_font)
        method_selector.bind("<<ComboboxSelected>>", self.update_method)
        method_selector.pack(pady=20)

    def create_slider(self, label, from_, to, resolution, initial, command):
        slider = tk.Scale(self.root, from_=from_, to=to, resolution=resolution, label=label,
                          orient="horizontal", length=500, command=command)
        slider.config(font=font.Font(size=16, weight="bold"))
        slider.set(initial)
        slider.pack(pady=10)
        return slider

    # Update methods for each parameter
    def update_pos_gains_master(self, _):
        master_value = self.pos_gains_master_slider.get()
        for slider in self.pos_gains_sliders:
            slider.set(master_value)  # Sync each individual slider with the master value
        self.update_pos_gains(None)  # Update controller parameters

    def update_pos_gains(self, _):
        new_pos_gains = tuple(slider.get() for slider in self.pos_gains_sliders)
        self.controller.set_parameters(pos_gains=new_pos_gains)

    def update_ori_gains_master(self, _):
        master_value = self.ori_gains_master_slider.get()
        for slider in self.ori_gains_sliders:
            slider.set(master_value)  # Sync each individual slider with the master value
        self.update_ori_gains(None)  # Update controller parameters

    def update_ori_gains(self, _):
        new_ori_gains = tuple(slider.get() for slider in self.ori_gains_sliders)
        self.controller.set_parameters(ori_gains=new_ori_gains)

    def update_damping_ratio(self, _):
        new_damping_ratio = self.damping_ratio_slider.get()
        self.controller.set_parameters(damping_ratio=new_damping_ratio)

    def update_error_tolerance_pos(self, _):
        new_error_tolerance_pos = self.error_tolerance_pos_slider.get()
        self.controller.set_parameters(error_tolerance_pos=new_error_tolerance_pos)

    def update_error_tolerance_ori(self, _):
        new_error_tolerance_ori = self.error_tolerance_ori_slider.get()
        self.controller.set_parameters(error_tolerance_ori=new_error_tolerance_ori)

    def update_max_pos_error(self, _):
        new_max_pos_error = self.max_pos_error_slider.get()
        self.controller.set_parameters(max_pos_error=new_max_pos_error)

    def update_max_ori_error(self, _):
        new_max_ori_error = self.max_ori_error_slider.get()
        self.controller.set_parameters(max_ori_error=new_max_ori_error)

    def update_integration_dt(self, _):
        new_integration_dt = self.integration_dt_slider.get()
        self.controller.integration_dt = new_integration_dt

    def update_method(self, event):
        selected_method = self.method_var.get()
        self.controller.set_parameters(method=selected_method)
