#!/usr/bin/env python3

import tkinter as tk

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Tkinter Frame Example")
        self.geometry("300x150")

        # Main frame
        main_frame = tk.Frame(self)
        main_frame.pack(padx=20, pady=20)

        # Input field
        self.entry = tk.Entry(main_frame, width=25)
        self.entry.pack(pady=5)

        # Submit button
        submit_button = tk.Button(main_frame, text="Submit", command=self.on_submit)
        submit_button.pack(pady=5)

        # Close button
        close_button = tk.Button(main_frame, text="Close", command=self.destroy)
        close_button.pack(pady=5)

    def on_submit(self):
        user_input = self.entry.get()
        print("User entered:", user_input)

if __name__ == "__main__":
    app = App()
    app.mainloop()

# class MainApplication(tk.Tk):
#     def __init__(self):
#         super().__init__()
#         self.title("Tkinter Frame Layout Example")
#         self.geometry("400x300")

#         # Create and pack the main frame
#         main_frame = MainFrame(self)
#         main_frame.pack(fill="both", expand=True)

# class MainFrame(tk.Frame):
#     def __init__(self, parent):
#         super().__init__(parent, bg='lightgrey')

#         # Create sub-frames
#         top_frame = tk.Frame(self, bg='skyblue', height=50)
#         middle_frame = tk.Frame(self, bg='white')
#         bottom_frame = tk.Frame(self, bg='lightgreen', height=50)

#         # Use pack, grid, or place to position sub-frames
#         top_frame.pack(fill="x")
#         middle_frame.pack(fill="both", expand=True)
#         bottom_frame.pack(fill="x")

#         # Add widgets to each frame
#         tk.Label(top_frame, text="Top Frame", bg='skyblue').pack(pady=10)
#         tk.Label(middle_frame, text="Middle Frame", bg='white').pack(pady=10)
#         tk.Button(bottom_frame, text="Click Me").pack(pady=10)

# if __name__ == "__main__":
#     app = MainApplication()
#     app.mainloop()

