#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, messagebox

class AppWithConfirmation:
    def __init__(self, root):
        self.root = root
        self.root.title("Window Closing with Confirmation")
        
        # Add some content
        label = ttk.Label(self.root, text="Do you want to close this window?")
        label.pack(pady=20)
        
        close_btn = ttk.Button(self.root, text="Close", command=self.confirm_close)
        close_btn.pack(pady=10)
        
        # Handle window close button (X)
        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
    
    def confirm_close(self):
        """Show confirmation dialog before closing"""
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = AppWithConfirmation(root)
    root.mainloop()