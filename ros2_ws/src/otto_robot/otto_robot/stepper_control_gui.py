#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox

from std_msgs.msg import Int32

class StepperControlGUI(Node):
    def __init__(self, root):
        super().__init__('stepper_control_gui')
        
        # self.subscription = self.create_subscription(Int32, 'stepper_position', self.position_callback, 10)
        # self.publisher = self.create_publisher(Int32, 'stepper_target', 10)
        
        self.subscription = self.create_subscription(Int32, 'stepper_feedback', self.position_callback, 10)
        self.publisher = self.create_publisher(Int32, 'stepper_control', 10)
        
        # Setup the Tkinter GUI
        self.root = root
        self.root.title("Stepper Control")
        self.root.geometry("420x80")
        
        self.entryInt = tk.IntVar()
        
        self.label = tk.Label(self.root, text="Commanded to move to : ").grid(row=0, column=0)
        self.entry = tk.Entry(self.root, text="100", textvariable=self.entryInt).grid(row=0, column=1)#ทำไงให้ใส่แค่ เลข 0-xxx        
        self.button = tk.Button(self.root, text="Stepper", command=self.move_to_position).grid(row=0, column=2)

        
        self.close_btn = tk.Button(self.root, text="Close", command=self.confirm_close).grid(row=2, column=0, columnspan=3)  

        self.root.protocol("WM_DELETE_WINDOW", self.confirm_close)
        
    def move_to_position(self, target=100):
        target = self.entryInt.get()
        msg = Int32()
        msg.data = target
        self.publisher.publish(msg)
        self.get_logger().info(f"Commanded to move to: {target}")

    def position_callback(self, msg):
        self.current_position = msg.data
        self.get_logger().info(f"Current position: {self.current_position}")
     
    def confirm_close(self):
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            print("Closing the application")
            self.root.destroy()
            self.root.mainloop()
        
def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = StepperControlGUI(root)
    
    # Tkinter main loop inside ROS spin 
    while rclpy.ok():
        rclpy.spin_once(gui, timeout_sec=0.1)
        gui.root.update_idletasks()
        gui.root.update()

    gui.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32

# class StepperControlGUI(Node):
#     def __init__(self):
#         super().__init__('stepper_control_gui')
        
#         self.subscription = self.create_subscription(Int32, 'stepper_position', self.position_callback, 10)
#         self.publisher = self.create_publisher(Int32, 'stepper_target', 10)
        
#         self.current_position = 0
#         self.get_logger().info("Stepper Controller Node Started")
        
#     def position_callback(self, msg):
#         self.current_position = msg.data
#         self.get_logger().info(f"Current position: {self.current_position}")
    
#     def move_to_position(self, target):
#         msg = Int32()
#         msg.data = target
#         self.publisher.publish(msg)
#         self.get_logger().info(f"Commanded to move to: {target}")

# def main(args=None):
#     rclpy.init(args=args)
#     controller = StepperControlGUI()
    
#     try:
#         while rclpy.ok():
#             # Example movement sequence
#             target = int(input("Enter target position (steps): "))
#             controller.move_to_position(target)
            
#             # Spin once to process callbacks
#             rclpy.spin_once(controller, timeout_sec=0.1)
#     except KeyboardInterrupt:
#         pass
    
#     controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()