import os
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

class MenuApp:

    def __init__(self, root):
        self.root = root
        self.root.title("Menu App")

        images_folder = "/home/jvox/Desktop/WS/img"

        for row in range(3):
            for col in range(2):
                frame = tk.Frame(root, padx=10, pady=10)
                frame.grid(row=row, column=col, sticky="nsew")

                image_files = [f for f in os.listdir(images_folder) if f.endswith(('.png', '.jpg', '.jpeg'))]
                
                if image_files:
                    image_index = row * 2 + col
                    if image_index < len(image_files):
                        image_path = os.path.join(images_folder, image_files[image_index])
                        image = Image.open(image_path)
                        image = image.resize((400, 300), Image.ANTIALIAS)
                        tk_image = ImageTk.PhotoImage(image)

                        label_image = tk.Label(frame, image=tk_image)
                        label_image.image = tk_image
                        label_image.grid(row=0, column=0, padx=5, pady=5)

                        label_name = tk.Label(frame, text=os.path.splitext(image_files[image_index])[0], font=("Helvetica", 12, "bold"))
                        label_name.grid(row=1, column=0)

                        label_price = tk.Label(frame, text="$10.00", font=("Helvetica", 10))
                        label_price.grid(row=2, column=0)

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    root = tk.Tk()
    app = MenuApp(root)
    app.run()

