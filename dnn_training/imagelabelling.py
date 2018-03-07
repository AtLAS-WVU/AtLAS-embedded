"""
This file will be used to help with labelling images we gather for neural network training.
To run this file, simply do
    python imagelabelling.py
"""

import tkinter as tk
from PIL import Image, ImageTk
import numpy as np

# This is necessary because pims uses Matplotlib, and Matplotlib interferes with tk and causes a crash
# if I don't do this.
import matplotlib
matplotlib.use("TkAgg")

import pims

# This will only run once, to make sure ffmpeg is downloaded. If it's already downloaded, it does nothing.
import imageio
imageio.plugins.ffmpeg.download()

# Height, Width
IMAGE_RESOLUTION = [1080//2, 1920//2]

MASK_OPACITY = 255


class Gui:

    def __init__(self):
        self.canvas_width = IMAGE_RESOLUTION[1]
        self.canvas_height = IMAGE_RESOLUTION[0]

        self.frame = tk.Frame(tk.Tk())
        self.frame.grid(row=0, column=0)

        tk.Label(master=self.frame, text="Usage: Left click to draw, right click to erase. "
                                         "Scroll to adjust brush size.").grid(row=0, column=0, columnspan=5)

        self.canvas = tk.Canvas(master=self.frame, bg="white", height=self.canvas_height, width=self.canvas_width)
        self.canvas.grid(row=1, column=0, columnspan=5)

        # Setup various callbacks for mouse events on the canvas
        self.canvas.bind("<Button-1>", self.left_click_down_callback)
        self.canvas.bind("<ButtonRelease-1>", self.left_click_up_callback)
        self.canvas.bind("<B1-Motion>", self.left_click_drag_callback)
        self.canvas.bind("<Button-2>", self.right_click_down_callback)
        self.canvas.bind("<ButtonRelease-2>", self.right_click_up_callback)
        self.canvas.bind("<B2-Motion>", self.right_click_drag_callback)
        self.canvas.bind("<MouseWheel>", self.mouse_wheel_callback)
        self.canvas.bind("<Motion>", self.mouse_move_callback)

        self.brush_handle = None

        # Buttons at bottom of window
        tk.Button(master=self.frame, text="<<", command=self.prev_frame).grid(row=2, column=0, sticky=tk.W)
        tk.Button(master=self.frame, text=">>", command=self.next_frame).grid(row=2, column=1, sticky=tk.W)
        tk.Button(master=self.frame, text="Clear current frame", command=self.clear_callback).grid(row=2, column=2,
                                                                                                   sticky=tk.W)
        # Checkbox for copying mask to next frame
        self.copy_mask_var = tk.IntVar()
        tk.Checkbutton(master=self.frame, text="Copy mask to next frame",
                       variable=self.copy_mask_var).grid(row=3, column=0, sticky=tk.W)

        # String containing the file name of the current video
        self.video_file = None
        # String containing the file name where the annotations are stored
        self.annotation_file = None
        # An instance of PIL.ImageTK.PhotoImage. This must be kept around, to prevent it from being garbage collected.
        # So say the elders of tkinter
        self.photo = None
        # Handle to the image object on the canvas, so that it can be deleted and replaced later
        self.photo_handle = None
        # An instance of PIL.ImageTK.PhotoImage, created from the image mask. Must be kept around for the same
        # reason as self.photo
        self.mask_photo = None
        # Handle to the mask image object on the canvas, so it can be deleted and redrawn as needed
        self.mask_photo_handle = None
        # The sequence of images being annoted, taken from pims.Video(...)
        self.image_seq = None
        # The number of the current frame being annotated
        self.current_frame = None

        # A numpy array representing the mask over the image. It should be of shape [num frames, height, width, 4]
        self.image_mask = None

        # Used to keep track of whether or not I have to save the file when I change frames.
        self.mask_modified = False

        # TODO: Remove this
        self.load_image_sequence("testvid.mov")

        # Radius of the mask brush, in pixels
        self.brush_radius = 15

    def load_image_sequence(self, filename):
        """
        Loads a video file into memory, and displays the first frame in the canvas.
        This function also checks to see if there is a corresponding annotation file available, and loads it if so.
        Finally, it navigates to the first un-annotated frame, so that if this video was already in progress, you
        can just pick up where you left off.
        :param filename: Name of video file to load
        :return: None
        """
        self.video_file = filename
        # This just finds the corresponding annotation file, a .npy file containing a numpy array.
        annotation_file = filename.split(".")
        annotation_file[-1] = "npy"
        self.annotation_file = ".".join(annotation_file)

        # Actually load the video
        self.image_seq = pims.Video(filename)

        # Create an annotation mask.
        # The mask is of shape [num_frames, height, width, 4], because it represents a series of RGBA images.
        # For each frame of video, the corresponding frame of the image mask is overlaid on top of the video.
        self.image_mask = np.zeros([len(self.image_seq), IMAGE_RESOLUTION[0], IMAGE_RESOLUTION[1], 4],
                                   dtype=np.uint8)
        # Turn the red channel of the entire mask all the way up, so it shows up as red in the canvas
        self.image_mask[:, :, :, 0] = 255

        try:
            # The mask is stored on disk as an array of shape [num_frames, height, width] and type float32.
            # In memory, it is shape [num_frames, height, width, 4] and type uint8, to represent an image.
            # This copies from the former to the latter.
            mask = np.load(self.annotation_file).astype(np.uint8)
            self.image_mask[:, :, :, 3] = mask * 255
            # Calculate an array of all frames that have been annotated: That is, all frames that have ANY nonzero
            # values in the mask.
            finished_frames = np.nonzero(np.any(self.image_mask[:, :, :, 3] > 0, axis=(1, 2)))[0]
            self.current_frame = finished_frames[-1]
            print("Successfully loaded annotation file")
        except (FileNotFoundError, OSError):  # OSError is if the file is not the correct format.
            print("No existing annotation file found. Starting from scratch.")
            self.current_frame = -1

        # Get the ball rolling by advancing to the next frame
        self.next_frame()
        print("Loaded image sequence from file {}. Image resolution: {}x{}. Number of frames: {}".format(
            filename, self.image_seq[0].shape[0], self.image_seq[0].shape[1], len(self.image_seq)
        ))

    def next_frame(self):
        """
        This function advances to the next frame of video. This entails:
         - Clearing the canvas and displaying the next frame of video
         - If the checkbox is checked, copy the mask over from the last frame to the next
         - Save the annotations to a file, if needed
        :return: None
        """
        if self.current_frame < len(self.image_seq) - 1:
            self.current_frame += 1
            self.show_image(self.current_frame)
            if self.mask_modified:
                np.save(self.annotation_file, self.get_final_mask())
                self.mask_modified = False
                print("Saved annotations to {}".format(self.annotation_file))
            if self.copy_mask_var.get() == 1 and np.all(self.image_mask[self.current_frame, :, :, 3] == 0):
                self.image_mask[self.current_frame, :, :, 3] = self.image_mask[self.current_frame - 1, :, :, 3]
                self.mask_modified = True

            self.display_mask()

            print("Now on frame {} out of {}.".format(self.current_frame, len(self.image_seq) - 1))

    def prev_frame(self):
        """
        This function retreats to the previous frame of video. This entails:
         - Clearing the canvas and displaying the next frame of video
\         - Save the annotations to a file, if needed
        :return: None
        """
        if self.current_frame > 0:
            self.current_frame -= 1
            self.show_image(self.current_frame)
            if self.mask_modified:
                np.save(self.annotation_file, self.get_final_mask())
                self.mask_modified = False
                print("Saved annotations to {}".format(self.annotation_file))

            self.display_mask()

            print("Now on frame {} out of {}.".format(self.current_frame, len(self.image_seq) - 1))

    def mainloop(self):
        """
        This needs to be called once to enter tk into its main loop.
        :return:
        """
        self.frame.master.mainloop()

    def show_image(self, frame_num):
        """
        Show the specified frame of video and annotations on the canvas.
        This function looks into self.video_seq to find the frame to display, so this must be initialized.
        :param frame_num: Number of the frame to display
        :return: None
        """
        if self.photo_handle is not None:
            self.canvas.delete(self.photo_handle)
        self.photo = Image.fromarray(self.image_seq[frame_num])
        self.photo = ImageTk.PhotoImage(self.photo)
        self.canvas.create_image([IMAGE_RESOLUTION[1] / 2, IMAGE_RESOLUTION[0] / 2], image=self.photo)

    def display_mask(self):
        """
        Show the annotation mask of the current frame on the canvas.
        Uses self.current_frame to determine which mask to show.
        :return: None
        """
        if self.mask_photo_handle is not None:
            self.canvas.delete(self.mask_photo_handle)
        self.mask_photo = Image.fromarray(self.image_mask[self.current_frame, ...], mode="RGBA")
        self.mask_photo = ImageTk.PhotoImage(self.mask_photo)
        self.mask_photo_handle = self.canvas.create_image([IMAGE_RESOLUTION[1] / 2, IMAGE_RESOLUTION[0] / 2],
                                                          image=self.mask_photo)

    def get_final_mask(self) -> np.ndarray:
        """
        The mask in memory is stored as a series of integer RGBA images.
        I want to save it to disk as a series of float32 single-channel images.
        This does the conversion.
        :return:
        """
        mask = (self.image_mask[..., 3] > 0)
        return mask.astype(np.uint8)

    def left_click_down_callback(self, event):
        # print("Left click down at ({}, {})".format(event.x, event.y))
        self.left_click_drag_callback(event)

    def left_click_up_callback(self, event):
        # print("Left click up at ({}, {})".format(event.x, event.y))
        pass

    def right_click_down_callback(self, event):
        # print("Right click at ({}, {})".format(event.x, event.y))
        self.right_click_drag_callback(event)

    def right_click_up_callback(self, event):
        # print("Right click at ({}, {})".format(event.x, event.y))
        pass

    def left_click_drag_callback(self, event):
        # print("Left click drag to ({}, {})".format(event.x, event.y))
        # Draw a circle in the mask
        for x in range(event.x - self.brush_radius, event.x + self.brush_radius):
            for y in range(event.y - self.brush_radius, event.y + self.brush_radius):
                if (x - event.x) ** 2 + (y - event.y) ** 2 < self.brush_radius ** 2 \
                        and 0 <= x < IMAGE_RESOLUTION[1] and 0 <= y < IMAGE_RESOLUTION[0]:
                    self.image_mask[self.current_frame, y, x, 3] = MASK_OPACITY
        self.display_mask()
        self.draw_brush(event.x, event.y)
        self.mask_modified = True

    def right_click_drag_callback(self, event):
        # print("Right click drag to ({}, {})".format(event.x, event.y))
        # Erase a circle from the mask
        for x in range(event.x - self.brush_radius, event.x + self.brush_radius):
            for y in range(event.y - self.brush_radius, event.y + self.brush_radius):
                if (x - event.x) ** 2 + (y - event.y) ** 2 < self.brush_radius ** 2 \
                        and 0 <= x < IMAGE_RESOLUTION[1] and 0 <= y < IMAGE_RESOLUTION[0]:
                    self.image_mask[self.current_frame, y, x, 3] = 0
        self.display_mask()
        self.draw_brush(event.x, event.y)
        self.mask_modified = True

    def draw_brush(self, x, y):
        """
        Draw a blue circle representing the paintbrush.
        :param x: X-coordinate of the mouse in the canvas
        :param y: Y-coordinate of the mouse in the canvas
        :return: None
        """
        if self.brush_handle is None:
            self.brush_handle = self.canvas.create_oval([-self.brush_radius, -self.brush_radius,
                                                         self.brush_radius, self.brush_radius], fill='blue')
        self.canvas.coords(self.brush_handle, [x - self.brush_radius, y - self.brush_radius,
                                               x + self.brush_radius, y + self.brush_radius])
        # This raises the oval up to the top layer so that it is definitely drawn over the mask
        self.canvas.tag_raise(self.brush_handle)

    def mouse_wheel_callback(self, event):
        # print("Mouse wheel! {}".format(event.delta))
        self.brush_radius += event.delta
        if self.brush_radius < 1:
            self.brush_radius = 1
        elif self.brush_radius > 500:
            self.brush_radius = 500
        self.draw_brush(event.x, event.y)

    def mouse_move_callback(self, event):
        # print("Mouse move: ({}, {})".format(event.x, event.y))
        self.draw_brush(event.x, event.y)

    def clear_callback(self):
        self.image_mask[self.current_frame, :, :, 3] = 0
        self.display_mask()


gui = Gui()

# There is a glitch where inertial scrolling on Mac causes weird problems with tk. This is a nice kludge to fix that.
while True:
    try:
        gui.mainloop()
        break
    except UnicodeDecodeError:
        pass
