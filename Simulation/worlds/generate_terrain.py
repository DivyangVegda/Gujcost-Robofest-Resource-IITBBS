import math

try:
    from PIL import Image
    size = 257
    img = Image.new('L', (size, size))
    pixels = img.load()
    for y in range(size):
        for x in range(size):
            # Creates smooth rolling hills and valleys using sine waves
            val = int((math.sin(x/15.0) * math.cos(y/15.0)) * 50 + 128)
            pixels[x, y] = val
    img.save("/home/dnvegda/ardupilot_gazebo/worlds/terrain.png")
    print("Success: terrain.png generated using PIL!")
except ImportError:
    try:
        import cv2
        import numpy as np
        size = 257
        img = np.zeros((size, size), dtype=np.uint8)
        for y in range(size):
            for x in range(size):
                val = int((math.sin(x/15.0) * math.cos(y/15.0)) * 50 + 128)
                img[y, x] = val
        cv2.imwrite("/home/dnvegda/ardupilot_gazebo/worlds/terrain.png", img)
        print("Success: terrain.png generated using cv2!")
    except ImportError:
        print("Error: You need either Pillow or OpenCV installed.")
        print("Run 'pip install Pillow' and try again.")
