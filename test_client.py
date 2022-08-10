import requests
import sys

import requests

from PIL import Image
import numpy as np

import base64
from io import BytesIO

def pil_to_base64(pil_img):
    im_file = BytesIO()
    pil_img.save(im_file, format="PNG")
    im_bytes = im_file.getvalue()  # im_bytes: image in binary format.
    im_b64 = base64.b64encode(im_bytes)

    return im_b64

img = Image.open("./output_raw/78.png")
bim = pil_to_base64(img)

print("TYPPEE", type(bim))

res = requests.post("http://localhost:5000/analyse", data=bim)
data = res.json()
print(data)