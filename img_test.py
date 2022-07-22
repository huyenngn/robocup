import base64
from io import BytesIO
from PIL import Image

def pil_to_base64(pil_img):
    im_file = BytesIO()
    pil_img.save(im_file, format="PNG")
    im_bytes = im_file.getvalue()  # im_bytes: image in binary format.
    im_b64 = base64.b64encode(im_bytes)

    return im_b64

def base64_to_pil(b64_img):
    im_bytes = base64.b64decode(b64_img)   # im_bytes is a binary image
    im_file = BytesIO(im_bytes)  # convert image to file-like object
    img = Image.open(im_file)   # img is now PIL Image object
    return img


img = Image.open('output_raw/1.png')
b = pil_to_base64(img)

img2 = base64_to_pil(b)
img2.save("./test.png", "PNG")