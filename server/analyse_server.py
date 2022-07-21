from flask import Flask, request
from PIL import Image

import base64
from io import BytesIO
app = Flask(__name__)

def base64_to_pil(b64_img):
    im_bytes = base64.b64decode(b64_img)   # im_bytes is a binary image
    im_file = BytesIO(im_bytes)  # convert image to file-like object
    img = Image.open(im_file)   # img is now PIL Image object
    return img

@app.route('/analyse', methods=['POST'])
def process_json():
    content_type = request.headers.get('Content-Type')
    if (content_type == 'application/json'):
        json = request.json
        img = base64_to_pil(json["b64_img"])

        # TODO: Add net call here

        return {'found': False, 'x': 0, 'y': 0, 'w': 0, 'h': 0}
        return json
    else:
        return 'Content-Type not supported!'

if __name__ == '__main__':
    app.run()  # run our Flask app  