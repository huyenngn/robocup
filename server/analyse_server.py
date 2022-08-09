from urllib.error import URLError
from flask import Flask, request
from PIL import Image
import torch
import json
import os

import base64
from io import BytesIO
import numpy as np

app = Flask(__name__)
TORCH_HUB_PATH = os.path.expanduser('~')+"/.cache/torch/hub/"
try:
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', device='cpu')
except URLError:
    print("use offline model (no network)")
    model = torch.hub.load(os.path.join(
        TORCH_HUB_PATH, 'ultralytics_yolov5_master'), 'custom', path='yolov5s.pt', source='local')
model.classes = [32]
model.max_det = 100
model.conf = 0.2


def base64_to_pil(b64_img):
    im_bytes = base64.b64decode(b64_img)  # im_bytes is a binary image
    im_file = BytesIO(im_bytes)  # convert image to file-like object
    img = Image.open(im_file)  # img is now PIL Image object
    return img


def generate_json_response(found, x=0, y=0, w=0, h=0):
    # generate json response
    return {'found': found, 'x': int(x), 'y': int(y), 'size': int((w + h) / 2)}


def check_color(area):
    a = area.mean(axis=(0, 1)).std() < 15
    return a


@app.route('/analyse', methods=['POST'])
def process_json():
    img = base64_to_pil(request.data)

    img_data = np.asarray(img)
    # call nn model
    results = model([img], size=240, augment=True)
    # get json format of model output
    json_outputs = json.loads(
        results.pandas().xyxy[0].to_json(orient="records"))
    # if model found one or more balls
    if len(json_outputs):
        score = 0
        out = generate_json_response(False)
        for json_output in json_outputs:
            x_p = np.array([int(np.round(json_output["xmin"])),
                           int(np.round(json_output["xmax"]))])
            y_p = np.array([int(np.round(json_output["ymin"])),
                           int(np.round(json_output["ymax"]))])
            h = np.abs(y_p[0] - y_p[1])
            w = np.abs(x_p[0] - x_p[1])
            # calculate center area extends
            h_ext = int(h / 8)
            w_ext = int(w / 8)
            # check if std of color is less than 15 in the center of the detected area (check main color is gray)
            if check_color(img_data[y_p[0] + h_ext:y_p[1] - h_ext, x_p[0] + w_ext:x_p[1] - w_ext]) and json_output[
                    "confidence"] > score:
                out = generate_json_response(True, x=np.round(x_p.mean()), y=np.round(y_p.mean()),
                                             w=w, h=h)
                score = json_output["confidence"]
        # return detection (max confidence)
        print(score)
        return out
    else:
        print("Not found")
        # no ball detected
        return generate_json_response(False)


if __name__ == '__main__':
    app.run()  # run our Flask app
