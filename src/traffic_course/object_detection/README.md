## Object Detection with YOLO

A demo use YOLOv3 to detect objects in carla.

### Requisites

+ `pip install -r requirements.txt`
+ if you need GPU to accelerate, please install nvidia-cuda-toolkit.

### How to run?

Firstly, let's add some live to the city, open a new terminal window and execute:

`python3 spawn_npc.py -n 80`

> NOTE: you can find spawn_npc.py in `PythonAPI` directory.

**object detection**

`python3 object_detection.py` 

**object detection with gpu**

1. `pip install yolo34py-gpu`
2. `python3 object_detection_gpu.py`
