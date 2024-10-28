import tensorrt as trt
import cv2
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
import os

class YOLOInt8Calibrator(trt.IInt8EntropyCalibrator2):
    def __init__(self, calibration_files, batch_size=8, input_shape=(3, 416, 416)):
        super(YOLOInt8Calibrator, self).__init__()
        self.batch_size = batch_size
        self.input_shape = input_shape
        self.calibration_files = calibration_files
        self.device_input = cuda.mem_alloc(trt.volume(input_shape) * batch_size * trt.float32.itemsize)
        self.current_index = 0

    def get_batch(self, names):
        if self.current_index + self.batch_size > len(self.calibration_files):
            return None
        batch = []
        for i in range(self.batch_size):
            img = cv2.imread(self.calibration_files[self.current_index + i])
            img = cv2.resize(img, (self.input_shape[2], self.input_shape[1]))
            img = img.astype(np.float32) / 255.0
            img = img.transpose(2, 0, 1)  # HWC to CHW
            batch.append(img)
        batch = np.ascontiguousarray(batch)
        cuda.memcpy_htod(self.device_input, batch)
        self.current_index += self.batch_size
        return [self.device_input]

    def get_batch_size(self):
        return self.batch_size

    def read_calibration_cache(self):
        return None

    def write_calibration_cache(self, cache):
        pass

def build_engine(cfg_file, weights_file, calibration_files, engine_file_path="yolov4-tiny-int8.trt"):
    TRT_LOGGER = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, TRT_LOGGER)
    config = builder.create_builder_config()
    config.max_workspace_size = 1 << 30  # Adjust based on available GPU memory

    # Enable INT8
    config.set_flag(trt.BuilderFlag.INT8)
    config.int8_calibrator = YOLOInt8Calibrator(calibration_files)

    # Convert the YOLO model
    with open(weights_file, "rb") as weights, open(cfg_file, "r") as cfg:
        if not parser.parse_from_file(cfg.read(), weights.read()):
            print("Failed to parse .cfg and .weights files!")
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None

    # Build and save the engine
    engine = builder.build_engine(network, config)
    with open(engine_file_path, "wb") as f:
        f.write(engine.serialize())
    return engine

# Usage
calibration_images = ["/path/to/calibration/images/img1.jpg", "/path/to/calibration/images/img2.jpg"]  # Update paths
cfg_path = "yolov4-tiny.cfg"
weights_path = "yolov4-tiny.weights"
engine_path = "yolov4-tiny-int8.trt"

engine = build_engine(cfg_path, weights_path, calibration_images, engine_path)
print("Engine created and saved to", engine_path)