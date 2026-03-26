import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


ARUCO_DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_")
}


def get_marker_corners_3d(marker_length: float) -> np.ndarray:
    half = marker_length / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )


def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> np.ndarray:
    trace = np.trace(rotation_matrix)

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        s = math.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s = math.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2.0
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        qz = 0.25 * s

    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    raise AttributeError("OpenCV ArUco detector parameters API is unavailable")


class OpenCVArucoDetector:
    def __init__(self, aruco_dict, detector_params) -> None:
        self._dictionary = aruco_dict
        self._params = detector_params
        self._detector = None

        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    def detect_markers(self, frame):
        if self._detector is not None:
            return self._detector.detectMarkers(frame)
        return cv2.aruco.detectMarkers(frame, self._dictionary, parameters=self._params)


class PoseKalmanFilter:
    def __init__(self, dt: float, process_noise: float, measurement_noise: float) -> None:
        self.translation_filter = self._create_filter(dt, process_noise, measurement_noise)
        self.rotation_filter = self._create_filter(dt, process_noise, measurement_noise)
        self.initialized = False

    def _create_filter(self, dt: float, process_noise: float, measurement_noise: float):
        kalman = cv2.KalmanFilter(6, 3)
        kalman.transitionMatrix = np.array(
            [
                [1.0, 0.0, 0.0, dt, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, dt, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, dt],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        kalman.measurementMatrix = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            ],
            dtype=np.float32,
        )
        kalman.processNoiseCov = np.eye(6, dtype=np.float32) * process_noise
        kalman.measurementNoiseCov = np.eye(3, dtype=np.float32) * measurement_noise
        kalman.errorCovPost = np.eye(6, dtype=np.float32)
        return kalman

    def _initialize_filter(self, kalman, measurement: np.ndarray) -> None:
        kalman.statePost = np.array(
            [
                [measurement[0, 0]],
                [measurement[1, 0]],
                [measurement[2, 0]],
                [0.0],
                [0.0],
                [0.0],
            ],
            dtype=np.float32,
        )
        kalman.statePre = kalman.statePost.copy()

    def update(self, rvec: np.ndarray, tvec: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        rotation_measurement = rvec.reshape(3, 1).astype(np.float32)
        translation_measurement = tvec.reshape(3, 1).astype(np.float32)

        if not self.initialized:
            self._initialize_filter(self.rotation_filter, rotation_measurement)
            self._initialize_filter(self.translation_filter, translation_measurement)
            self.initialized = True
        else:
            self.rotation_filter.predict()
            self.translation_filter.predict()

        filtered_rotation = self.rotation_filter.correct(rotation_measurement)[:3]
        filtered_translation = self.translation_filter.correct(translation_measurement)[:3]
        return filtered_rotation.reshape(3, 1), filtered_translation.reshape(3, 1)


class ArucoTfNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_tf_node")

        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("camera_calibration_file", "")
        self.declare_parameter("detected_ids_topic", "/aruco/detected_ids")
        self.declare_parameter("marker_length", 0.05)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("fallback_fx", 800.0)
        self.declare_parameter("fallback_fy", 800.0)
        self.declare_parameter("capture_width", 0)
        self.declare_parameter("capture_height", 0)
        self.declare_parameter("capture_fps", 30.0)
        self.declare_parameter("processing_scale", 1.0)
        self.declare_parameter("frame_skip", 0)
        self.declare_parameter("use_grayscale", True)
        self.declare_parameter("enable_kalman_filter", True)
        self.declare_parameter("kalman_process_noise", 1e-4)
        self.declare_parameter("kalman_measurement_noise", 5e-3)

        video_device_param = self.get_parameter("video_device").get_parameter_value().string_value
        dictionary_name = self.get_parameter("aruco_dictionary").get_parameter_value().string_value
        self.camera_frame_override = self.get_parameter("camera_frame").get_parameter_value().string_value
        calibration_file = self.get_parameter("camera_calibration_file").get_parameter_value().string_value
        detected_ids_topic = self.get_parameter("detected_ids_topic").get_parameter_value().string_value
        self.marker_length = self.get_parameter("marker_length").get_parameter_value().double_value
        self.fallback_fx = self.get_parameter("fallback_fx").get_parameter_value().double_value
        self.fallback_fy = self.get_parameter("fallback_fy").get_parameter_value().double_value
        capture_width = self.get_parameter("capture_width").get_parameter_value().integer_value
        capture_height = self.get_parameter("capture_height").get_parameter_value().integer_value
        capture_fps = self.get_parameter("capture_fps").get_parameter_value().double_value
        self.capture_fps = capture_fps
        self.processing_scale = self.get_parameter("processing_scale").get_parameter_value().double_value
        self.frame_skip = self.get_parameter("frame_skip").get_parameter_value().integer_value
        self.use_grayscale = self.get_parameter("use_grayscale").get_parameter_value().bool_value
        self.enable_kalman_filter = self.get_parameter("enable_kalman_filter").get_parameter_value().bool_value
        self.kalman_process_noise = self.get_parameter("kalman_process_noise").get_parameter_value().double_value
        self.kalman_measurement_noise = self.get_parameter("kalman_measurement_noise").get_parameter_value().double_value

        if self.processing_scale <= 0.0 or self.processing_scale > 1.0:
            raise ValueError("processing_scale must be in the range (0.0, 1.0].")
        if self.frame_skip < 0:
            raise ValueError("frame_skip must be greater than or equal to 0.")
        if self.kalman_process_noise <= 0.0 or self.kalman_measurement_noise <= 0.0:
            raise ValueError("Kalman noise parameters must be greater than 0.")

        if dictionary_name not in ARUCO_DICTIONARIES:
            valid = ", ".join(sorted(ARUCO_DICTIONARIES))
            raise ValueError(f"Unsupported ArUco dictionary '{dictionary_name}'. Valid values: {valid}")

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[dictionary_name])
        detector_params = create_detector_parameters()
        self.detector = OpenCVArucoDetector(aruco_dict, detector_params)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.detected_ids_pub = self.create_publisher(Int32MultiArray, detected_ids_topic, 10)

        self.object_points = get_marker_corners_3d(self.marker_length)
        self.calibrated_camera_matrix = None
        self.calibrated_dist_coeffs = None
        self.frame_counter = 0
        self.pose_filters = {}
        if calibration_file:
            self.calibrated_camera_matrix, self.calibrated_dist_coeffs = self.load_calibration(calibration_file)

        self.capture = self.open_capture(video_device_param)

        if capture_width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(capture_width))
        if capture_height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(capture_height))
        if capture_fps > 0.0:
            self.capture.set(cv2.CAP_PROP_FPS, capture_fps)

        timer_period = 1.0 / capture_fps if capture_fps > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

        self.get_logger().info(
            f"Publishing TF from video device '{video_device_param}' with parent frame '{self.camera_frame_override}'."
        )
        self.get_logger().info(f"Publishing detected marker IDs on '{detected_ids_topic}'.")
        if calibration_file:
            self.get_logger().info(f"Using calibration file '{calibration_file}'.")
        else:
            self.get_logger().info("No calibration file provided; assuming the camera image is already corrected.")
        self.get_logger().info(
            f"capture_fps={capture_fps}, processing_scale={self.processing_scale}, "
            f"frame_skip={self.frame_skip}, use_grayscale={self.use_grayscale}, "
            f"enable_kalman_filter={self.enable_kalman_filter}"
        )

    def open_capture(self, video_device: str) -> cv2.VideoCapture:
        device = int(video_device) if video_device.isdigit() else video_device
        capture = cv2.VideoCapture(device)
        if not capture.isOpened():
            raise RuntimeError(f"Could not open video device '{video_device}'")
        return capture

    def load_calibration(self, calibration_file: str) -> tuple[np.ndarray, np.ndarray]:
        calibration_path = Path(calibration_file)
        if not calibration_path.is_file():
            raise FileNotFoundError(f"Calibration file '{calibration_file}' does not exist")

        storage = cv2.FileStorage(str(calibration_path), cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            raise RuntimeError(f"Could not read calibration file '{calibration_file}'")

        try:
            camera_matrix = storage.getNode("camera_matrix").mat()
            dist_coeffs = storage.getNode("dist_coeff").mat()
            if dist_coeffs is None:
                dist_coeffs = storage.getNode("distortion_coefficients").mat()
            if dist_coeffs is None:
                dist_coeffs = storage.getNode("dist_coeffs").mat()
        finally:
            storage.release()

        if camera_matrix is None:
            raise ValueError(
                f"Calibration file '{calibration_file}' is missing 'camera_matrix'"
            )
        if dist_coeffs is None:
            raise ValueError(
                f"Calibration file '{calibration_file}' is missing distortion coefficients"
            )

        return np.array(camera_matrix, dtype=np.float32), np.array(dist_coeffs, dtype=np.float32)

    def build_fallback_camera_model(self, image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        height, width = image.shape[:2]
        camera_matrix = np.array(
            [
                [self.fallback_fx, 0.0, width / 2.0],
                [0.0, self.fallback_fy, height / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        return camera_matrix, dist_coeffs

    def scale_camera_model(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        scale: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        if scale == 1.0:
            return camera_matrix, dist_coeffs

        scaled_camera_matrix = np.array(camera_matrix, dtype=np.float32, copy=True)
        scaled_camera_matrix[0, 0] *= scale
        scaled_camera_matrix[1, 1] *= scale
        scaled_camera_matrix[0, 2] *= scale
        scaled_camera_matrix[1, 2] *= scale
        return scaled_camera_matrix, dist_coeffs

    def prepare_detection_frame(self, frame: np.ndarray) -> tuple[np.ndarray, float]:
        scale = self.processing_scale
        detection_frame = frame

        if scale < 1.0:
            detection_frame = cv2.resize(
                frame,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_AREA,
            )

        if self.use_grayscale:
            detection_frame = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2GRAY)

        return detection_frame, scale

    def capture_and_publish(self) -> None:
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().warning("Failed to read frame from video device.")
            return

        self.frame_counter += 1
        if self.frame_skip > 0 and (self.frame_counter - 1) % (self.frame_skip + 1) != 0:
            return

        detection_frame, scale = self.prepare_detection_frame(frame)
        corners, ids, _ = self.detector.detect_markers(detection_frame)
        if self.calibrated_camera_matrix is None or self.calibrated_dist_coeffs is None:
            camera_matrix, dist_coeffs = self.build_fallback_camera_model(detection_frame)
        else:
            camera_matrix, dist_coeffs = self.scale_camera_model(
                self.calibrated_camera_matrix,
                self.calibrated_dist_coeffs,
                scale,
            )

        detected_ids_msg = Int32MultiArray()
        detected_ids_msg.data = [] if ids is None else [int(marker_id) for marker_id in ids.flatten()]
        self.detected_ids_pub.publish(detected_ids_msg)

        if ids is not None:
            camera_frame = self.camera_frame_override or "camera"
            stamp = self.get_clock().now().to_msg()

            for marker_corners, marker_id in zip(corners, ids.flatten()):
                image_points = marker_corners.reshape((4, 2)).astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    image_points,
                    camera_matrix,
                    dist_coeffs,
                )

                if not success:
                    continue

                if self.enable_kalman_filter:
                    pose_filter = self.pose_filters.get(marker_id)
                    if pose_filter is None:
                        pose_filter = PoseKalmanFilter(
                            dt=1.0 / max(1.0, self.capture_fps),
                            process_noise=float(self.kalman_process_noise),
                            measurement_noise=float(self.kalman_measurement_noise),
                        )
                        self.pose_filters[marker_id] = pose_filter
                    rvec, tvec = pose_filter.update(rvec, tvec)

                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = rotation_matrix_to_quaternion(rotation_matrix)

                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = camera_frame
                transform.child_frame_id = f"aruco_{marker_id}"
                transform.transform.translation.x = float(tvec[0][0])
                transform.transform.translation.y = float(tvec[1][0])
                transform.transform.translation.z = float(tvec[2][0])
                transform.transform.rotation.x = float(quaternion[0])
                transform.transform.rotation.y = float(quaternion[1])
                transform.transform.rotation.z = float(quaternion[2])
                transform.transform.rotation.w = float(quaternion[3])

                self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> None:
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
