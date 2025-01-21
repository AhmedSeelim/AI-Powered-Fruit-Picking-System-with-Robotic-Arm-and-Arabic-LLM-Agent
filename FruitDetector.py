from ultralytics import YOLO
import numpy as np

class FruitDetector:
    def __init__(self, image_width=640, image_height=480):
        """
        Initialize the FruitDetector class.

        Args:
            image_width (int): Width of the input image.
            image_height (int): Height of the input image.
        """
        self.model_path = "fruits_detect.pt"
        self.model = YOLO(self.model_path)
        self.image_width = image_width
        self.image_height = image_height

    def get_closest_and_most_accurate(self, result_boxes, class_id):
        """
        Get the closest and most accurate detection for a given class ID.

        Args:
            result_boxes: List of bounding box results from YOLO inference.
            class_id: The class ID for the target object (e.g., 0 for "apple").

        Returns:
            A tuple containing the closest and most accurate bounding box
            (x1, y1, x2, y2) and confidence score, or None if no detections.
        """
        target_boxes = [
            (box.xyxy[0].tolist(), box.conf[0].item(), box.cls[0].item())
            for box in result_boxes
            if int(box.cls[0].item()) == class_id
        ]

        if not target_boxes:
            return None  # No detections for the specified class

        # Calculate the center of each box and sort by confidence and proximity to image center
        sorted_boxes = sorted(
            target_boxes,
            key=lambda x: (
                -x[1],  # Sort by confidence (descending)
                np.linalg.norm(
                    np.array([(x[0][0] + x[0][2]) / 2, (x[0][1] + x[0][3]) / 2]) -
                    np.array([self.image_width / 2, self.image_height / 2])
                )  # Sort by proximity to image center (ascending)
            )
        )

        return sorted_boxes[0]  # Return the best box (highest confidence and closest to center)

    def detect_object(self, results, class_id):
        """
        Detect the closest and most accurate object of a given class ID.

        Args:
            results: YOLO inference results.
            class_id: The class ID for the target object.

        Returns:
            The closest and most accurate detection or None if no detections.
        """
        return self.get_closest_and_most_accurate(results, class_id)

    def detect_orange(self, results):
        """Detect the closest and most accurate apple."""
        return self.detect_object(results, class_id=3)

    def detect_banana(self, results):
        """Detect the closest and most accurate banana."""
        return self.detect_object(results, class_id=1)

    def run_inference(self, image_path, show=True, save=True):
        """
        Run inference on an image and return the results.

        Args:
            image_path (str): Path to the input image.
            show (bool): Whether to display the inference result.
            save (bool): Whether to save the inference result.

        Returns:
            YOLO inference results.
        """
        return self.model.predict(source=image_path, show=show, save=save)

# Example usage
# if __name__ == "__main__":
#     # Initialize the detector
#     detector = FruitDetector()
#
#     # Run inference
#     results = detector.run_inference(image_path="images.jpeg")
#
#     # Process results
#     for result in results:
#         closest_apple = detector.detect_apple(result.boxes)
#         closest_banana = detector.detect_banana(result.boxes)
#
#         if closest_apple:
#             print("Closest and most accurate Apple:", closest_apple)
#         else:
#             print("No Apple detected.")
#
#         if closest_banana:
#             print("Closest and most accurate Banana:", closest_banana)
#         else:
#             print("No Banana detected.")
