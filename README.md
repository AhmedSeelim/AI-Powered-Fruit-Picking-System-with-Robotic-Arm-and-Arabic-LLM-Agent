# AI-Powered-Fruit-Picking-System-with-Robotic-Arm-and-Arabic-LLM-Agent

## Project Overview
This project integrates a conversational AI agent with object detection, robotic arm control, and real-time user interaction to create an innovative fruit-ordering system. Users can interact with the system through voice or text to request specific fruits, which are then identified, picked up, and delivered by a robotic arm controlled via Raspberry Pi.

## Features
### LLM Agent
The Language Learning Model (LLM) agent enables a natural conversational interface for ordering fruits:
- Processes spoken or typed inputs.
- Manages fruit orders.
- Provides voice and text responses in Egyptian Arabic.

### Object Detection
Utilizes YOLO-based object detection to identify fruits (oranges and bananas) in the camera feed:
- Identifies the closest and most accurate bounding box for the requested fruit.
- Integrates detections with the order system.

### Robotic Arm Control
The robotic arm employs inverse kinematics (IK) to pick up and deliver fruits:
- Six degrees of freedom.
- Grips and retrieves fruits based on calculated 3D coordinates.

---

## Components

### Hardware
1. **Camera**: Captures images for object detection.
2. **Microphone**: Records user voice input.
3. **Robotic Arm**: Operates with six degrees of freedom:
   - Base rotation
   - Shoulder joint
   - Elbow joint
   - Wrist pitch
   - Wrist roll
   - Gripper control
4. **Raspberry Pi**: Coordinates robotic arm movement and manages communication with the main system.
5. **Power Supply**: Supports Raspberry Pi and servo motors.

### Software
1. **Python Libraries**:
   - `LangChain` for LLM integration.
   - `speech_recognition` for speech-to-text conversion.
   - `gtts` for text-to-speech.
   - `ultralytics` for YOLO-based object detection.
   - `opencv` for image processing.
   - `RPi.GPIO` for robotic arm control.
   - `numpy` for mathematical calculations.
2. **Pre-trained Models**:
   - LLM: Gemini-2.0-Flash-Exp.
   - YOLO model: `fruits_detect.pt`.

---

## Workflow

### LLM Agent
1. **User Interaction**:
   - Captures user input through voice or text.
   - Processes input to identify requested fruits.
2. **Order Management**:
   - Adds items to the order.
   - Provides summaries and ends the order.
3. **Tool Execution**:
   - Interacts with tools for fruit-specific operations (order, detect, finalize).

### Object Detection
1. Captures images from the camera.
2. Runs inference using YOLO to detect fruits.
3. Filters detections to find the closest fruit with the highest confidence.

### Coordinate Conversion
1. Transforms 2D pixel coordinates to 3D world coordinates using intrinsic and extrinsic camera parameters.
2. Sends calculated (x, y, z) coordinates to the Raspberry Pi.

### Robotic Arm
1. Moves to an initial position.
2. Calculates joint angles using inverse kinematics to reach the fruit.
3. Aligns and grips the fruit.
4. Moves the fruit to a designated position.

---

## Code Components

### LLM Agent
Handles user interaction and order management through:
- Natural language processing.
- Integration with tools for adding and finalizing fruit orders.
- Voice feedback for responses.

### Object Detection
1. **Model Initialization**: Loads the `fruits_detect.pt` model.
2. **Inference**: Detects bounding boxes for fruits.
3. **Filtering**: Selects the most accurate detection.

### Robotic Arm
1. **GPIO Setup**: Configures servo motors.
2. **Inverse Kinematics**:
   - Calculates joint angles based on (x, y, z) coordinates.
   - Ensures the target position is within reach.
3. **Servo Movement**: Moves servos to the calculated angles.
4. **Gripping**: Aligns the gripper to secure the fruit.

---

## System Flow
1. User interacts with the LLM agent via voice or text.
2. The system identifies and adds the requested fruit to the order.
3. Camera captures an image, and YOLO identifies the fruit's location.
4. Coordinates are converted and sent to the Raspberry Pi.
5. The robotic arm moves to pick up and deliver the fruit.
6. The order is finalized, and a summary is provided to the user.

---

## Troubleshooting
- **Camera Not Detected**: Check connections and ensure the camera is enabled.
- **Servo Not Moving**: Verify GPIO pin configuration and power supply.
- **Incorrect Positioning**: Validate inverse kinematics calculations.
- **Audio Issues**: Adjust microphone settings and test the `speech_recognition` library.

---

## Demo
[Watch the demo video](https://youtu.be/73HRjttQJC4)


