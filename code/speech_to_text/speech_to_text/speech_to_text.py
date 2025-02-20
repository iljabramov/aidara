"""
ROS 2 Node for speech-to-text conversion.

The node listens to the default microphone, waits for a keyword-prefixed instruction,
and publishes it on a topic.
"""

import string

import rclpy
import sounddevice  # noqa: F401
import speech_recognition as sr
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class SpeechToText(Node):
    """ROS 2 Node for speech-2-text conversion using OpenAI Whisper."""

    def __init__(self) -> None:
        """Initialize SpeechToText."""
        super().__init__("speech_to_text")

        self._keyword = "alfred"
        self._recognizer = sr.Recognizer()

        self._min_energy_threshold = 800
        self._max_energy_threshold = 3000
        self._recognizer.energy_threshold = 800
        self._recognizer.dynamic_energy_threshold = True

        self._counter = 0
        self._max_counter = 5

        self._punctuation_replacement = str.maketrans("", "", string.punctuation)

        self._pub = self.create_publisher(
            String,
            "/speech_to_text",
            1,
        )
        self._raw_pub = self.create_publisher(String, "/speech_to_text_raw", 1)

    def _transcribe_microphone(self) -> str | None:
        """Transcribe microphone input using OpenAI Whisper."""
        with sr.Microphone() as microphone:
            audio_listened = self._recognizer.listen(microphone)
            self._counter += 1

            if not audio_listened.frame_data:
                return None

            if self._counter >= self._max_counter:
                self.get_logger().info("Adjusting to ambient noise...")
                self._recognizer.adjust_for_ambient_noise(microphone)
                self._counter = 0
                self._check_ambient_noise_adjustment()
        try:
            text = self._recognizer.recognize_whisper(
                audio_listened,
                model="large-v3",
                language="english",
                initial_prompt=self._keyword*2,
            )
        except sr.UnknownValueError:
            self.get_logger().info("speech_to_text was unable to understand a phrase.")
            return None

        self._raw_pub.publish(String(data=text))

        self.get_logger().info(text)
        stripped_text = text.strip().lower().translate(self._punctuation_replacement)
        self.get_logger().debug(f"recognized text: '{stripped_text}'.")

        return stripped_text

    def _check_ambient_noise_adjustment(self) -> None:
        """Check if energy_threshold needs to be adjusted within a range (800-3000)."""
        if self._recognizer.energy_threshold < self._min_energy_threshold:
            self._recognizer.energy_threshold = self._min_energy_threshold
            self.get_logger().info(
                f"Adjustment lower than Min.: {self._recognizer.energy_threshold}",
            )

        elif self._recognizer.energy_threshold > self._max_energy_threshold:
            self._recognizer.energy_threshold = self._max_energy_threshold
            self.get_logger().info(
                f"Adjustment higher than Max.: {self._recognizer.energy_threshold}",
            )

        else:
            self.get_logger().info(
                f"New energy_threshold: {self._recognizer.energy_threshold}",
            )

    def _extract_instruction(self, stripped_text: str) -> str | None:
        """Extract instruction from text."""
        _preamble, *instructions = stripped_text.split(self._keyword, maxsplit=1)

        if not instructions:
            return None

        return instructions[0]

    def listen(self) -> None:
        """Extract instruction from the microphone and publish it on /speech_to_text."""
        self.get_logger().info("Listening...")
        stripped_text = self._transcribe_microphone()

        if self._recognizer.energy_threshold < self._min_energy_threshold:
            self._recognizer.energy_threshold = 800
        if not stripped_text:
            return

        extracted_instruction = self._extract_instruction(stripped_text)
        if not extracted_instruction:
            return

        msg = String()
        msg.data = extracted_instruction
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Initialize and run the SpeechToText node."""
    rclpy.init(args=args)

    try:
        speech_to_text = SpeechToText()

        executor = MultiThreadedExecutor()
        executor.add_node(speech_to_text)

        while rclpy.ok():
            listen = executor.create_task(speech_to_text.listen)
            executor.spin_until_future_complete(listen)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
