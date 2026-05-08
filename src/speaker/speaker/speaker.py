import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
from gtts import gTTS
import io

class SpeakerControl(Node):
    def __init__(self):
        super().__init__('speaker_control')
        pygame.init()
        pygame.mixer.init()  # Initialize Pygame mixer

        self.subscription = self.create_subscription(
            String,
            '/speaker',
            self.speaker_callback,
            10
        )
        self.latest_content = None

    def play_text(self, text):
        tts = gTTS(text=text, lang='en', tld='co.za', slow=False)  # Generate the speech tts_obj = gTTS(text=tts, tld=tlds["South Africa"], lang=lang, slow=False)

        fp = io.BytesIO()  # Create an in-memory file-like object
        tts.write_to_fp(fp)  # Write the speech data to the file-like object
        fp.seek(0)  # Rewind the file to the beginning
        pygame.mixer.music.load(fp)  # Load this into Pygame
        pygame.mixer.music.play()  # Play the sound

    def speaker_callback(self, msg):
        self.latest_content = msg.data
        self.get_logger().info(f"Received: {self.latest_content}")
        self.play_text(self.latest_content)  # Use TTS to speak the message


def main(args=None):
    rclpy.init(args=args)
    speaker_control = SpeakerControl()
    rclpy.spin(speaker_control)
    speaker_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

