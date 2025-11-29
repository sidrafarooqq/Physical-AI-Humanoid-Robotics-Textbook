# Chapter 1: Voice-to-Action - Speech Recognition and Command Processing

### 1.1 Introduction to Voice-Enabled Robotics

Voice interfaces represent a critical frontier in human-robot interaction by enabling intuitive, natural communication. This chapter explores how robots can listen, understand, and act on spoken commands using OpenAI's Whisper model and language processing frameworks.

**Why Voice Commands Matter:**
- **Accessibility**: Non-technical users can control robots naturally
- **Real-time Interaction**: Faster than manual control or GUI interfaces
- **Hands-free Operation**: Essential for collaborative robotics scenarios
- **Cognitive Load Reduction**: Users think in natural language, not robot commands

### 1.2 OpenAI Whisper: Speech-to-Text Foundation

#### 1.2.1 Architecture and Capabilities

OpenAI Whisper is a robust, multilingual automatic speech recognition (ASR) system trained on 680,000 hours of multilingual and multitask supervised data collected from the web.

**Model Variants:**

| Model | Parameters | English-Only Speed | Multilingual Speed | Typical Use Case |
|-------|-----------|-------------------|-------------------|-----------------|
| **Tiny** | 39M | ~32x faster | ~16x faster | Edge devices, Jetson |
| **Base** | 74M | ~16x faster | ~8x faster | Real-time, onboard |
| **Small** | 244M | ~6x faster | ~3x faster | High accuracy, fast |
| **Medium** | 769M | ~2x faster | ~1.6x faster | Production standard |
| **Large** | 1.5B | Real-time | Real-time | Maximum accuracy |

#### 1.2.2 Acoustic Modeling

```
Audio Waveform
     ↓
┌─────────────────────┐
│ Log-Mel Spectrogram │ (80 mel-frequency bins)
└─────────────────────┘
     ↓
┌─────────────────────┐
│ Encoder (Transformer) │ → Audio embeddings
└─────────────────────┘
     ↓
┌─────────────────────┐
│ Decoder (Transformer) │ → Text tokens
└─────────────────────┘
     ↓
Transcribed Text
```

#### 1.2.3 Implementation on Robots

```python
# OpenAI Whisper Integration on Humanoid Robot
import openai
import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Initialize Whisper
        self.model = openai.api_resources.Model.retrieve("whisper-1")
        
        # Audio recording parameters
        self.CHUNK = 2048
        self.FORMAT = pyaudio.paFloat32
        self.CHANNELS = 1
        self.RATE = 16000  # 16kHz optimal for Whisper
        self.RECORD_SECONDS = 5
        
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        
        # Create publisher for recognized commands
        self.command_publisher = self.create_publisher(
            String,
            '/robot/voice_command',
            10
        )
        
    def record_audio(self):
        """Record audio from microphone"""
        stream = self.p.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )
        
        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
            data = stream.read(self.CHUNK)
            frames.append(data)
        
        stream.stop_stream()
        stream.close()
        
        return b''.join(frames)
    
    def transcribe_with_whisper(self, audio_data):
        """
        Transcribe audio using Whisper
        
        Args:
            audio_data: Raw audio bytes
            
        Returns:
            transcribed_text: String containing recognized speech
        """
        # Convert audio to wav format
        import io
        import wave
        
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(self.CHANNELS)
            wav_file.setsampwidth(self.p.get_sample_size(self.FORMAT))
            wav_file.setframerate(self.RATE)
            wav_file.writeframes(audio_data)
        
        wav_buffer.seek(0)
        
        # Call OpenAI Whisper API
        transcript = openai.Audio.transcribe(
            model="whisper-1",
            file=wav_buffer,
            language="en"  # Specify language for better accuracy
        )
        
        return transcript['text']
    
    def listen_and_transcribe(self):
        """Main loop for voice command listening"""
        self.get_logger().info("🎤 Listening for voice commands...")
        
        while rclpy.ok():
            try:
                # Record audio
                audio_data = self.record_audio()
                
                # Transcribe using Whisper
                command_text = self.transcribe_with_whisper(audio_data)
                
                self.get_logger().info(f"📝 Recognized: {command_text}")
                
                # Publish recognized command
                msg = String()
                msg.data = command_text
                self.command_publisher.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f"Error in voice recognition: {e}")

# Entry point
if __name__ == '__main__':
    rclpy.init()
    node = VoiceCommandNode()
    node.listen_and_transcribe()
```

#### 1.2.4 Handling Edge Cases and Robustness

```python
class RobustVoiceCommandProcessor:
    """Handle edge cases in voice recognition"""
    
    def __init__(self):
        self.confidence_threshold = 0.7
        self.max_retries = 3
        self.background_noise_threshold = -40  # dB
        
    def validate_audio_quality(self, audio_data):
        """Check if audio quality is sufficient"""
        # Compute RMS (Root Mean Square) for loudness
        rms = np.sqrt(np.mean(audio_data**2))
        
        if rms < 0.01:
            return False, "Audio too quiet"
        
        return True, "Audio quality OK"
    
    def handle_silence(self, audio_data):
        """Detect and handle silence"""
        threshold = np.mean(np.abs(audio_data)) * 2
        silent_frames = np.sum(np.abs(audio_data) < threshold)
        silence_ratio = silent_frames / len(audio_data)
        
        if silence_ratio > 0.8:
            return "SILENCE_DETECTED"
        
        return None
    
    def retry_with_feedback(self, voice_processor):
        """Retry recognition with user feedback"""
        for attempt in range(self.max_retries):
            try:
                transcript = voice_processor.transcribe_with_whisper()
                
                if len(transcript) > 0:
                    return transcript
                    
            except Exception as e:
                if attempt < self.max_retries - 1:
                    print(f"⚠️ Recognition failed. Retrying... ({attempt + 1}/{self.max_retries})")
                else:
                    print(f"❌ Failed after {self.max_retries} attempts")
        
        return None
```

### 1.3 Command Parsing and Validation

Once speech is transcribed, the text must be parsed to extract intent and parameters.

#### 1.3.1 Intent Recognition

```python
import re
from typing import Tuple, Dict

class CommandParser:
    """Parse natural language commands into robot actions"""
    
    def __init__(self):
        # Define command patterns
        self.patterns = {
            'move': r'(move|go|walk|navigate)\s+(?:to|towards)?\s+(\w+)',
            'pick': r'(pick up|grab|grasp)\s+(?:the\s+)?(\w+)',
            'place': r'(put|place|drop)\s+(?:the\s+)?(\w+)\s+(?:at|on|in)\s+(\w+)',
            'look': r'(look|see|find|search)\s+(?:for\s+)?(?:the\s+)?(\w+)',
            'stop': r'(stop|halt|pause)',
            'reset': r'(reset|restart)'
        }
    
    def parse_command(self, text: str) -> Tuple[str, Dict]:
        """
        Parse command text into action and parameters
        
        Args:
            text: Transcribed voice command
            
        Returns:
            action: Command type (e.g., 'move', 'pick')
            params: Dictionary of command parameters
        """
        text = text.lower().strip()
        
        for action, pattern in self.patterns.items():
            match = re.match(pattern, text)
            if match:
                groups = match.groups()
                
                # Extract parameters based on action
                if action == 'move':
                    return action, {'destination': groups[1]}
                
                elif action == 'pick':
                    return action, {'object': groups[1]}
                
                elif action == 'place':
                    return action, {
                        'object': groups[1],
                        'location': groups[2]
                    }
                
                elif action == 'look':
                    return action, {'target': groups[1]}
                
                elif action in ['stop', 'reset']:
                    return action, {}
        
        return 'unknown', {}
    
    def validate_command(self, action: str, params: Dict) -> bool:
        """
        Validate that command has all required parameters
        
        Args:
            action: Command type
            params: Command parameters
            
        Returns:
            True if valid, False otherwise
        """
        required_params = {
            'move': ['destination'],
            'pick': ['object'],
            'place': ['object', 'location'],
            'look': ['target'],
            'stop': [],
            'reset': []
        }
        
        if action not in required_params:
            return False
        
        for param in required_params[action]:
            if param not in params or params[param] is None:
                return False
        
        return True
```

#### 1.3.2 Confidence Scoring

```python
class ConfidenceEstimator:
    """Estimate confidence of parsed commands"""
    
    def estimate_confidence(self, command_text: str, 
                          parsed_action: str, 
                          parse_confidence: float) -> float:
        """
        Multi-factor confidence estimation
        
        Args:
            command_text: Original transcribed text
            parsed_action: Extracted action
            parse_confidence: Confidence from parser (0-1)
            
        Returns:
            Overall confidence score (0-1)
        """
        # Length-based confidence (very short = suspicious)
        length_score = min(len(command_text) / 10, 1.0)
        
        # Known action confidence
        known_actions = ['move', 'pick', 'place', 'look', 'stop', 'reset']
        action_score = 1.0 if parsed_action in known_actions else 0.3
        
        # Combine scores
        overall_confidence = (parse_confidence * 0.5 + 
                             length_score * 0.25 + 
                             action_score * 0.25)
        
        return overall_confidence
```

---