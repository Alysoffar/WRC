#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Whisper Speech Recognition Library for Yahboom Robot
This library provides speech-to-text functionality using OpenAI's Whisper model
"""

import numpy as np
import os
import sounddevice as sd
from scipy.io.wavfile import write
import whisper
import queue
import time
import threading

class WhisperSpeechRecognizer:
    """
    A class that provides speech-to-text functionality using Whisper
    """
    
    def __init__(self, model_size="base", sample_rate=44100, channels=1):
        """
        Initialize the Whisper speech recognizer
        
        Args:
            model_size (str): Whisper model size ("tiny", "base", "small", "medium", "large")
            sample_rate (int): Audio sample rate in Hz
            channels (int): Number of audio channels (1 for mono, 2 for stereo)
        """
        self.model_size = model_size
        self.fs = sample_rate
        self.channels = channels
        self.silence_threshold = 0.01  # Lower = more sensitive
        self.silence_duration = 2.0    # seconds of silence before stopping
        self.chunk_duration = 0.2      # seconds per chunk
        
        # Initialize audio queue
        self.audio_queue = queue.Queue()
        
        # Load Whisper model
        print(f"Loading Whisper model: {model_size}")
        self.model = whisper.load_model(model_size)
        print("Whisper model loaded successfully!")
        
        # Set up temporary directory for audio files
        self.temp_dir = "/tmp"
        if not os.path.exists(self.temp_dir):
            os.makedirs(self.temp_dir)
    
    def audio_callback(self, indata, frames, time, status):
        """
        Callback function for sounddevice audio stream
        
        Args:
            indata: Input audio data
            frames: Number of frames
            time: Time information
            status: Stream status
        """
        if status:
            print(f"Audio status: {status}")
        self.audio_queue.put(indata.copy())
    
    def listen_and_transcribe(self, max_silence_duration=None, max_recording_time=30):
        """
        Record audio and transcribe it using Whisper
        
        Args:
            max_silence_duration (float): Seconds of silence before stopping recording
            max_recording_time (float): Maximum recording time in seconds
            
        Returns:
            str: Transcribed text or None if failed
        """
        if max_silence_duration is None:
            max_silence_duration = self.silence_duration
            
        print("Starting speech recording...")
        print("Speak into the microphone (will stop when silent for {}s)".format(max_silence_duration))
        
        try:
            # Clear the audio queue
            while not self.audio_queue.empty():
                self.audio_queue.get()
            
            audio_data = []
            silent_chunks = 0
            required_silent_chunks = int(max_silence_duration / self.chunk_duration)
            max_chunks = int(max_recording_time / self.chunk_duration)
            chunk_count = 0
            
            with sd.InputStream(
                samplerate=self.fs,
                channels=self.channels,
                callback=self.audio_callback,
                blocksize=int(self.fs * self.chunk_duration)
            ):
                print("Recording... speak now!")
                
                while chunk_count < max_chunks:
                    try:
                        # Get audio chunk with timeout
                        chunk = self.audio_queue.get(timeout=1.0)
                        audio_data.append(chunk)
                        chunk_count += 1
                        
                        # Compute RMS loudness to detect silence
                        rms = np.sqrt(np.mean(chunk**2))
                        
                        if rms < self.silence_threshold:
                            silent_chunks += 1
                        else:
                            silent_chunks = 0
                        
                        # Stop recording if silence detected
                        if silent_chunks >= required_silent_chunks and len(audio_data) > required_silent_chunks:
                            print("Silence detected. Stopping recording.")
                            break
                            
                    except queue.Empty:
                        print("Audio queue timeout - no audio data received")
                        break
            
            if not audio_data:
                print("No audio data recorded")
                return None
            
            # Combine audio chunks
            recording = np.concatenate(audio_data, axis=0)
            
            # Generate unique filename
            timestamp = int(time.time() * 1000)
            wav_path = os.path.join(self.temp_dir, f"speech_{timestamp}.wav")
            
            # Save as WAV file
            write(wav_path, self.fs, (recording * 32767).astype(np.int16))
            print(f"Audio saved as {wav_path}")
            
            # Transcribe using Whisper
            print("Transcribing with Whisper...")
            result = self.model.transcribe(wav_path)
            
            text = result["text"].strip()
            print(f"Transcription: {text}")
            
            # Clean up temporary file
            try:
                os.remove(wav_path)
            except:
                pass
            
            return text if text else None
            
        except Exception as e:
            print(f"Speech recognition error: {e}")
            return None
    
    def transcribe_file(self, audio_file_path):
        """
        Transcribe an existing audio file
        
        Args:
            audio_file_path (str): Path to the audio file
            
        Returns:
            str: Transcribed text or None if failed
        """
        try:
            print(f"Transcribing file: {audio_file_path}")
            result = self.model.transcribe(audio_file_path)
            text = result["text"].strip()
            print(f"Transcription: {text}")
            return text if text else None
        except Exception as e:
            print(f"File transcription error: {e}")
            return None
    
    def set_silence_threshold(self, threshold):
        """
        Set the silence detection threshold
        
        Args:
            threshold (float): Silence threshold (lower = more sensitive)
        """
        self.silence_threshold = threshold
        print(f"Silence threshold set to: {threshold}")
    
    def set_silence_duration(self, duration):
        """
        Set the silence duration before stopping
        
        Args:
            duration (float): Silence duration in seconds
        """
        self.silence_duration = duration
        print(f"Silence duration set to: {duration} seconds")


# Convenience function for quick usage
def quick_transcribe(model_size="base", max_silence_duration=2.0):
    """
    Quick function to transcribe speech with default settings
    
    Args:
        model_size (str): Whisper model size
        max_silence_duration (float): Silence duration before stopping
        
    Returns:
        str: Transcribed text or None if failed
    """
    recognizer = WhisperSpeechRecognizer(model_size=model_size)
    return recognizer.listen_and_transcribe(max_silence_duration=max_silence_duration)


# Test function
if __name__ == "__main__":
    # Test the library
    recognizer = WhisperSpeechRecognizer(model_size="base")
    
    print("Testing Whisper Speech Recognition Library")
    text = recognizer.listen_and_transcribe(max_silence_duration=3.0)
    
    if text:
        print(f"Final result: {text}")
    else:
        print("No speech detected or transcription failed")
