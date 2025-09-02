import numpy as np
import os
import sounddevice as sd
from scipy.io.wavfile import write
import whisper
import queue

# Add FFmpeg to PATH (Windows only)
os.environ["PATH"] += os.pathsep + r"C:\ffmpeg-8.0-full_build\ffmpeg-8.0-full_build\bin"

# Parameters
fs = 44100  # Sample rate
channels = 1
silence_threshold = 0.01  # Lower = more sensitive
silence_duration = 2.0    # seconds of silence before stopping

# Queue to store recorded chunks
q = queue.Queue()

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status)
    q.put(indata.copy())

print("Recording... speak into the mic (stop when silent for 2s)")

audio_data = []
silent_chunks = 0
chunk_duration = 0.2  # seconds per chunk
required_silent_chunks = int(silence_duration / chunk_duration)

with sd.InputStream(samplerate=fs, channels=channels, callback=callback, blocksize=int(fs*chunk_duration)):
    while True:
        chunk = q.get()
        audio_data.append(chunk)

        # Compute RMS loudness
        rms = np.sqrt(np.mean(chunk**2))
        if rms < silence_threshold:
            silent_chunks += 1
        else:
            silent_chunks = 0

        if silent_chunks >= required_silent_chunks:
            print("Silence detected. Stopping.")
            break

# Combine chunks
recording = np.concatenate(audio_data, axis=0)

# Save as WAV
wav_path = "myvoice.wav"
write(wav_path, fs, (recording * 32767).astype(np.int16))
print(f"Recording saved as {wav_path}")

# Load Whisper model and transcribe
print("Transcribing with Whisper...")
model = whisper.load_model("base")
result = model.transcribe(wav_path)

print("Transcription:")
print(result["text"])
