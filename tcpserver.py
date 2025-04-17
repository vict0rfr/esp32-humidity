import socket
import wave
import numpy as np

# Server configuration
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 12345      # Port number (must match the ESP32's configuration)
# AUDIO_FILE = r"C:\Users\victo\Desktop\bombastic-comp.wav"

AUDIO_FILE = "test.wav"
sample_rate = 44100  # 44.1 kHz
duration = 5  # 5 seconds
frequency = 440.0  # A4 note

t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
audio_data = (0.5 * np.sin(2 * np.pi * frequency * t) * 32767).astype(np.int16)

# Save the generated audio data as a WAV file
with wave.open(AUDIO_FILE, 'wb') as audio:
    audio.setnchannels(1)  # Mono
    audio.setsampwidth(2)  # 16-bit samples
    audio.setframerate(sample_rate)  # 44.1 kHz sample rate
    audio.writeframes(audio_data.tobytes())

print(f"Generated WAV file saved as {AUDIO_FILE}")

# Create a TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Server listening on {HOST}:{PORT}")

try:
    while True:
        # Accept a connection from the client (ESP32)
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address}")

            # Open the audio file in binary mode and send its contents
        with open(AUDIO_FILE, 'rb') as audio_file:
            print(f"Sending WAV file: {AUDIO_FILE}")
            data = audio_file.read(8096)  # Read in chunks
            while data:
                client_socket.sendall(data)
                data = audio_file.read(8096)

        print("Finished sending WAV file.")

        # Close the client connection
        client_socket.close()
except KeyboardInterrupt:
    print("Shutting down server...")
finally:
    server_socket.close()