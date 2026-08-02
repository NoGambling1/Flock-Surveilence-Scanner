import struct
import os

# Define file names
input_file = 'cameras.bin'
output_file = 'cameras_sorted.bin'

# 1. Auto-detect the record size
# ESP32 sizeof(CameraPoint) is 12 bytes due to memory alignment padding.
# If your generator didn't include padding, it will be 9 bytes.
file_size = os.path.getsize(input_file)

if file_size % 12 == 0:
    record_size = 12
    # Format: Little-endian (<), Float (f), Float (f), Unsigned Char (B), 3 Pad Bytes (3x)
    struct_format = '<ffB3x' 
    print(f"Detected 12-byte padded records. Total cameras: {file_size // 12}")
elif file_size % 9 == 0:
    record_size = 9
    # Format: Little-endian (<), Float (f), Float (f), Unsigned Char (B)
    struct_format = '<ffB'
    print(f"Detected 9-byte packed records. Total cameras: {file_size // 9}")
else:
    print(f"[ERROR] File size ({file_size} bytes) doesn't evenly divide by 9 or 12.")
    print("Check your cameras.bin file. It might be corrupted.")
    exit()

cameras = []

# 2. Read and unpack the existing unsorted binary file
with open(input_file, 'rb') as f:
    while True:
        chunk = f.read(record_size)
        if not chunk:
            break
        
        # Unpack the binary chunk into Python variables
        lat, lng, cam_type = struct.unpack(struct_format, chunk)
        cameras.append((lat, lng, cam_type))

# 3. Sort the list based on Latitude (the first item in the tuple: index 0)
# This is mandatory for the ESP32's binary search to work!
cameras.sort(key=lambda x: x[0])

# 4. Write the sorted data back to a new binary file
with open(output_file, 'wb') as f:
    for cam in cameras:
        # Pack the variables back into raw bytes and write to disk
        f.write(struct.pack(struct_format, cam[0], cam[1], cam[2]))

print(f"\n[SUCCESS] Sorted {len(cameras)} cameras.")
print(f"Saved as '{output_file}'. Upload this file to your ESP32 LittleFS partition!")