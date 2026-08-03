import struct # This python script takes the location of Flock cameras, sorts by lattitude for the Binary Search, and saves to a .bin
import os # https://overpass-turbo.eu/

input_file = 'input.bin' # Change to your input file
output_file = 'cameras.bin'
# 1. Auto-detect the record size
file_size = os.path.getsize(input_file)

if file_size % 12 == 0:
    record_size = 12
    struct_format = '<ffB3x' 
    print(f"Detected 12-byte padded records. Total cameras: {file_size // 12}")
elif file_size % 9 == 0:
    record_size = 9
    struct_format = '<ffB'
    print(f"Detected 9-byte packed records. Total cameras: {file_size // 9}")
else:
    print(f"[ERROR] File size ({file_size} bytes) doesn't evenly divide by 9 or 12.")
    print("Check your cameras.bin file. It might be corrupted.")
    exit()

cameras = []

with open(input_file, 'rb') as f:
    while True:
        chunk = f.read(record_size)
        if not chunk:
            break
        
        lat, lng, cam_type = struct.unpack(struct_format, chunk)
        cameras.append((lat, lng, cam_type))


cameras.sort(key=lambda x: x[0])

with open(output_file, 'wb') as f:
    for cam in cameras:
        f.write(struct.pack(struct_format, cam[0], cam[1], cam[2]))

print(f"\n[SUCCESS] Sorted {len(cameras)} cameras.")
print(f"Saved as '{output_file}'. Upload this file to your ESP32 LittleFS partition!")
