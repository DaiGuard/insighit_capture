import time
import cv2
import numpy as np

def cognex2cv(data: bytes):

    start_time = time.time()

    tmp = data.split(b'\r\n')
    status = int(tmp[0].decode())
    size = int(tmp[1].decode())
    checksum = tmp[-2]
    image = tmp[2:-2]

    end_time = time.time()
    print(f'SPLIT: {end_time - start_time}')

    start_time = time.time()

    image_data = b''
    for row in image:
        for i in range(0, len(row), 2):
            one_byte_data = row[i:i+2]
            one_byte_str = one_byte_data.decode()
            one_byte_int = int(one_byte_str, 16)
            one_byte_hex = one_byte_int.to_bytes(1, 'big')
            image_data += one_byte_hex

    end_time = time.time()
    print(f'CONVERT: {end_time - start_time}')

    start_time = time.time()

    header = image_data[:14]
    file_type = header[:2]
    file_size = int.from_bytes(header[2:6], 'little')
    file_offset = int.from_bytes(header[10:14], 'little')

    info = image_data[14:54]
    header_size = int.from_bytes(info[:4], 'little')
    image_width = int.from_bytes(info[4:8], 'little')
    image_height = int.from_bytes(info[8:12], 'little')
    plane_num = int.from_bytes(info[12:14], 'little')
    bit_count = int.from_bytes(info[14:16], 'little')
    compression = int.from_bytes(info[16:20], 'little')
    image_data_size = int.from_bytes(info[20:24], 'little')
    xpixel_per_meter = int.from_bytes(info[24:28], 'little')
    ypixel_per_meter = int.from_bytes(info[28:32], 'little')
    cir_used = int.from_bytes(info[32:36], 'little')
    important_cir = int.from_bytes(info[36:40], 'little')

    pixels = image_data[file_offset:]

    cv_image = np.frombuffer(pixels, np.uint8)
    cv_image = cv_image.reshape([image_height, image_width])

    end_time = time.time()
    print(f'READ: {end_time - start_time}')

    cv2.imwrite("test.jpg", cv_image)
    
    print(f'File Type  : {file_type}\n \
            File Size  : {file_size}\n \
            File Offset: {file_offset}')
    

if __name__ == '__main__':

    # raw = b'42'
    # print(int(raw.decode(), 16).to_bytes(1, 'big'))

    # val = int.from_bytes(b'42')
    # data = val.to_bytes(1, "big")

    # print(val, data)


    with open("data/raw_image.txt") as file:
        data_str = file.read()
        data_byte = data_str.encode()
        data_byte = data_byte.replace(b'\n', b'\r\n')

        cognex2cv(data_byte)
