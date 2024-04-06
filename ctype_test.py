import ctypes
import copy
import numpy as np
import cv2
import time

libc = ctypes.cdll.LoadLibrary("./build/libinsight_capture.so")

if __name__ == '__main__':

    data_type = b''    
    with open("data/raw_image.txt") as file:
        data_str = file.read()
        data_byte = data_str.encode()
        data_byte = data_byte.replace(b'\n', b'\r\n')

    start_time = time.time()

    src = data_byte[12:].replace(b'\r\n', b'')
    size = len(src)
    c_size = ctypes.c_int(size)
    np_src = np.frombuffer(src, np.uint8)
    p_src = np_src.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
    np_dst = np.array([0]*int(size/2), np.uint8)
    p_dst = np_dst.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))

    c_res = libc.string2bytes(c_size, p_src, p_dst)

    file_type = b"  "
    file_size = 0
    file_offset = 0

    np_header = np_dst[:14]
    p_header = np_header.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
    c_file_type = ctypes.c_char_p(file_type)
    c_file_size = ctypes.c_int(file_size)
    p_file_size = ctypes.pointer(c_file_size)
    c_file_offset = ctypes.c_int(file_offset)
    p_file_offset = ctypes.pointer(c_file_offset)

    c_res = libc.readHeader(p_header, c_file_type, p_file_size, p_file_offset)

    file_type = c_file_type.value
    file_size = c_file_size.value
    file_offset = c_file_offset.value

    image_width = 0
    image_height = 0

    np_info = np_dst[14:54]
    p_info = np_info.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
    c_image_width = ctypes.c_int(image_width)
    p_image_width = ctypes.pointer(c_image_width)
    c_image_height = ctypes.c_int(image_height)
    p_image_height = ctypes.pointer(c_image_height)

    c_res = libc.readInformation(p_info, p_image_width, p_image_height)

    image_width = c_image_width.value
    image_height = c_image_height.value

    cv_image = np_dst[file_offset:file_offset+image_width*image_height]
    cv_image = cv_image.reshape([image_height, image_width])

    end_time = time.time()
    print(f"TIME: {end_time-start_time}")
    print(f'File Type  : {file_type}\n \
            File Size  : {file_size}\n \
            File Offset: {file_offset}\n \
            Image Size: {image_width}x{image_height}')


    cv2.imwrite("test2.jpg", cv_image)

    
