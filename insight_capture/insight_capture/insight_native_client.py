import socket
import time
import ctypes
import numpy as np

class InSightNativeClient:

    def __init__(self, ip, port, username, password, timeout, lib_path):
        self.server_ip = ip
        self.server_port = port
        self.server_timeout = timeout
        self.username = username
        self.password = password

        print(lib_path)

        self.libc = ctypes.cdll.LoadLibrary(lib_path)

        self.is_connected = False
        self.is_login = False

        self.recv_temp_buffer = b''

    def connect(self):
        try:
            if self.is_connected:
                self.socket.close()
                self.is_login = False
                self.is_connected = False

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
            self.socket.settimeout(self.server_timeout)
            self.socket.connect((self.server_ip, self.server_port))
            self.is_login = False
            self.is_connected = True

        except Exception as ex:
            self.is_login = False
            self.is_connected = False
            print("socket connection error: {}".format(ex))
            return False
        
        return True

    def read_until_size(self, size):
        datas = self.recv_temp_buffer
        recvd_size = 0

        try:
            if self.is_connected:
                while True:
                    data = self.socket.recv(1024)
                    recvd_size += len(data)
                    datas += data

                    if recvd_size >= size:
                        self.recv_temp_buffer = datas[size:]
                        return(size, data[:size])
                    
        except Exception as ex:
            print("failed to read until size: {}".format(ex))
            return (0, b'')


    def read_until_match(self, match):
        datas = self.recv_temp_buffer

        try:
            if self.is_connected:
                while True:
                    data = self.socket.recv(1024)
                    if len(data) > 0:
                        datas += data
                        
                        if match in datas:
                            tmp = datas.split(match, 1)

                            if len(tmp) > 0:
                                recv_data = tmp[0] + match

                            if len(tmp) > 1:
                                self.recv_temp_buffer = tmp[1]
                            else:
                                self.recv_temp_buffer = b''

                            break
                    else:
                        raise RuntimeError('not match recv data')
                
                return (len(recv_data), recv_data)
            else:
                raise RuntimeError('not connected')

        except Exception as ex:
            print("failed to read data until match: {}".format(ex))
            return (0, b'')
  
    def login(self):
        try:
            if self.is_connected:
                # Send to Username
                ret, recv_data = self.read_until_match(b'User: ')
                if ret == 0:
                    self.is_login = False
                    self.is_connected = True
                    print("failed to recv user string")
                    return False
                
                msg = self.username + "\r\n"
                send_data = msg.encode('utf-8')
                ret = self.socket.send(send_data)
                if ret != len(send_data):
                    self.is_login = False
                    self.is_connected = True
                    print("failed to send username and password")
                    return False
                
                # Send to Password
                ret, recv_data = self.read_until_match(b'Password: ')
                if ret == 0:
                    self.is_login = False
                    self.is_connected = True
                    print("failed to recv password string")
                    return False

                msg = self.password + "\r\n"
                send_data = msg.encode('utf-8')
                ret = self.socket.send(send_data)
                if ret != len(send_data):
                    self.is_login = False
                    self.is_connected = True
                    print("failed to send username and password")
                    return False
                
                time.sleep(0.5)

                # Return check
                ret, recv_data = self.read_until_match(b'\r\n')
                if ret > 0 and b'User Logged In' in recv_data:
                    self.is_login = True
                    return True
                else:
                    self.is_login = False
                    self.is_connected = True
                    print("failed to login")
                    return False
            
            else:
                self.is_login = False
                self.is_connected = True
                print("not connected")
                return False
        except Exception as ex:
            self.is_login = False
            self.is_connected = True
            print("login faled: {}".format(ex))
            return False

    def capture(self):

        if self.is_connected and self.is_login:
            msg = "RI" + "\r\n"
            send_data = msg.encode()
            ret = self.socket.send(send_data)
            if ret != len(send_data):
                self.is_login = False
                self.is_connected = False
                print("failed to send username and password")
                return False
            
            time.sleep(0.1)

            datas = b''
            try:
                while True:
                    data = self.socket.recv(1024)
                    datas += data
            except TimeoutError:
                pass
            
            tmp = datas.split(b'\r\n')
            status = tmp[0].decode()
            size = tmp[1].decode()
            checksum = tmp[-2]
            image = tmp[2:-2]
            image_data = b''
            for row in image:
                image_data += row


            src = image_data.replace(b'\r\n', b'')
            size = len(src)
            c_size = ctypes.c_int(size)
            np_src = np.frombuffer(src, np.uint8)
            p_src = np_src.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
            np_dst = np.array([0]*int(size/2), np.uint8)
            p_dst = np_dst.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))

            c_res = self.libc.string2bytes(c_size, p_src, p_dst)

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

            c_res = self.libc.readHeader(p_header, c_file_type, p_file_size, p_file_offset)

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

            c_res = self.libc.readInformation(p_info, p_image_width, p_image_height)

            image_width = c_image_width.value
            image_height = c_image_height.value

            cv_image = np_dst[file_offset:file_offset+image_width*image_height]
            cv_image = cv_image.reshape([image_height, image_width])

            return cv_image

        else:
            print("not connected or not login")
            return None
