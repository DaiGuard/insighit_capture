import socket
import time

class inSightNativeClient:

    def __init__(self, ip, port, username, password, timeout):
        self.server_ip = ip
        self.server_port = port
        self.server_timeout = timeout
        self.username = username
        self.password = password

        self.is_connected = False
        self.is_login = False

        self.recv_temp_buffer = b''

    def connect(self):
        try:
            if self.is_connected:
                self.socket.close()
                self.is_connected = False

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
            self.socket.settimeout(self.server_timeout)
            self.socket.connect((self.server_ip, self.server_port))
            self.is_connected = True

            return True
        except Exception as ex:
            self.is_connected = False
            print("socket connection error: {}".format(ex))
            return False

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
                    print("failed to recv user string")
                    return False
                
                msg = self.username + "\r\n"
                send_data = msg.encode('utf-8')
                ret = self.socket.send(send_data)
                if ret != len(send_data):
                    self.is_login = False
                    print("failed to send username and password")
                    return False
                
                # Send to Password
                ret, recv_data = self.read_until_match(b'Password: ')
                if ret == 0:
                    self.is_login = False
                    print("failed to recv password string")
                    return False

                msg = self.password + "\r\n"
                send_data = msg.encode('utf-8')
                ret = self.socket.send(send_data)
                if ret != len(send_data):
                    self.is_login = False
                    print("failed to send username and password")
                    return False
                
                time.sleep(0.5)

                # Return check
                ret, recv_data = self.read_until_match(b'\r\n')
                if ret > 0 and b'User Logged In' in recv_data:
                    self.is_login = True
                    return True
                else:
                    return False
            
            else:
                self.is_login = False
                print("not connected")
                return False
        except Exception as ex:
            self.is_login = False
            print("login faled: {}".format(ex))
            return False

    def capture(self):

        if self.is_connected and self.is_login:
            msg = "RI" + "\r\n"
            send_data = msg.encode()
            ret = self.socket.send(send_data)
            if ret != len(send_data):
                self.is_login = False
                print("failed to send username and password")
                return False
            
            time.sleep(0.5)

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
                image_data =+ row

            return True

        else:
            print("not connected or not login")
            return False
