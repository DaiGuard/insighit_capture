import argparse
import socket
import time
import xml.etree.ElementTree as ET
from telnetlib import Telnet
import cv2

class iSightTelnetClient:

    def __init__(self, ip, port, username, password, timeout):
        self.server_ip = ip
        self.server_port = port
        self.server_timeout = timeout
        self.username = username
        self.password = password

        self.is_connected = False
        self.is_login = False

    def connect(self):

        self.client = Telnet(self.server_ip, self.server_port, timeout=self.server_timeout)

        time.sleep(0.5)
        data = self.client.read_until(b'User: ', timeout=self.server_timeout)
        print(data)
        msg = self.username + "\r\n"
        send_data = msg.encode('utf-8')
        self.client.write(send_data)

        print(send_data)
        time.sleep(0.5)
        data = self.client.read_until(b'Password: ', timeout=self.server_timeout)
        print(data)
        msg = self.password + "\r\n"
        send_data = msg.encode('utf-8')
        self.client.write(send_data)

        print(send_data)

        time.sleep(5.0)

        data = self.client.read_eager()
        print(data)

    def capture(self):

        self.client.write(b'RI\r\n')

        time.sleep(5.0)

        data = self.client.read_eager()
        print(len(data), data)

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

            cv

            with open("log.txt", "w") as file:
                file.write(status)
                file.write("\n")
                file.write(size)
                file.write("\n")
                file.write(checksum.decode())
                file.write("\n")
                file.write(image_data.decode())

            # # Read result
            # ret, recv_data = self.read_until_match(b'\r\n')
            # print(ret, recv_data) 
            # # Read size
            # ret, recv_data = self.read_until_match(b'\r\n')
            # print(ret, recv_data) 

            # size = int(recv_data.split(b'\r\n', 1)[0].decode())
            # tmp = 0
            # while True:
            #     data = self.socket.recv(1024)
            #     # print(ret, recv_data)
            #     tmp += len(data)
            #     print(size, tmp)
            # size = int(recv_data.split(b'\r\n', 1)[0].decode())
            # ret, recv_data = self.read_until_size(size)
            # print(ret, recv_data) 

            # tmp = recv_data.split(b'\r\n')
            # print(len(tmp), len(tmp[0]))

            return True

        else:
            print("not connected or not login")
            return False

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="cognex isight2000 image capture program")

    parser.add_argument("--ip", type=str, default="127.0.0.1", help="isight server ip address")
    parser.add_argument("--port", type=int, default=50000, help="isight server port")
    parser.add_argument("--user", type=str, default="admin", help="isight user name")
    parser.add_argument("--passwd", type=str, default="", help="isight login password")

    args = parser.parse_args()

    print("server ip: {}".format(args.ip))
    print("server port: {}".format(args.port))
    print("username: {}".format(args.user))
    print("password: {}".format(args.passwd))

    client = inSightNativeClient(args.ip, args.port, args.user, args.passwd, 1.0)
    if not client.connect():
        print("Faled to connect")
        exit()
    print("Success to connect")
    
    if not client.login():
        print("Failed to login")
        exit()
    print("Success to login")

    client.capture()

    # client = iSightTelnetClient(args.ip, args.port, args.user, args.passwd, 3.0)
    # client.connect()
    # client.capture()


