import argparse
from insight_native_client import inSightNativeClient

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



