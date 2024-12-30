import socket


SOCKET_PATH = "/tmp/can_read_socket"



# 创建 Unix 域套接字

client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
try:
    # 尝试连接到指定的 Unix 域套接字地址
    client_socket.connect(SOCKET_PATH)
    print(f"已连接到 Unix 域套接字: {SOCKET_PATH}")
except socket.error as e:
    print(f"连接到 Unix 域套接字失败: {e}")

try:
    while True:
        # 一次最多读取 1024 字节，可根据实际情况调大或调小
        data = client_socket.recv(1024)
        if not data:
            print("对端已关闭连接，客户端退出。")
            break

        # 将字节数据解码为字符串
        msg = data.decode("utf-8", errors="replace")
        print("收到数据:", msg, end="")  
        # 注意 C 端发送时可能自带换行 "\n"，所以这里不再额外添加换行。

except KeyboardInterrupt:
    print("\n检测到 Ctrl+C，准备退出。")

finally:
    client_socket.close()
    print("客户端套接字已关闭。")