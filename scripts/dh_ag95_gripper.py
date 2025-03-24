import socket
import time

class ag95_gripper:
    def __init__(self):
        # IP and port configuration
        ip = '169.254.128.19'
        port_no = 8080

        # Create a socket and connect to the server
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port_no))
        print("First connection to the robotic arm", ip)

        time.sleep(3)


        point6_00 = '{"command":"set_tool_voltage","voltage_type":3}\r\n'
        _ = self.send_cmd(self.client, cmd_6axis=point6_00)
        time.sleep(3)
        print("Setting tool voltage output to 24V")

        point6_00 = '{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout ":2}\r\n'
        _ = self.send_cmd(self.client, cmd_6axis=point6_00)
        time.sleep(2)
        print("Configuring communication port to ModbusRTU mode")

        point6_00 = '{"command":"write_single_register","port":1,"address":256,"data":1, "device":1}\r\n'
        _ = self.send_cmd(self.client, cmd_6axis=point6_00)
        time.sleep(2)
        print("Initialization Successful")
        #point6_00 = '{"command":"write_single_register","port":1,"address":257,"data":30, "device":1}\r\n'
        #_ = send_cmd(self.client, cmd_6axis=point6_00)
        #time.sleep(2)
        #print(" 设置30% 力值 （写操作）")

    def send_cmd(self, client, cmd_6axis):
        client.send(cmd_6axis.encode('utf-8'))
        # Optional: Receive a response from the server
        # _ = self.client.recv(1024).decode()
        return True



    def open(self):
        point6_00 = '{"command":"write_single_register","port":1,"address":259,"data":1000, "device":1}\r\n'
        _ = self.send_cmd(self.client, cmd_6axis=point6_00)
        time.sleep(0.001)

    def semi_open(self):
        point6_00 = '{"command":"write_single_register","port":1,"address":259,"data":500, "device":1}\r\n'
        _ = self.send_cmd(self.client, cmd_6axis=point6_00)
        time.sleep(0.001)

    def close(self):
        point6_00 = '{"command":"write_single_register","port":1,"address":259,"data":0, "device":1}\r\n'
        _ = self.send_cmd(self.client, cmd_6axis=point6_00)
        time.sleep(0.001)
    
    def command(self, command):
        if command == 0:
            self.close()
        elif command == 500:
            self.semi_open()
        elif command == 1000:
            self.open()