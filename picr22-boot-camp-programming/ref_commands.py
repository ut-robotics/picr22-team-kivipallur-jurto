import json, time, queue
import multiprocessing as mp    
import websocket as wsc
import numpy as np

#
#This code is inspired from Team OWO-s referee commands.
#

class Referee_cmd_client:
    def __init__(self):
        #ip and port
        self.ip = "192.168.3.40"
        self.port = "8222"

        #define queue
        self.queue = mp.Queue()

    def open(self):
        #opening and connecting to websocket
        self.ws = wsc.WebSocket()
        self.connect()
        self.process = mp.Process(target=self.listen, args=())
        self.process.start()

    def close(self):
        self.process.join() #waiting for process to finish and closing websocket
        self.ws.close()

    #from this point on, we got help from team OWO:
    def connect(self):
        for i in range(5): #trying to connect 5 times
            try:
                self.ws.connect("ws://" + self.ip + ":" + self.port) #text formatting and connecting
            except ConnectionRefusedError:
                print("Error")
                time.sleep(2)
                continue
            else:
                print("No error")
                return True
                break
        return False

    def get_cmd(self):
        try:
            return self.queue.get_nowait() #reading from queue
        except queue.Empty:
            return None

    def listen(self):
        while True:
            print("listening")
            try:
                msg = self.ws.recv()
            except wsc.WebSocketConnectionClosedException:
                if self.connect():
                    continue
                else:
                    break
            try:
                self.queue.put(json.loads(msg)) #json loading the image so we can get it as a dictionary
            except json.JSONDecodeError:
                continue

if __name__ == "__main__":
    client = Referee_cmd_client()
    client.open()
    try:

        while(True):
            print("getting")
            msg = client.get_cmd()
            print(msg)
            time.sleep(1)

            
    except KeyboardInterrupt:
        client.close()