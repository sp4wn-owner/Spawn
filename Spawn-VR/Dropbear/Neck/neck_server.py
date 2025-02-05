# pip install websockets
# Running this server on Windows may require opening port 8765
import asyncio
import websockets
import json

class neckWsServer:
    def __init__(self):
        self.neckServer = None

    async def handle_message(self, message):
        try:
            data = json.loads(message)
            if data.get('type') == 'tracking':
                head_orientation = data.get('head', {}).get('orientation')
                if head_orientation:
                    head_quaternion = [head_orientation['w'], head_orientation['x'], head_orientation['y'], head_orientation['z']]
                    print("Received head quaternion:", head_quaternion)
                    # Here, you can further process or handle the quaternion data
                    # Instead of sending to serial directly, we'll pass the message to another script
                    #    import subprocess
                    #    try:
                    #        subprocess.run(["python", "serial_handler.py", message], check=True)
                    #    except subprocess.CalledProcessError as e:
                    #        print(f"Error running serial handler script: {e}")
                else:
                    print("No head orientation data in tracking message")
            
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
        except Exception as e:
            print(f"Error processing message: {e}")
        
    async def handler(self, websocket):
        try:
            async for message in websocket:
                # print("Received message:", message)
                await self.handle_message(message)
        except websockets.exceptions.ConnectionClosed:
            print('WebSocket connection closed')
        finally:
            print('WebSocket closed')

    async def start_server(self):
        print("Starting WebSocket server...")
        self.neckServer = await websockets.serve(self.handler, "localhost", 8765)
        print("WebSocket server started. Waiting for connections...")
        await self.neckServer.wait_closed()

async def main():
    server = neckWsServer()
    await server.start_server()

if __name__ == "__main__":
    asyncio.run(main())