import asyncio
import websockets
import json

class RobotWebSocketServer:
    def __init__(self):
        self.simServer = None

    def update_robot_position(self, x, y, z):
        print(f"Updated robot position to: x={x}, y={y}, z={z}")

    def handle_tracking_data(self, data):
        robot_position = data.get("position", None)
        if robot_position:
            x, y, z = robot_position
            self.update_robot_position(x, y, z)

    async def handle_message(self, message):
        try:
            data = json.loads(message)
            if data.get('type') == 'tracking':
                self.handle_tracking_data(data)
            elif data.get('type') == 'control':
                print("Received control message:", data)
                # Process control data here
            else:
                print(f"Unknown message type: {data.get('type')}")
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
        except Exception as e:
            print(f"Error processing message: {e}")

    async def handler(self, websocket):
        try:
            async for message in websocket:
                print("Received message:", message)
                await self.handle_message(message)
        except websockets.exceptions.ConnectionClosed:
            print('WebSocket connection closed')
        finally:
            print('WebSocket closed')

    async def start_server(self):
        print("Starting WebSocket server...")
        self.simServer = await websockets.serve(self.handler, "localhost", 8765)
        print("WebSocket server started. Waiting for connections...")
        await self.simServer.wait_closed()

async def main():
    server = RobotWebSocketServer()
    await server.start_server()

if __name__ == "__main__":
    asyncio.run(main())