from bleak import BleakScanner, BleakClient

class Blue:
    def __init__(self):
        self.CURRENT_CLIENT: BleakClient | None = None

    async def scan(self, duration: float):
        devices = await BleakScanner.discover(duration)
        return devices

    async def connect(self, address: str):
        if self.CURRENT_CLIENT and self.CURRENT_CLIENT.is_connected:
            await self.CURRENT_CLIENT.disconnect()

        print(f"Connecting to {address} ...")
        client = BleakClient(address)

        await client.connect()
        if not client.is_connected:
            print("Failed to connect")
            return False

        print("Connected")

        print("Services discovered:")
        for service in client.services:
            print(f"[Service] {service.uuid}: {service.description}")
            for char in service.characteristics:
                print(f"  [Characteristic] {char.uuid}: {char.description}")

        self.CURRENT_CLIENT = client
        return True

    async def write(self, uuid: str, data: str):
        if self.CURRENT_CLIENT is None or not self.CURRENT_CLIENT.is_connected:
            print("No client connected.")
            return

        if not self.CURRENT_CLIENT.services:
            await self.CURRENT_CLIENT.connect()

        if isinstance(data, str):
            data = data.encode()

        print(f"Writing to {uuid}: {data}")
        await self.CURRENT_CLIENT.write_gatt_char(uuid, data)

    async def disconnect(self):
        if self.CURRENT_CLIENT and self.CURRENT_CLIENT.is_connected:
            print("Disconnecting...")
            await self.CURRENT_CLIENT.disconnect()
            print("Disconnected.")

            return True
