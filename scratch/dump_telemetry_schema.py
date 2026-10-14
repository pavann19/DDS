"""
Inspect one live telemetry frame schema
"""
import asyncio
import json
import websockets

async def dump_frame():
    async with websockets.connect("ws://127.0.0.1:8000/ws/telemetry") as ws:
        for _ in range(5):
            msg = await ws.recv()
            data = json.loads(msg)
            if "channels" in data:
                print("Top keys:", list(data.keys()))
                channels = data["channels"]
                print("Channels keys:", list(channels.keys()))
                for ch_name, ch_val in channels.items():
                    print(f"--- Channel: {ch_name} ---")
                    if isinstance(ch_val, dict):
                        for sub_k, sub_v in ch_val.items():
                            print(f"  {sub_k}: type={type(sub_v).__name__}, preview={str(sub_v)[:100]}")
                    else:
                        print(f"  value: type={type(ch_val).__name__}, preview={str(ch_val)[:100]}")
                break

if __name__ == "__main__":
    asyncio.run(dump_frame())
