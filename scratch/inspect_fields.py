"""
Inspect complete channel fields
"""
import asyncio
import json
import websockets

async def inspect_fields():
    async with websockets.connect("ws://127.0.0.1:8000/ws/telemetry") as ws:
        # read 10 frames
        for _ in range(10):
            msg = await ws.recv()
            data = json.loads(msg)
            if "channels" in data:
                print("\n=== EGO POSE ===")
                print(json.dumps(data["channels"]["pose"]["ego"], indent=2))

                print("\n=== PLANNER ===")
                p = data["channels"]["semantic"].get("planner", {})
                print(f"trajectory points: {len(p.get('trajectory', []))}")
                print(f"trajectory preview: {p.get('trajectory', [])[:3]}")
                print({k: v for k, v in p.items() if k != "trajectory"})

                print("\n=== SAFETY SHIELD ===")
                print(json.dumps(data["channels"]["semantic"].get("safety_shield", {}), indent=2))

                print("\n=== DRIVER ANALYTICS ===")
                print(json.dumps(data["channels"]["semantic"].get("driver_analytics", {}), indent=2))

                print("\n=== PREDICTION ===")
                print(json.dumps(data["channels"]["heavy"].get("prediction", {}), indent=2))

                print("\n=== PERCEPTION & TRAFFIC ===")
                print(f"traffic count: {len(data['channels']['semantic'].get('traffic', []))}")
                print(f"perception detections: {len(data['channels']['semantic'].get('perception', []))}")
                print(f"surround_perception count: {len(data['channels']['heavy'].get('surround_perception', []))}")
                break

if __name__ == "__main__":
    asyncio.run(inspect_fields())
