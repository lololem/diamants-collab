#!/usr/bin/env python3
"""Simulate 4 Crazyflie drones sending telemetry to DiamantsBridge.
Runs indefinitely until Ctrl+C.  Reconnects automatically if the
WebSocket connection drops.
"""
import asyncio, json, math, sys, time
import websockets

URI = 'ws://127.0.0.1:8765'
DRONE_IDS = [f'crazyflie_{i:02d}' for i in range(1, 5)]
HZ = 10  # send rate


async def send_loop():
    """Connect, stream positions, reconnect on failure."""
    step = 0
    while True:
        try:
            async with websockets.connect(URI, ping_interval=20, ping_timeout=10) as ws:
                await ws.recv()  # skip initial_state
                print(f'✈️  Connected – streaming {len(DRONE_IDS)} drones at {HZ} Hz …',
                      flush=True)
                while True:
                    t = step * (1.0 / HZ)
                    positions = {}
                    for i, d_id in enumerate(DRONE_IDS):
                        angle = t * 0.3 + i * (math.pi / 2)
                        radius = 5.0 + i * 2.0
                        positions[d_id] = {
                            'position': {
                                'x': round(math.cos(angle) * radius, 2),
                                'y': round(9.5 + math.sin(t * 0.5 + i) * 1.0, 2),
                                'z': round(math.sin(angle) * radius, 2),
                            },
                            'status': 'flying',
                            'battery': round(95.0 - (step % 1000) * 0.01, 1),
                            'last_seen': time.time(),
                        }
                    msg = {
                        'type': 'telemetry_positions',
                        'data': {'drones': positions},
                        'timestamp': time.time(),
                    }
                    await ws.send(json.dumps(msg))

                    # Also send propeller speeds (hover ~14200 RPM ± variation)
                    prop_speeds = {}
                    for i, d_id in enumerate(DRONE_IDS):
                        # Simulate slight RPM variation per motor (PID oscillation)
                        base_rpm = 14200
                        variation = math.sin(t * 5.0 + i * 1.5) * 200
                        prop_speeds[d_id] = [
                            round(base_rpm + variation, 0),
                            round(base_rpm - variation * 0.8, 0),
                            round(base_rpm + variation * 0.6, 0),
                            round(base_rpm - variation * 0.4, 0),
                        ]
                    await ws.send(json.dumps({
                        'type': 'propeller_speeds',
                        'data': prop_speeds,
                        'timestamp': time.time(),
                    }))
                    if step % (HZ * 10) == 0:  # log every 10 s
                        print(f'  t={t:.0f}s  step={step}', flush=True)
                    step += 1
                    await asyncio.sleep(1.0 / HZ)
        except (websockets.exceptions.ConnectionClosed,
                websockets.exceptions.InvalidStatusCode,
                ConnectionRefusedError,
                OSError) as exc:
            print(f'⚠️  WS lost ({exc!r}) – retrying in 2 s …', flush=True)
            await asyncio.sleep(2)


try:
    asyncio.run(send_loop())
except KeyboardInterrupt:
    print('\n🛑 Simulation stopped')
