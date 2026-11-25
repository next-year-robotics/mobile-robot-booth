"""브링업 완료 전에 대시보드 WebSocket에서 전방 영상 한 장을 확인한다."""
import asyncio
import json

import websockets


async def wait_frame():
    async with websockets.connect("ws://127.0.0.1:8080/ws", max_size=None) as ws:
        async for message in ws:
            if isinstance(message, bytes):
                end = 4 + int.from_bytes(message[:4], "little")
                header = json.loads(message[4:end])
                if header["type"] == "frame" and len(message) > end:
                    print(f"  전방 영상 수신 ({header['w']}×{header['h']})")
                    return
    raise ConnectionError("대시보드 연결 종료")


if __name__ == "__main__":
    try:
        asyncio.run(asyncio.wait_for(wait_frame(), timeout=45))
    except TimeoutError:
        raise SystemExit("45초 동안 전방 영상이 들어오지 않았습니다.")
