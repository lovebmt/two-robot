import asyncio

from tworobot.links import NetLink


def test_poll_tolerates_wifi_latency_above_500ms():
    async def go():
        async def handler(reader, writer):
            await reader.readuntil(b"\r\n\r\n")
            await asyncio.sleep(0.65)
            body = b'{"tel":{"t":"tel","state":"IDLE"},"events":[],"eseq":0}'
            writer.write(
                b"HTTP/1.1 200 OK\r\n"
                b"Content-Type: application/json\r\n"
                b"Content-Length: " + str(len(body)).encode() + b"\r\n"
                b"Connection: close\r\n\r\n" + body
            )
            await writer.drain()
            writer.close()
            await writer.wait_closed()

        server = await asyncio.start_server(handler, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        link = NetLink(f"http://127.0.0.1:{port}")
        got = asyncio.get_running_loop().create_future()
        link.on_message = lambda msg: got.done() or got.set_result(msg)
        task = asyncio.create_task(link._poll())
        try:
            msg = await asyncio.wait_for(got, 1.5)
            assert msg["state"] == "IDLE"
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            link.close()
            server.close()
            await server.wait_closed()

    asyncio.run(go())
