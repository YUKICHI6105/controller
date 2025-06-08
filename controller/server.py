# server.py
from ament_index_python.packages import get_package_share_directory
import asyncio
import websockets
import socket
import ssl
import qrcode
import os
from aiohttp import web
import json
import netifaces

share_dir = get_package_share_directory("controller")

HTML_PATH = os.path.join(share_dir, "data_files/index.html")
CERT_PATH = os.path.join(share_dir, "data_files/certificate.pem")
KEY_PATH  = os.path.join(share_dir, "data_files/key.pem")

def generate_qr_code(data, filename="ip_qrcode.png"):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    img.save(filename)
    print(f"QRcode is saved as {filename}")

def generate_ip_addresses_qrcode():
    ip_map = {}
    for iface in netifaces.interfaces():
        addrs = netifaces.ifaddresses(iface)
        inet = addrs.get(netifaces.AF_INET)
        if inet:
            for entry in inet:
                ip = entry.get('addr')
                if ip and not ip.startswith('127.'):
                    ip_map[iface] = ip

    print("非ループバックIPアドレス一覧:")
    for iface, ip in ip_map.items():
        print(f"{iface}: {ip}")
        generate_qr_code(f"https://{ip}:8081", f"{iface}_ip_qrcode.png")

async def handle_index(request):
    return web.FileResponse(HTML_PATH)

def create_ssl_context():
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(certfile=CERT_PATH, keyfile=KEY_PATH)
    return ssl_context

async def handle_connection(websocket, callback):
    try:
        async for message in websocket:
            # print(f"received data: {message}")
            if message == "Hello server!":
                print("Hello server!")
                await websocket.send("Hello client!")
            elif message == "sleep blocked":
                print("sleep blocked")
            else:
                try:
                    data = json.loads(message)
                    print("data received")
                    await callback(data)  # ← コールバック呼び出し
                except json.JSONDecodeError:
                    print("Invalid JSON received")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except websockets.ConnectionClosed:
        print("Connection dropped by client")
    except Exception as e:
        print(f"Websocket error: {e}")

async def start_server(callback):
    # Generate QR codes for IP addresses
    generate_ip_addresses_qrcode()

    ssl_context = create_ssl_context()

    # HTTP server
    app = web.Application()
    app.router.add_get('/', handle_index)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8081, ssl_context=ssl_context)
    await site.start()

    # WebSocket server
    await websockets.serve(lambda ws: handle_connection(ws, callback), "0.0.0.0", 8765, ssl=ssl_context)
    print("WebSocket server is started with HTTPS.")

    await asyncio.Event().wait()
