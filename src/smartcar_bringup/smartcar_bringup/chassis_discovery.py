"""Find the chassis using the sensor HAL's discovery and renewable lease."""
from __future__ import annotations

import argparse
import ipaddress
import json
import signal
import socket
import subprocess
import time

from smartcar_sensor_hal.discovery import DiscoveryClient


def default_ipv4_network(interface=''):
    """Find an interface IPv4 and directed broadcast, avoiding TUN broadcasts."""
    routes = ([{'dev': interface}] if interface else
              json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'show', 'default'], text=True)))
    for route in sorted(routes, key=lambda item: item.get('metric', 0)):
        device = route.get('dev')
        if not device:
            continue
        addresses = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show', 'dev', device], text=True))
        for interface in addresses:
            for address in interface.get('addr_info', []):
                if address.get('family') == 'inet' and address.get('scope') == 'global':
                    network = ipaddress.IPv4Interface(f"{address['local']}/{address['prefixlen']}")
                    if network.network.prefixlen <= 30:
                        return device, str(network.ip), str(network.network.broadcast_address)
    raise RuntimeError('no IPv4 default route with a broadcast-capable subnet')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--agent-port', type=int, default=8891)
    parser.add_argument('--discovery-port', type=int, default=8892)
    parser.add_argument('--broadcast', default='')
    parser.add_argument('--device-id', default='')
    parser.add_argument('--device-ip', default='')
    parser.add_argument('--interface', default='')
    args, _ = parser.parse_known_args()
    if not 1024 <= args.agent_port <= 65535 or args.agent_port == args.discovery_port:
        parser.error('agent port must be 1024..65535 and differ from discovery port')
    if args.broadcast:
        ipaddress.IPv4Address(args.broadcast)

    running = True
    def stop(_signum, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    client = None
    network = False
    pinned_id = args.device_id
    next_network_check = 0.0
    previous = None
    try:
        while running:
            now = time.monotonic()
            if now >= next_network_check:
                next_network_check = now + 3.0
                try:
                    current = default_ipv4_network(args.interface)
                except (OSError, subprocess.CalledProcessError, ValueError, RuntimeError, KeyError, IndexError):
                    current = None
                if current != network:
                    if client:
                        pinned_id = client.status().device_id or pinned_id
                        client.close()
                        client = None
                    network = current
                    previous = None
                    if network:
                        interface, source, directed_broadcast = network
                        broadcast = args.broadcast or directed_broadcast
                        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        try:
                            sock.bind((source, 0))
                            client = DiscoveryClient(data_port=args.agent_port, device_id=pinned_id or '',
                                                     target_ip=args.device_ip, broadcast_address=broadcast,
                                                     port=args.discovery_port, sock=sock)
                        except OSError as error:
                            sock.close()
                            network = None
                            print(f'chassis discovery cannot bind {source}: {error}; retrying', flush=True)
                            continue
                        except Exception:
                            sock.close()
                            raise
                        print(f'chassis discovery on {interface} ({source}), broadcast {broadcast}, '
                              f'UDP {args.discovery_port}; Agent UDP {args.agent_port}', flush=True)
                    else:
                        print('chassis discovery waiting for a usable IPv4 default route', flush=True)
            if client:
                status = client.poll()
                pinned_id = status.device_id or pinned_id
                summary = (status.state, status.reason, status.device_id, status.source_ip)
                if summary != previous:
                    print(f'chassis discovery: state={status.state} reason={status.reason} '
                          f'device={status.device_id or "-"} ip={status.source_ip or "-"} '
                          f'candidates={status.candidates}', flush=True)
                    previous = summary
            time.sleep(0.05)
    finally:
        if client:
            client.close()


if __name__ == '__main__':
    main()
