#!/usr/bin/env python3
"""Update only local ESP-IDF network options without displaying credentials."""
from __future__ import annotations

import argparse
import getpass
import ipaddress
import json
from pathlib import Path
import re
import subprocess
import sys


def configure(config: Path, ssid: str, host: str | None, password: str) -> None:
    address = None if host is None else ipaddress.IPv4Address(host)
    if address is not None and (address.is_loopback or address.is_multicast or address.is_unspecified or str(address) == '255.255.255.255'):
        raise ValueError('Receiver must be a reachable LAN IPv4 address')
    if not 1 <= len(ssid.encode('utf-8')) <= 32 or any(c in ssid for c in '\x00\r\n'):
        raise ValueError('SSID must contain 1-32 UTF-8 bytes and no line breaks')
    valid_password = (password == '' or 8 <= len(password.encode('utf-8')) <= 63
                      or bool(re.fullmatch(r'[0-9a-fA-F]{64}', password)))
    if not valid_password or any(c in password for c in '\x00\r\n'):
        raise ValueError('Wi-Fi password must be empty, 8-63 bytes, or a 64-digit hexadecimal key')
    content = config.read_text(encoding='utf-8')
    values = {'SMARTCAR_WIFI_SSID': ssid, 'SMARTCAR_WIFI_PASSWORD': password}
    if address is not None:
        values['SMARTCAR_HOST_IPV4'] = str(address)
    for name, value in values.items():
        line = f'CONFIG_{name}={json.dumps(value, ensure_ascii=False)}'
        pattern = rf'^CONFIG_{name}=.*$'
        if re.search(pattern, content, re.MULTILINE):
            content = re.sub(pattern, lambda _: line, content, flags=re.MULTILINE)
        else:
            content += '\n' + line + '\n'
    # Deliberately do not create a second credentials file or print its contents.
    with config.open('w', encoding='utf-8') as stream:
        stream.write(content)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ssid', required=True)
    parser.add_argument('--host', help='Optional legacy fixed receiver IPv4; auto discovery does not need it')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] / 'firmware/sdkconfig')
    parser.add_argument('--mac-dialog', action='store_true', help='Request password in a hidden local macOS dialog')
    args = parser.parse_args()
    if not args.config.is_file():
        parser.error('Create the local sdkconfig with idf.py menuconfig first')
    if args.mac_dialog:
        # Password is captured in process memory only, never printed or passed as an argument.
        message = f'请输入 {args.ssid} 的 Wi-Fi 密码。仅保存到本机 S3 配置，不会发送到聊天。'
        script = 'text returned of (display dialog ' + json.dumps(message, ensure_ascii=False) + ' default answer "" with hidden answer buttons {"取消", "保存"} default button "保存")'
        result = subprocess.run(['/usr/bin/osascript', '-e', script], capture_output=True, text=True)
        if result.returncode:
            print('网络配置未更改：本机密码输入已取消或窗口不可用。')
            return 1
        password = result.stdout.removesuffix('\n')
    else:
        password = getpass.getpass(f'Wi-Fi password for {args.ssid} (hidden): ')
    try:
        configure(args.config, args.ssid, args.host, password)
    except (ValueError, OSError):
        print('网络配置未完成：请检查 SSID、IPv4、密码长度及本机配置文件。', file=sys.stderr)
        return 1
    route = 'automatic discovery' if args.host is None else f'legacy receiver={args.host}:8888'
    print(f'Updated local configuration: SSID={args.ssid}; {route}; password hidden.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
