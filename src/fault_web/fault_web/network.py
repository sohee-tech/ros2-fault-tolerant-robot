"""Find the address phones can actually reach and build the QR code. Never returns a loopback address."""
import io
import ipaddress
import json
import socket
import subprocess

import qrcode

VIRTUAL_PREFIXES = ('lo', 'docker', 'br-', 'veth', 'virbr', 'vmnet', 'vboxnet', 'tun', 'tap', 'wg')


def usable(ifname, ip):
    if ifname.startswith(VIRTUAL_PREFIXES):
        return False
    addr = ipaddress.ip_address(ip)
    return not (addr.is_loopback or addr.is_link_local or addr.is_multicast or addr.is_unspecified)


def interface_addresses():
    """[(ifname, ip)] of interfaces that are UP."""
    out = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'up'], capture_output=True, text=True, timeout=2.0).stdout
    return [(link['ifname'], a['local']) for link in json.loads(out or '[]') for a in link.get('addr_info', [])]


def default_route_ip():
    """Source address of the default route. UDP connect() only selects a route, it sends nothing."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(('10.254.254.254', 9))
            return s.getsockname()[0]
        except OSError:
            return None


def candidates(addresses=None, primary=None):
    """Usable IPs, the default-route address first."""
    addresses = interface_addresses() if addresses is None else addresses
    ips = [ip for name, ip in addresses if usable(name, ip)]
    primary = default_route_ip() if primary is None else primary
    return sorted(ips, key=lambda ip: ip != primary)


def nmcli(*args):
    """Output of a read-only nmcli query, '' when unavailable."""
    try:
        return subprocess.run(['nmcli', '-t', *args], capture_output=True, text=True, timeout=1.5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def network_info(iface):
    """{'name', 'enterprise'}: 802.1X Wi-Fi (campus, company) often blocks traffic between devices."""
    if not iface:
        return {'name': None, 'enterprise': False}
    active = [line for line in nmcli('-f', 'active,security', 'device', 'wifi', 'list', 'ifname', iface, '--rescan', 'no').splitlines()
              if line.startswith('yes:')]
    return {'name': nmcli('-g', 'GENERAL.CONNECTION', 'device', 'show', iface) or None,
            'enterprise': any('802.1X' in line for line in active)}


def describe(port, public_host=None):
    """{'url', 'urls', 'hosts', 'ok'}: url is None when no reachable address exists (no localhost fallback)."""
    try:
        addresses = interface_addresses()
        hosts = [public_host] if public_host else candidates(addresses)
    except (OSError, ValueError, subprocess.SubprocessError):
        addresses, hosts = [], []
    iface_of = {ip: name for name, ip in addresses}
    details = [{'ip': h, 'iface': iface_of.get(h), 'url': f'http://{h}:{port}/', **network_info(iface_of.get(h))}
               for h in hosts]
    urls = [d['url'] for d in details]
    return {'ok': bool(urls), 'url': urls[0] if urls else None, 'urls': urls, 'hosts': details, 'port': port,
            'own_ips': sorted({ip for _, ip in addresses} | {'127.0.0.1', '::1'})}


def qr_png(url):
    img = qrcode.make(url, box_size=10, border=4)   # quiet zone required by the QR standard
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()


def qr_ascii(url):
    qr = qrcode.QRCode(border=2)
    qr.add_data(url)
    buf = io.StringIO()
    qr.print_ascii(out=buf, invert=True)
    return buf.getvalue()
