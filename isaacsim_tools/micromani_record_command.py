"""向本地仿真录制器发送操作并等待确认。"""
import argparse
import json
import socket


def main():
    p=argparse.ArgumentParser()
    p.add_argument('op', choices=['start','save','discard','reset','finish','axes'])
    p.add_argument('--task')
    p.add_argument('--outcome', choices=['unreviewed','success','failure'], default='unreviewed')
    p.add_argument('--axes', type=float, nargs=14)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=49010)
    a=p.parse_args()
    if a.op == 'start' and not a.task:
        p.error('start requires --task')
    if a.op == 'axes' and a.axes is None:
        p.error('axes requires --axes with 14 values')
    packet={'axes':a.axes} if a.op == 'axes' else {'op':a.op,'task':a.task,'outcome':a.outcome}
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.settimeout(15)
        sock.connect((a.host,a.port))
        sock.send(json.dumps(packet,allow_nan=False).encode())
        reply=json.loads(sock.recv(65535))
    print(json.dumps(reply,ensure_ascii=False))
    if not reply.get('ok'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
