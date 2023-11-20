import socket
from time import perf_counter_ns, sleep  # For example only


def send_udp_trigger(target_ip: str, port: int = 9090):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(b'TRIG', (target_ip, port))
    s.close()


def send_tcp_trigger(target_ip: str, port: int = 9090):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((target_ip, port))
    s.sendall(b'TRIG')
    d = s.recv(1024)
    if d != b'ACK':
        raise Exception(f'Trigger handshake error: got "{d}"')
    s.close()


# Examples of use

print('Sending UDP trigger example')
send_udp_trigger('localhost')
print('Done.')

print('Sending TCP trigger example')
send_tcp_trigger('localhost')
print('Done.')

print('Testing performance (3~5sek)')
times = []
for i in range(300):
    sleep(0.01)
    tic = perf_counter_ns()
    send_tcp_trigger('localhost')
    toc = perf_counter_ns()
    times.append(toc-tic)

print(f"""TCP Trigger time in microseconds:
 - Min: {min(times)/1000:.3f} µs
 - Max: {max(times)/1000:.3f} µs
 - Avg: {sum(times)/len(times)/1000:.3f} µs
""")
