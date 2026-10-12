import subprocess
import threading
import time

# Topics to check
TOPICS = [
    ('/scan', 'Laser Scan'),
    ('/odom', 'Odometry'),
    ('/depth_cam/depth/image_raw', 'Depth Image'),
    ('/depth_cam/rgb/image_raw', 'RGB Image'),
]

# Function to run ros2 topic hz
def run_hz(topic, label):
    print(f'\n===== {label} ({topic}) Rate =====')
    try:
        subprocess.run(['ros2', 'topic', 'hz', topic, '-w', '20'], timeout=10)
    except Exception as e:
        print(f'Error running hz for {topic}: {e}')

def run_echo(topic, label, lines=5):
    print(f'\n===== {label} ({topic}) Echo (showing {lines} messages) =====')
    try:
        proc = subprocess.Popen(['ros2', 'topic', 'echo', topic, '--qos-reliability', 'best_effort'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        count = 0
        for line in proc.stdout:
            print(line, end='')
            if 'stamp:' in line or 'header:' in line:
                count += 1
            if count >= lines:
                proc.terminate()
                break
    except Exception as e:
        print(f'Error running echo for {topic}: {e}')

def main():
    print('Checking sensor topic rates...')
    for topic, label in TOPICS:
        run_hz(topic, label)
        time.sleep(1)
    print('\nChecking sensor topic timestamps (header.stamp)...')
    for topic, label in TOPICS:
        run_echo(topic, label, lines=5)
        time.sleep(1)

if __name__ == '__main__':
    main()
