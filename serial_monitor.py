"""
STM32 串口调试监视器
用法: python serial_monitor.py COM13 115200
按 Ctrl+C 退出
"""
import sys
import serial
import time

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM13"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"[*] 串口: {port} | 波特率: {baud}")
    print(f"[*] 等待数据... (Ctrl+C 退出)\n")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[+] 已连接 {port}")
        print("-" * 50)

        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                try:
                    text = data.decode('utf-8', errors='replace')
                    sys.stdout.write(text)
                    sys.stdout.flush()
                except:
                    print(f"[HEX] {data.hex()}")

            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"[!] 串口错误: {e}")
        print(f"    请检查:")
        print(f"    1. {port} 是否被其他程序占用")
        print(f"    2. CH340 是否已连接电脑")
        print(f"    3. 查看可用串口:")
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"       {p.device} - {p.description}")
    except KeyboardInterrupt:
        print("\n[*] 已退出")
    finally:
        if 'ser' in dir():
            ser.close()
            print("[*] 串口已关闭")

if __name__ == "__main__":
    main()
