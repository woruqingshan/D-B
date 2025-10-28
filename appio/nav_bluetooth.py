# ================================
# file: appio/nav_bluetooth.py
# ================================
"""
Navigation-specific BLE interface for real robot navigation
Extends the basic BLE functionality with navigation-specific commands and data parsing
"""

from __future__ import annotations
from typing import Optional, List
import threading
import time
import queue

try:
    import serial
except Exception:
    serial = None


class NavBleInterface:
    """Navigation-specific BLE serial wrapper for real robot navigation.
    
    Features:
    - Commands: "S" (lidar on), "T" (lidar off), "(M,<deg>,<mm>)" (turn/move)
    - Events: 'arrival', 'turning finished'
    - Data: LIDAR lines and ODOMETRY DATA blocks
    - Thread-safe queue-based data reading
    """
    
    def __init__(self, port="COM4", baud=921600, timeout=0.01):
        self.port, self.baud, self.timeout = port, baud, timeout
        self.ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        
        # High baudrate optimization
        from core.config import HIGH_BAUDRATE_MODE, HIGH_BAUDRATE_QUEUE_SIZE, HIGH_BAUDRATE_TIMEOUT
        if HIGH_BAUDRATE_MODE and baud >= 921600:
            self.lines = queue.Queue(maxsize=HIGH_BAUDRATE_QUEUE_SIZE)
            self.timeout = HIGH_BAUDRATE_TIMEOUT
            print(f"[HIGH_BAUD] 启用高波特率优化模式: {baud}")
        else:
            self.lines = queue.Queue(maxsize=4096)

    def connect(self):
        """Connect to the robot via serial port"""
        if serial is None:
            raise RuntimeError("pyserial not installed")
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(1.5)  # MCU warm-up
        self._running = True
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()

    def close(self):
        """Close the connection"""
        self._running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def send(self, cmd: str):
        """Send command to robot"""
        if not self.ser or not self.ser.is_open:
            return
        data = (cmd.strip() + "\n").encode("utf-8", errors="ignore")
        self.ser.write(data)
        self.ser.flush()

    def _reader(self):
        """Background thread for reading serial data - 优化：批量读取"""
        import codecs
        
        # 初始化解码器和缓冲区
        buf = bytearray()
        dec = codecs.getincrementaldecoder('utf-8')('ignore')
        
        while self._running and self.ser and self.ser.is_open:
            try:
                # 批量读取可用数据
                n = self.ser.in_waiting
                if n > 0:
                    chunk = self.ser.read(n)
                    buf.extend(chunk)
                
                # 处理完整的行
                while b'\n' in buf:
                    line_bytes, buf = buf.split(b'\n', 1)
                    if line_bytes.endswith(b'\r'):
                        line_bytes = line_bytes[:-1]
                    
                    line = dec.decode(line_bytes, final=False)
                    if line:
                        # 使用非阻塞put避免数据丢失
                        try:
                            self.lines.put_nowait(line)
                        except Exception:
                            # 队列满，丢弃最旧的数据
                            try:
                                _ = self.lines.get_nowait()
                                self.lines.put_nowait(line)
                            except Exception:
                                pass  # 放弃这行数据
                else:
                    time.sleep(0.001)  # 没有完整行时短暂等待
                    
            except Exception:
                time.sleep(0.01)

    def get_line_nowait(self) -> Optional[str]:
        """Get a single line from queue without blocking"""
        try:
            return self.lines.get_nowait()
        except Exception:
            return None
    
    def drain_all_lines(self) -> List[str]:
        """Extract all lines from queue at once"""
        out = []
        while True:
            try:
                out.append(self.lines.get_nowait())
            except Exception:
                break
        return out
    

    def readline(self) -> str:
        """Direct blocking serial readline"""
        if not self.ser or not self.ser.is_open:
            return ""
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            return line
        except Exception:
            return ""

    def drain_after_T(self, idle_gap_s: float = 0.30, max_wait_s: float = 1.0):
        """After sending 'T', keep reading until no more lidar lines"""
        t_end = time.time() + max_wait_s
        last_seen = time.time()
        while time.time() < t_end:
            ln = self.get_line_nowait()
            if ln:
                last_seen = time.time()
                continue
            if time.time() - last_seen >= idle_gap_s:
                break
            time.sleep(0.01)

    def read_batch_lines(self, max_lines: int = 50) -> List[str]:
        """
        单线程批量读取串口数据，避免多线程开销
        
        Args:
            max_lines: 最大读取行数，避免一次读取过多数据
            
        Returns:
            读取到的行列表
        """
        if not self.ser or not self.ser.is_open:
            return []
        
        lines = []
        attempts = 0
        max_attempts = max_lines * 2  # 允许更多尝试，避免过早退出
        
        while len(lines) < max_lines and attempts < max_attempts:
            try:
                line = self.ser.readline()
                if line:
                    decoded_line = line.decode("utf-8", errors="ignore").strip()
                    if decoded_line:
                        lines.append(decoded_line)
                else:
                    time.sleep(0.001)  # 短暂等待，避免立即退出
                attempts += 1
            except Exception:
                break
        
        return lines

    def read_batch_lines_filtered(self, max_lines: int = 50, important_prefixes: tuple = None) -> List[str]:
        """
        单线程批量读取并预处理过滤，只返回关键行
        
        Args:
            max_lines: 最大读取行数
            important_prefixes: 重要行前缀，默认为LIDAR/Angle/ODOM相关
            
        Returns:
            过滤后的关键行列表
        """
        if important_prefixes is None:
            important_prefixes = ("LIDAR", "Angle:", "ODOM", "Odom", "ODOMETRY")
        
        all_lines = self.read_batch_lines(max_lines)
        
        # 预处理过滤，只保留关键行 - 移除调试打印以提升性能
        filtered_lines = []
        for line in all_lines:
            if line.startswith(important_prefixes):
                filtered_lines.append(line)
            # 移除调试打印，避免I/O瓶颈
        
        return filtered_lines
