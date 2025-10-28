# ================================
# file: test_nav_modular.py
# ================================
"""
Modular Navigation Test
使用模块化的导航和建图组件进行测试
"""

import argparse
import time
import sys
import os

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from nav.nav_mapping_app import NavMappingApp

# Global variable to store terminal log file handle
_terminal_log_file = None
_original_stdout = None
_original_stderr = None


def setup_early_logging():
    """Setup terminal logging before NavMappingApp initialization"""
    global _terminal_log_file, _original_stdout, _original_stderr
    
    try:
        # Create log directory
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        base_logdir = os.path.join("logs", "nav_exploring")
        txt_output_dir = os.path.join(base_logdir, "txt_output")
        os.makedirs(txt_output_dir, exist_ok=True)
        
        # Create terminal log file
        terminal_log_file = os.path.join(txt_output_dir, f"terminal_output_{timestamp}.txt")
        _terminal_log_file = open(terminal_log_file, 'w', encoding='utf-8')
        
        # Save original stdout and stderr
        _original_stdout = sys.stdout
        _original_stderr = sys.stderr
        
        # Create tee-like output redirector
        class TeeOutput:
            def __init__(self, terminal_output, file_output):
                self.terminal = terminal_output
                self.file = file_output
                
            def write(self, message):
                # Write to terminal
                self.terminal.write(message)
                self.terminal.flush()
                # Write to file with timestamp
                if message.strip():
                    timestamp_str = time.strftime("%H:%M:%S")
                    self.file.write(f"[{timestamp_str}] {message}")
                    self.file.flush()
                    
            def flush(self):
                self.terminal.flush()
                self.file.flush()
                
            def __getattr__(self, name):
                return getattr(self.terminal, name)
        
        # Redirect stdout and stderr
        sys.stdout = TeeOutput(_original_stdout, _terminal_log_file)
        sys.stderr = TeeOutput(_original_stderr, _terminal_log_file)
        
        print(f"[EARLY_LOG] 早期终端日志记录已启用: {terminal_log_file}")
        
    except Exception as e:
        print(f"[WARN] 早期日志设置失败: {e}")


def restore_early_logging():
    """Restore original stdout/stderr and close log file"""
    global _terminal_log_file, _original_stdout, _original_stderr
    
    try:
        if _original_stdout is not None:
            sys.stdout = _original_stdout
        if _original_stderr is not None:
            sys.stderr = _original_stderr
            
        if _terminal_log_file is not None:
            _terminal_log_file.flush()
            _terminal_log_file.close()
            print("[EARLY_LOG] 早期终端日志已关闭")
    except Exception as e:
        print(f"[WARN] 恢复早期日志失败: {e}")


def main():
    # Setup early logging before anything else
    setup_early_logging()
    
    try:
        parser = argparse.ArgumentParser(description="Modular navigation and mapping for real robot")
        parser.add_argument("--port", default="COM3", help="Serial port (default: COM4)")
        parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
        parser.add_argument("--size_m", type=float, default=2.8, help="Map size in meters (default: 2.8)")
        parser.add_argument("--res", type=float, default=0.01, help="Map resolution (default: 0.01)")
        parser.add_argument("--cycles", type=int, default=50, help="Max exploration cycles (default: 50)")
        parser.add_argument("--exit_xy", nargs=2, type=float, default=None, help="Manual exit coordinates (x y)")
        args = parser.parse_args()

        print(f"[INFO] Starting modular navigation exploration:")
        print(f"  - Port: {args.port}")
        print(f"  - Baud: {args.baud}")
        print(f"  - Map size: {args.size_m}m x {args.size_m}m")
        print(f"  - Resolution: {args.res:.3f}m/pixel")
        print(f"  - Max cycles: {args.cycles}")

        app = NavMappingApp(port=args.port, baud=args.baud, 
                          size_m=args.size_m, map_res=args.res)
        
        # [ADD] optional manual exit for bench test
        # This allows manual exit band if detector is not wired yet.
        if args.exit_xy is not None:
            ex, ey = args.exit_xy
            app.exit_band = (ex-0.08, ex+0.08, ey-0.08, ey+0.08)  # 16 cm band
            print(f"[INFO] Manual exit band set: ({ex:.3f}, {ey:.3f}) ± 8cm")
        
        app.run(max_cycles=args.cycles)
        
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted (Ctrl+C). Saving data before shutdown...")
    except SystemExit:
        print("\n[INFO] System exit requested. Saving data...")
    except Exception as e:
        print(f"[ERROR] Fatal error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Always attempt to save data and cleanup, regardless of how we exit
        print("[INFO] Performing final cleanup and data saving...")
        try:
            if app:
                app.close()  # This will call _force_save_all_data
            else:
                print("[WARN] App was not properly initialized, skipping data save")
        except Exception as cleanup_error:
            print(f"[ERROR] Cleanup failed: {cleanup_error}")
            
        try:
            if plt:
                plt.close('all')
        except Exception as plt_error:
            print(f"[WARN] Matplotlib cleanup failed: {plt_error}")
        
        # Restore early logging at the very end
        restore_early_logging()
            
        print("[INFO] Cleanup completed.")


if __name__ == "__main__":
    main()

