#!/usr/bin/env python3
"""
VR Monitor - Independent VR control information monitoring script
Can call XLeVR's VR functionality from other folders and read/print VR control information
"""

import os
import sys
import asyncio
import json
import logging
import threading
import http.server
import ssl
import socket
from pathlib import Path
from typing import Optional
from collections import deque

# Set the absolute path to the xlevr folder
XLEVR_PATH = "/home/that/lerobot/examples/xlerobot/XLeVR"

def setup_xlevr_environment():
    """Setup xlevr environment"""
    # Add xlevr path to Python path
    if XLEVR_PATH not in sys.path:
        sys.path.insert(0, XLEVR_PATH)
    
    # Set working directory
    os.chdir(XLEVR_PATH)
    
    # Set environment variables
    os.environ['PYTHONPATH'] = f"{XLEVR_PATH}:{os.environ.get('PYTHONPATH', '')}"

def get_local_ip():
    """Get the local IP address of this machine."""
    try:
        # Connect to a remote address to determine the local IP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # Fallback: get hostname IP
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            # Final fallback
            return "localhost"

def import_xlevr_modules():
    """Import xlevr modules"""
    try:
        from xlevr.config import XLeVRConfig
        from xlevr.inputs.vr_ws_server import VRWebSocketServer
        from xlevr.inputs.base import ControlGoal, ControlMode
        return XLeVRConfig, VRWebSocketServer, ControlGoal, ControlMode
    except ImportError as e:
        print(f"Error importing xlevr modules: {e}")
        print(f"Make sure XLEVR_PATH is correct: {XLEVR_PATH}")
        return None, None, None, None

class SimpleAPIHandler(http.server.BaseHTTPRequestHandler):
    """Simplified HTTP request handler, only provides basic web services"""
    
    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass
    
    def do_OPTIONS(self):
        """Handle preflight CORS requests."""
        self.send_response(200)
        self.end_headers()
    
    def log_message(self, format, *args):
        """Override to reduce HTTP request logging noise."""
        pass  # Disable default HTTP logging
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/' or self.path == '/index.html':
            # Serve main page from web-ui directory
            self.serve_file('web-ui/index.html', 'text/html')
        elif self.path.endswith('.css'):
            # Serve CSS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'text/css')
        elif self.path.endswith('.js'):
            # Serve JS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'application/javascript')
        elif self.path.endswith('.ico'):
            self.serve_file(self.path[1:], 'image/x-icon')
        elif self.path.endswith(('.jpg', '.jpeg', '.png', '.gif')):
            # Serve image files from web-ui directory
            content_type = 'image/jpeg' if self.path.endswith(('.jpg', '.jpeg')) else 'image/png' if self.path.endswith('.png') else 'image/gif'
            self.serve_file(f'web-ui{self.path}', content_type)
        else:
            self.send_error(404, "Not found")
    
    def serve_file(self, filename, content_type):
        """Serve a file with the given content type."""
        try:
            # Get the web root path from the server
            web_root = getattr(self.server, 'web_root_path', XLEVR_PATH)
            file_path = os.path.join(web_root, filename)
            
            if os.path.exists(file_path):
                with open(file_path, 'rb') as f:
                    content = f.read()
                
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_error(404, f"File not found: {filename}")
        except Exception as e:
            print(f"Error serving file {filename}: {e}")
            self.send_error(500, "Internal server error")

class SimpleHTTPSServer:
    """Simplified HTTPS server for providing web interface"""
    
    def __init__(self, config):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.web_root_path = XLEVR_PATH
    
    async def start(self):
        """Start the HTTPS server."""
        try:
            # Create server
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), SimpleAPIHandler)
            
            # Set web root path for file serving
            self.httpd.web_root_path = self.web_root_path
            
            # Setup SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain('cert.pem', 'key.pem')
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)
            
            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            print(f"🌐 HTTPS server started on {self.config.host_ip}:{self.config.https_port}")
            
        except Exception as e:
            print(f"❌ Failed to start HTTPS server: {e}")
            raise
    
    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            print("🌐 HTTPS server stopped")

class VRMonitor:
    """VR control information monitor"""
    
    def __init__(self, *, print_goal: bool = False):
        self.config = None
        self.vr_server = None
        self.https_server = None
        self.is_running = False
        self.print_goal = print_goal
        self._goal_lock = threading.Lock()  # Add thread lock

        # per-arm goal queues
        self._goal_queues = {
            "left": deque(),
            "right": deque(),
            "headset": deque(),
        }
    
    def initialize(self):
        """Initialize VR monitor"""
        print("🔧 Initializing XLeVR Monitor...")
        
        # Setup environment
        setup_xlevr_environment()
        
        # Import modules
        XLeVRConfig, VRWebSocketServer, ControlGoal, ControlMode = import_xlevr_modules()
        if XLeVRConfig is None:
            print("❌ Failed to import xlevr modules")
            return False
        
        # Create configuration
        self.config = XLeVRConfig()
        self.config.enable_vr = True
        self.config.enable_keyboard = False
        self.config.enable_https = True  # Enable HTTPS server, VR requires web interface
        
        # Create command queue
        self.command_queue = asyncio.Queue()
        
        # Create VR server (print-only mode)
        try:
            self.vr_server = VRWebSocketServer(
                command_queue=self.command_queue,
                config=self.config,
                print_only=False  # Changed to False to send data to queue
            )
        except Exception as e:
            print(f"❌ Failed to create VR WebSocket server: {e}")
            return False
        
        # Create HTTPS server
        try:
            self.https_server = SimpleHTTPSServer(self.config)
        except Exception as e:
            print(f"❌ Failed to create HTTPS server: {e}")
            return False
        
        print("✅ XLeVR Monitor initialized successfully")
        
        return True
    
    async def start_monitoring(self):
        """Start monitoring VR control information"""
        print("🚀 Starting VR Monitor...")
        
        if not self.initialize():
            print("❌ Failed to initialize VR monitor")
            return
        
        try:
            # Start HTTPS server
            await self.https_server.start()
            
            # Start VR server
            await self.vr_server.start()
            
            self.is_running = True
            print("✅ VR Monitor is now running")
            
            # Display connection information
            host_display = get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
            print(f"📱 Open your VR headset browser and navigate to:")
            print(f"   https://{host_display}:{self.config.https_port}")
            print("🎯 Press Ctrl+C to stop monitoring")
            print()
            
            # Monitor command queue
            await self.monitor_commands()
            
        except KeyboardInterrupt:
            print("\n⏹️  Stopping VR monitor...")
        except Exception as e:
            print(f"❌ Error in VR monitor: {e}")
            import traceback
            print(f"Traceback: {traceback.format_exc()}")
        finally:
            await self.stop_monitoring()
    
    async def monitor_commands(self):
        """Monitor commands from VR controllers"""
        print("📊 Monitoring VR control commands...")
        
        while self.is_running:
            try:
                # Wait for command with 1-second timeout
                goal = await asyncio.wait_for(self.command_queue.get(), timeout=1.0)
                if self.print_goal:
                    self.print_control_goal(goal)
                
                # Save goal by arm type
                with self._goal_lock:

                    # Put goal into the per-arm queue
                    if goal.arm in self._goal_queues:
                        self._goal_queues[goal.arm].append(goal)
                
            except asyncio.TimeoutError:
                # Timeout, continue loop
                continue
            except Exception as e:
                print(f"❌ Error processing command: {e}")
                import traceback
                print(f"Traceback: {traceback.format_exc()}")
    
    def pop_ordered_goals_for_arm(self, arm: str):
        """
        Drain all pending goals for the given arm and return a tuple:
            (reset_goal, motion_goal)

        - If any RESET goals are present, the *last* one is returned as reset_goal.
        - For motion, we return the *last* POSITION_CONTROL or IDLE goal.
        - If no goals are pending, both are None.

        The idea: you call this once per control cycle, then:
            1) If reset_goal is not None -> handle RESET (calibration)
            2) If motion_goal is not None -> handle motion (POSITION_CONTROL / IDLE)
        """

        reset_goal = None
        motion_goal = None

        if arm not in self._goal_queues:
            return None, None

        with self._goal_lock:
            q = self._goal_queues[arm]
            if not q:
                return None, None

            while q:
                goal = q.popleft()
                mode_val = getattr(goal.mode, "value", goal.mode)
                if mode_val == "reset":  # ControlMode.RESET.value
                    # keep the last RESET in this batch
                    reset_goal = goal
                elif mode_val in ("position", "idle"):
                    # keep only the last motion-related goal
                    motion_goal = goal

        return reset_goal, motion_goal
    
    def reset_goal_queues(self, arm: str | None = None) -> None:
        """
        Clear stored goals.

        arm:
          - None     -> clear all queues ("left", "right", "headset")
          - "left"   -> clear only left arm queue
          - "right"  -> clear only right arm queue
          - "headset"-> clear only headset queue
        """
        if arm is None:
            for q in self._goal_queues.values():
                q.clear()
        else:
            if arm in self._goal_queues:
                self._goal_queues[arm].clear()

    
    def print_control_goal(self, goal):
        """Print control goal information"""
        print(f"\n🎮 Control Goal Received:")
        print(f"   Timestamp: {asyncio.get_event_loop().time():.3f}")
        print(f"   Arm: {goal.arm}")
        print(f"   Mode: {goal.mode}")
        
        if goal.relative_position is not None:
            print(f"   Relative Position: {goal.relative_position}")
        if goal.relative_rotvec is not None:
            print(f"   Relative Rotvec: {goal.relative_rotvec}")
        if goal.vr_ctrl_rotation is not None:
            print(f"   VR Frame: {goal.vr_ctrl_rotation}")
        if goal.trigger is not None:
            print(f"   Trigger: {goal.trigger}")
        if goal.thumbstick is not None:
            print(f"   Thumbstick: x={goal.thumbstick['x']:.2f}, y={goal.thumbstick['y']:.2f}")
        if goal.buttons is not None:
            print(f"   Buttons: {goal.buttons}")
        if goal.metadata:
            print(f"   Metadata: {goal.metadata}")
        
        print("-" * 30)
    
    async def stop_monitoring(self):
        """Stop monitoring"""
        self.is_running = False
        
        if self.vr_server:
            await self.vr_server.stop()
        
        if self.https_server:
            await self.https_server.stop()
        
        print("✅ VR Monitor stopped")

def main():
    """Main function"""
    print("🎮 XLeVR Monitor - XLeVR VR Control Information Monitor")
    print("=" * 60)
    
    # Check XLeVR path
    if not os.path.exists(XLEVR_PATH):
        print(f"❌ XLeVR path does not exist: {XLEVR_PATH}")
        print("Please update XLEVR_PATH in the script")
        return
    
    # Create monitor
    monitor = VRMonitor(print_goal=False)
    
    # Run monitoring
    try:
        asyncio.run(monitor.start_monitoring())
    except KeyboardInterrupt:
        print("\n👋 XLeVR Monitor stopped by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

if __name__ == "__main__":
    main() 
