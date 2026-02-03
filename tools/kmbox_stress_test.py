#!/usr/bin/env python3
"""
KMBox Stress Test - Mimics real-world KMBox usage patterns
Tests humanization under various movement scenarios common in actual applications.

Based on patterns from:
- OceanTw/KMNet.java: Rapid back-and-forth movements
- uve192/KMBox.NET: Interpolated and Bezier movements  
- ZCban/kmboxNET: 10,000+ rapid small movements

Usage:
    python3 kmbox_stress_test.py /dev/tty.usbmodem2101
    
Keyboard Controls:
    1-7: Run specific test
    A:   Run all tests
    Q:   Quit
"""

import serial
import time
import sys
import math
import random
from enum import Enum

class TestResult(Enum):
    PASS = "✅ PASS"
    FAIL = "❌ FAIL"
    WARN = "⚠️  WARN"

class KMBoxTester:
    def __init__(self, port, baudrate=921600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.stats = {
            'commands_sent': 0,
            'responses_ok': 0,
            'responses_err': 0,
            'timeouts': 0
        }
        
    def connect(self):
        """Connect to KMBox bridge"""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(0.5)  # Let connection stabilize
            # Flush any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"✅ Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from KMBox bridge"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✅ Disconnected")
    
    def send_move(self, x, y, expect_response=True):
        """Send a mouse move command"""
        if not self.ser or not self.ser.is_open:
            return False
        
        try:
            cmd = f"km.move({x}, {y})\n"
            self.ser.write(cmd.encode('utf-8'))
            self.stats['commands_sent'] += 1
            
            if expect_response:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if response == "OK":
                    self.stats['responses_ok'] += 1
                    return True
                elif response.startswith("ERR"):
                    self.stats['responses_err'] += 1
                    return False
                else:
                    self.stats['timeouts'] += 1
                    return False
            return True
        except Exception as e:
            print(f"❌ Send error: {e}")
            self.stats['responses_err'] += 1
            return False
    
    def print_stats(self):
        """Print current statistics"""
        print("\n" + "="*60)
        print("📊 Test Statistics:")
        print(f"  Commands sent:    {self.stats['commands_sent']}")
        print(f"  Responses OK:     {self.stats['responses_ok']}")
        print(f"  Responses ERR:    {self.stats['responses_err']}")
        print(f"  Timeouts:         {self.stats['timeouts']}")
        success_rate = (self.stats['responses_ok'] / self.stats['commands_sent'] * 100) if self.stats['commands_sent'] > 0 else 0
        print(f"  Success rate:     {success_rate:.1f}%")
        print("="*60 + "\n")
    
    # ========================================================================
    # Test 1: Rapid Small Movements (Most Common Pattern)
    # ========================================================================
    def test_rapid_small_movements(self):
        """
        Test pattern from ZCban/kmboxNET demo:
        10,000 rapid small movements back and forth
        
        This is the MOST COMMON pattern in real-world usage!
        Tests humanization with rapid direction changes.
        """
        print("\n" + "="*60)
        print("🧪 TEST 1: Rapid Small Movements (10,000 iterations)")
        print("="*60)
        print("Pattern: move(0, 10) → move(0, -10) × 10,000")
        print("Expected behavior: Smooth, no glitches, minimal jitter")
        print("Testing humanization on rapid direction changes...")
        
        count = 10000
        start_time = time.time()
        errors = 0
        
        try:
            for i in range(count):
                if not self.send_move(0, 10, expect_response=False):
                    errors += 1
                if not self.send_move(0, -10, expect_response=False):
                    errors += 1
                
                # Progress indicator
                if (i + 1) % 1000 == 0:
                    elapsed = time.time() - start_time
                    rate = (i + 1) * 2 / elapsed
                    print(f"  Progress: {i+1}/{count} ({rate:.0f} cmds/sec)")
        
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return TestResult.WARN
        
        elapsed = time.time() - start_time
        total_commands = count * 2
        rate = total_commands / elapsed
        
        print(f"\n📊 Results:")
        print(f"  Total movements:  {total_commands:,}")
        print(f"  Time elapsed:     {elapsed:.2f}s")
        print(f"  Commands/sec:     {rate:.0f}")
        print(f"  Errors:           {errors}")
        
        # Success criteria: >500 commands/sec, <1% errors
        if rate > 500 and errors < total_commands * 0.01:
            print(f"\n{TestResult.PASS.value}: Fast and reliable")
            return TestResult.PASS
        elif errors > total_commands * 0.05:
            print(f"\n{TestResult.FAIL.value}: Too many errors ({errors/total_commands*100:.1f}%)")
            return TestResult.FAIL
        else:
            print(f"\n{TestResult.WARN.value}: Slower than expected or some errors")
            return TestResult.WARN
    
    # ========================================================================
    # Test 2: Horizontal Sweep Pattern
    # ========================================================================
    def test_horizontal_sweep(self):
        """
        Test smooth left-right sweeping motion
        Common in aim training and tracking scenarios
        """
        print("\n" + "="*60)
        print("🧪 TEST 2: Horizontal Sweep Pattern")
        print("="*60)
        print("Pattern: Smooth left-right sweeps")
        print("Expected behavior: Fluid motion, humanization should be minimal")
        
        sweep_distance = 400  # pixels
        step_size = 20
        sweeps = 10
        errors = 0
        
        try:
            for sweep in range(sweeps):
                # Sweep right
                for i in range(0, sweep_distance, step_size):
                    if not self.send_move(step_size, 0, expect_response=False):
                        errors += 1
                    time.sleep(0.005)  # 200Hz update rate
                
                time.sleep(0.05)
                
                # Sweep left
                for i in range(0, sweep_distance, step_size):
                    if not self.send_move(-step_size, 0, expect_response=False):
                        errors += 1
                    time.sleep(0.005)
                
                time.sleep(0.05)
                print(f"  Sweep {sweep + 1}/{sweeps} complete")
        
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return TestResult.WARN
        
        print(f"\n📊 Results:")
        print(f"  Total sweeps:     {sweeps}")
        print(f"  Errors:           {errors}")
        
        if errors == 0:
            print(f"\n{TestResult.PASS.value}: Smooth sweeps")
            return TestResult.PASS
        else:
            print(f"\n{TestResult.WARN.value}: Some errors occurred")
            return TestResult.WARN
    
    # ========================================================================
    # Test 3: Small Precise Movements (Humanization Focus)
    # ========================================================================
    def test_small_precise_movements(self):
        """
        Test very small movements (1-5px)
        This is where humanization should be MOST visible
        """
        print("\n" + "="*60)
        print("🧪 TEST 3: Small Precise Movements (Humanization Test)")
        print("="*60)
        print("Pattern: Small 1-5px movements in various directions")
        print("Expected behavior: Light jitter visible, looks human")
        
        movements = [
            (1, 0), (0, 1), (-1, 0), (0, -1),   # 1px
            (2, 0), (0, 2), (-2, 0), (0, -2),   # 2px
            (3, 2), (2, 3), (-3, -2), (-2, -3), # Diagonal small
            (5, 0), (0, 5), (-5, 0), (0, -5),   # 5px
        ]
        
        iterations = 50
        errors = 0
        
        try:
            for i in range(iterations):
                for dx, dy in movements:
                    if not self.send_move(dx, dy, expect_response=False):
                        errors += 1
                    time.sleep(0.01)  # 100Hz
                
                if (i + 1) % 10 == 0:
                    print(f"  Iteration {i+1}/{iterations}")
        
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return TestResult.WARN
        
        total_moves = iterations * len(movements)
        print(f"\n📊 Results:")
        print(f"  Total movements:  {total_moves}")
        print(f"  Errors:           {errors}")
        print(f"\n💡 Visual Check: Did small movements look human-like?")
        print(f"   (Light jitter is GOOD, straight lines are BAD)")
        
        if errors < total_moves * 0.05:
            print(f"\n{TestResult.PASS.value}: Small movements executed")
            return TestResult.PASS
        else:
            print(f"\n{TestResult.FAIL.value}: Too many errors")
            return TestResult.FAIL
    
    # ========================================================================
    # Test 4: Large Fast Flicks
    # ========================================================================
    def test_large_fast_flicks(self):
        """
        Test large rapid movements (flicks)
        Humanization should be MINIMAL here
        """
        print("\n" + "="*60)
        print("🧪 TEST 4: Large Fast Flicks")
        print("="*60)
        print("Pattern: Large quick movements (100-300px)")
        print("Expected behavior: Fast, minimal jitter, feels snappy")
        
        flicks = [
            (200, 0), (-200, 0),    # Horizontal
            (0, 150), (0, -150),    # Vertical
            (150, 150), (-150, -150), # Diagonal
            (100, -200), (-100, 200), # Mixed
        ]
        
        iterations = 20
        errors = 0
        
        try:
            for i in range(iterations):
                for dx, dy in flicks:
                    if not self.send_move(dx, dy, expect_response=False):
                        errors += 1
                    time.sleep(0.02)  # 50Hz for large movements
                
                if (i + 1) % 5 == 0:
                    print(f"  Iteration {i+1}/{iterations}")
        
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return TestResult.WARN
        
        total_flicks = iterations * len(flicks)
        print(f"\n📊 Results:")
        print(f"  Total flicks:     {total_flicks}")
        print(f"  Errors:           {errors}")
        print(f"\n💡 Visual Check: Did large movements feel snappy?")
        print(f"   (Minimal jitter is GOOD, too much smoothing is BAD)")
        
        if errors < total_flicks * 0.05:
            print(f"\n{TestResult.PASS.value}: Large movements executed")
            return TestResult.PASS
        else:
            print(f"\n{TestResult.FAIL.value}: Too many errors")
            return TestResult.FAIL
    
    # ========================================================================
    # Test 5: Circular Motion
    # ========================================================================
    def test_circular_motion(self):
        """
        Test smooth circular motion
        Tests humanization during continuous curved movement
        """
        print("\n" + "="*60)
        print("🧪 TEST 5: Circular Motion")
        print("="*60)
        print("Pattern: Smooth circles of various sizes")
        print("Expected behavior: Smooth curves, light humanization")
        
        radii = [50, 100, 150]  # pixels
        steps = 36  # 10 degree increments
        errors = 0
        
        try:
            for radius in radii:
                print(f"  Drawing circle with radius {radius}px...")
                for i in range(steps):
                    angle1 = (i * 2 * math.pi) / steps
                    angle2 = ((i + 1) * 2 * math.pi) / steps
                    
                    dx = int(radius * (math.cos(angle2) - math.cos(angle1)))
                    dy = int(radius * (math.sin(angle2) - math.sin(angle1)))
                    
                    if not self.send_move(dx, dy, expect_response=False):
                        errors += 1
                    time.sleep(0.01)  # 100Hz
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return TestResult.WARN
        
        total_moves = len(radii) * steps
        print(f"\n📊 Results:")
        print(f"  Circles drawn:    {len(radii)}")
        print(f"  Total movements:  {total_moves}")
        print(f"  Errors:           {errors}")
        
        if errors < total_moves * 0.05:
            print(f"\n{TestResult.PASS.value}: Circles drawn smoothly")
            return TestResult.PASS
        else:
            print(f"\n{TestResult.FAIL.value}: Too many errors")
            return TestResult.FAIL
    
    # ========================================================================
    # Test 6: Mixed Pattern (Real-world Simulation)
    # ========================================================================
    def test_mixed_pattern(self):
        """
        Test mixed pattern simulating real usage:
        Small adjustments + occasional large movements + tracking
        """
        print("\n" + "="*60)
        print("🧪 TEST 6: Mixed Pattern (Real-world Simulation)")
        print("="*60)
        print("Pattern: Realistic mix of movement types")
        print("Expected behavior: Natural-looking, adaptive humanization")
        
        duration = 30  # seconds
        start_time = time.time()
        move_count = 0
        errors = 0
        
        try:
            while time.time() - start_time < duration:
                # Random movement type
                move_type = random.choice(['small', 'medium', 'large', 'tracking'])
                
                if move_type == 'small':
                    # Small precise movement (high humanization)
                    dx = random.randint(-5, 5)
                    dy = random.randint(-5, 5)
                    if not self.send_move(dx, dy, expect_response=False):
                        errors += 1
                    move_count += 1
                    time.sleep(0.01)
                
                elif move_type == 'medium':
                    # Medium movement (moderate humanization)
                    dx = random.randint(-50, 50)
                    dy = random.randint(-50, 50)
                    if not self.send_move(dx, dy, expect_response=False):
                        errors += 1
                    move_count += 1
                    time.sleep(0.02)
                
                elif move_type == 'large':
                    # Large flick (minimal humanization)
                    dx = random.randint(-200, 200)
                    dy = random.randint(-200, 200)
                    if not self.send_move(dx, dy, expect_response=False):
                        errors += 1
                    move_count += 1
                    time.sleep(0.05)
                
                elif move_type == 'tracking':
                    # Smooth tracking (10-20 small movements)
                    track_steps = random.randint(10, 20)
                    dx_per_step = random.randint(-3, 3)
                    dy_per_step = random.randint(-3, 3)
                    for _ in range(track_steps):
                        if not self.send_move(dx_per_step, dy_per_step, expect_response=False):
                            errors += 1
                        move_count += 1
                        time.sleep(0.008)  # 125Hz tracking
                
                # Progress every 5 seconds
                elapsed = time.time() - start_time
                if int(elapsed) % 5 == 0 and elapsed > 0:
                    print(f"  Elapsed: {int(elapsed)}s, Moves: {move_count}, Errors: {errors}")
        
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return TestResult.WARN
        
        elapsed = time.time() - start_time
        rate = move_count / elapsed
        
        print(f"\n📊 Results:")
        print(f"  Duration:         {elapsed:.1f}s")
        print(f"  Total movements:  {move_count}")
        print(f"  Moves/second:     {rate:.0f}")
        print(f"  Errors:           {errors}")
        
        if errors < move_count * 0.05:
            print(f"\n{TestResult.PASS.value}: Mixed pattern executed well")
            return TestResult.PASS
        else:
            print(f"\n{TestResult.FAIL.value}: Too many errors")
            return TestResult.FAIL
    
    # ========================================================================
    # Test 7: Report Rate Verification
    # ========================================================================
    def test_report_rate(self):
        """
        Verify that physical mouse + synthetic = same report count
        This is the critical fix we made!
        """
        print("\n" + "="*60)
        print("🧪 TEST 7: Report Rate Verification")
        print("="*60)
        print("Testing that synthetic movements don't double HID reports")
        print("\n⚠️  MANUAL TEST:")
        print("  1. Open system mouse settings or use 'iosnoop' to monitor")
        print("  2. Move physical mouse slowly")
        print("  3. Note the report rate")
        print("  4. Keep moving physical mouse while test runs")
        print("  5. Verify report rate stays the SAME")
        print("\nSending 500 synthetic movements over 5 seconds...")
        
        count = 500
        duration = 5.0
        delay = duration / count
        errors = 0
        
        input("\nPress ENTER when ready to start (start moving physical mouse now)...")
        
        try:
            for i in range(count):
                if not self.send_move(1, 1, expect_response=False):
                    errors += 1
                time.sleep(delay)
                
                if (i + 1) % 100 == 0:
                    print(f"  Sent {i+1}/{count} synthetic movements...")
        
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
            return TestResult.WARN
        
        print(f"\n📊 Results:")
        print(f"  Movements sent:   {count}")
        print(f"  Errors:           {errors}")
        print(f"\n💡 Did the report rate STAY THE SAME?")
        
        response = input("Did report count stay the same? (y/n): ").strip().lower()
        if response == 'y':
            print(f"\n{TestResult.PASS.value}: Report rate unchanged (CORRECT!)")
            return TestResult.PASS
        else:
            print(f"\n{TestResult.FAIL.value}: Report rate doubled (BUG!)")
            return TestResult.FAIL
    
    # ========================================================================
    # Run All Tests
    # ========================================================================
    def run_all_tests(self):
        """Run all tests and summarize results"""
        print("\n" + "="*60)
        print("🚀 RUNNING ALL TESTS")
        print("="*60)
        
        tests = [
            ("Rapid Small Movements", self.test_rapid_small_movements),
            ("Horizontal Sweep", self.test_horizontal_sweep),
            ("Small Precise Movements", self.test_small_precise_movements),
            ("Large Fast Flicks", self.test_large_fast_flicks),
            ("Circular Motion", self.test_circular_motion),
            ("Mixed Pattern", self.test_mixed_pattern),
            ("Report Rate Verification", self.test_report_rate),
        ]
        
        results = {}
        
        for name, test_func in tests:
            result = test_func()
            results[name] = result
            time.sleep(1)  # Brief pause between tests
        
        # Summary
        print("\n" + "="*60)
        print("📊 TEST SUMMARY")
        print("="*60)
        
        passed = sum(1 for r in results.values() if r == TestResult.PASS)
        warned = sum(1 for r in results.values() if r == TestResult.WARN)
        failed = sum(1 for r in results.values() if r == TestResult.FAIL)
        
        for name, result in results.items():
            print(f"  {result.value}: {name}")
        
        print(f"\nTotal: {passed} passed, {warned} warnings, {failed} failed")
        
        self.print_stats()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 kmbox_stress_test.py /dev/tty.usbmodem2101")
        sys.exit(1)
    
    port = sys.argv[1]
    tester = KMBoxTester(port)
    
    if not tester.connect():
        sys.exit(1)
    
    print("\n" + "="*60)
    print("🎯 KMBox Humanization Stress Test")
    print("="*60)
    print("\nTest Menu:")
    print("  1: Rapid Small Movements (10,000 iterations)")
    print("  2: Horizontal Sweep Pattern")
    print("  3: Small Precise Movements (Humanization Focus)")
    print("  4: Large Fast Flicks")
    print("  5: Circular Motion")
    print("  6: Mixed Pattern (Real-world)")
    print("  7: Report Rate Verification")
    print("  A: Run All Tests")
    print("  Q: Quit")
    
    try:
        while True:
            choice = input("\nSelect test (1-7, A, Q): ").strip().upper()
            
            if choice == 'Q':
                break
            elif choice == '1':
                tester.test_rapid_small_movements()
            elif choice == '2':
                tester.test_horizontal_sweep()
            elif choice == '3':
                tester.test_small_precise_movements()
            elif choice == '4':
                tester.test_large_fast_flicks()
            elif choice == '5':
                tester.test_circular_motion()
            elif choice == '6':
                tester.test_mixed_pattern()
            elif choice == '7':
                tester.test_report_rate()
            elif choice == 'A':
                tester.run_all_tests()
            else:
                print("❌ Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    
    finally:
        tester.disconnect()
        tester.print_stats()
        print("👋 Goodbye!")


if __name__ == "__main__":
    main()
