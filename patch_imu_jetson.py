import re

with open('deploy/biped_ws/src/biped_driver/biped_driver/imu_node.py', 'r') as f:
    text = f.read()

# Replace the lgpio reset block with a platform-aware version
old_reset = """            # Hardware reset via RST pin — uses lgpio directly to avoid
            # "GPIO busy" errors from digitalio after unclean exits
            if self._reset_pin >= 0:
                try:
                    import lgpio
                    h = lgpio.gpiochip_open(4)  # RPi5 GPIO chip
                    try:"""

# Find the full reset block and replace it
old_block = """            # Hardware reset via RST pin — uses lgpio directly to avoid
            # "GPIO busy" errors from digitalio after unclean exits
            if self._reset_pin >= 0:
                try:
                    import lgpio
                    h = lgpio.gpiochip_open(4)  # RPi5 GPIO chip
                    try:
                        lgpio.gpio_free(h, self._reset_pin)
                    except Exception:
                        pass
                    lgpio.gpio_claim_output(h, self._reset_pin, 0)  # pull low
                    time.sleep(0.1)
                    lgpio.gpio_write(h, self._reset_pin, 1)  # release
                    time.sleep(1.0)  # BNO085 needs ~1s to boot
                    lgpio.gpio_free(h, self._reset_pin)
                    lgpio.gpiochip_close(h)
                    self.get_logger().info(f'BNO085 hardware reset via GPIO {self._reset_pin}')
                except Exception as e:
                    self.get_logger().warn(f'GPIO reset failed: {e} — continuing without reset')"""

new_block = """            # Hardware reset via RST pin — platform-aware (RPi5 lgpio / Jetson GPIO)
            if self._reset_pin >= 0:
                try:
                    try:
                        # Try Jetson.GPIO first
                        import Jetson.GPIO as GPIO
                        GPIO.setmode(GPIO.BOARD)
                        GPIO.setup(self._reset_pin, GPIO.OUT, initial=GPIO.LOW)
                        time.sleep(0.1)
                        GPIO.output(self._reset_pin, GPIO.HIGH)
                        time.sleep(1.0)
                        GPIO.cleanup(self._reset_pin)
                        self.get_logger().info(f'BNO085 hardware reset via Jetson.GPIO pin {self._reset_pin}')
                    except (ImportError, Exception):
                        # Fall back to lgpio (RPi5)
                        import lgpio
                        h = lgpio.gpiochip_open(4)  # RPi5 GPIO chip
                        try:
                            lgpio.gpio_free(h, self._reset_pin)
                        except Exception:
                            pass
                        lgpio.gpio_claim_output(h, self._reset_pin, 0)
                        time.sleep(0.1)
                        lgpio.gpio_write(h, self._reset_pin, 1)
                        time.sleep(1.0)
                        lgpio.gpio_free(h, self._reset_pin)
                        lgpio.gpiochip_close(h)
                        self.get_logger().info(f'BNO085 hardware reset via lgpio pin {self._reset_pin}')
                except Exception as e:
                    self.get_logger().warn(f'GPIO reset failed: {e} — continuing without reset')"""

text = text.replace(old_block, new_block)

# Update the I2C bus default to support Jetson (bus 7) vs RPi (bus 1)
text = text.replace("self.declare_parameter('i2c_bus', 1)",
                    "self.declare_parameter('i2c_bus', 7)  # Jetson Orin Nano: bus 7 (pins 3,5), RPi5: bus 1")

# Update the reset_pin default to use BOARD pin 7 (works on both platforms)
text = text.replace("self.declare_parameter('reset_pin', 4)  # GPIO pin for BNO085 RST (-1 = not connected)",
                    "self.declare_parameter('reset_pin', 7)  # BOARD pin 7 for BNO085 RST (-1 = not connected)")

# Update the docstring
text = text.replace("Hardware: BNO085 on I2C bus 1, address 0x4B, RST on GPIO 4.\n  RPi 5 GPIO 2 (SDA) / GPIO 3 (SCL), 400kHz.",
                    "Hardware: BNO085 on I2C.\n  Jetson Orin Nano: I2C bus 7, Pin 3 (SDA) / Pin 5 (SCL), RST on Pin 7.\n  RPi 5: I2C bus 1, Pin 3 (SDA) / Pin 5 (SCL), RST on Pin 7 (GPIO4).")

with open('deploy/biped_ws/src/biped_driver/biped_driver/imu_node.py', 'w') as f:
    f.write(text)

print("patched")
