import socketserver
import Jetson.GPIO as GPIO
import time
from threading import Thread

PIN_PWM1 = 32
PIN_AUX1 = 38
PIN_PWM2 = 33
PIN_AUX2 = 37

def main():
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(PIN_PWM1, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(PIN_PWM2, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(PIN_AUX1, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(PIN_AUX2, GPIO.OUT, initial=GPIO.LOW)

  pwm1 = GPIO.PWM(PIN_PWM1, 100)
  pwm2 = GPIO.PWM(PIN_PWM2, 100)
  pwm1.start(0)
  pwm2.start(0)

  def set_pwm(value, ch):
    # find pins from channel
    pwm, aux_pin = (None, None)
    if ch == 1:
      pwm, aux_pin = (pwm1, PIN_AUX1)
    elif ch == 2:
      pwm, aux_pin = (pwm2, PIN_AUX2)
    else:
      print("invalid motor channel")
      return
    # compute duty cycle out and auxillary out
    value = int(value)
    duty, aux_out = (None, None)
    if value < 0:
      duty = 100 + value
      aux_out = 1
    else:
      duty = value
      aux_out = 0
    # set
    pwm.ChangeDutyCycle(duty)
    GPIO.output(aux_pin, aux_out)

  set_pwm(0, 1)
  set_pwm(0, 2)

  last_packet_time = time.clock_gettime(time.CLOCK_MONOTONIC)

  def manager_thread():
    return
    while True:
      if (last_packet_time + 1.0) < time.clock_gettime(time.CLOCK_MONOTONIC):
        print("reset")
        set_pwm(0, 1)
        set_pwm(0, 2)
      time.sleep(1)

  Thread(target=manager_thread).start()

  class PacketHandler(socketserver.DatagramRequestHandler):
    def handle(self):
      try:
        (data, socket) = self.request
        print(data)
        spl = data.decode("utf-8").strip().split(",")
        if spl[0] == "pwm":
          set_pwm(int(spl[1]), 1)
          set_pwm(int(spl[2]), 2)
          last_packet_time = time.clock_gettime(time.CLOCK_MONOTONIC)
          print(f"PWM set: {spl[1]} {spl[2]}")
      except Exception as err:
        print("Error while handling packet: ", err)

  try:
    with socketserver.UDPServer(("0.0.0.0", 13456), PacketHandler) as server:
      server.serve_forever()
  except KeyboardInterrupt:
    GPIO.cleanup()