import time

try:
    from typing import Union
except ImportError:
    # CircuitPython does not provide the typing module
    pass

import digitalio
import board
import busio
import supervisor

import adafruit_vl53l0x

import capablerobot_usbhub
import capablerobot_tlc59116

boot_time = time.monotonic()

led1 = digitalio.DigitalInOut(board.LED1)
led1.switch_to_output(value=False)

led2 = digitalio.DigitalInOut(board.LED2)
led2.switch_to_output(value=True)

led3 = digitalio.DigitalInOut(board.LED3)
led3.switch_to_output(value=True)

BEAT = 0.05
LED_BRIGHT = 80
COLOR_YELLOW = (LED_BRIGHT - 20, LED_BRIGHT, 0)

DISTANCE_READ_INTERVAL_S = 1
MAX_SITTING_TIME_S = 60 * 60


def stdout(*args):
    if supervisor.runtime.serial_connected:
        print(*args)


stdout("... booted ...")

i2c1 = busio.I2C(board.SCL, board.SDA)
i2c2 = busio.I2C(board.SCL2, board.SDA2)


def initialize_distance_sensor():
    global distance_sensor
    global distance_last_time
    global distance_failures
    global distance_failure_blink_state
    global desk_state

    try:
        distance_sensor = adafruit_vl53l0x.VL53L0X(i2c2, io_timeout_s=1)
        distance_sensor.start_continuous()
        distance_sensor.measurement_timing_budget = 200000
        distance_last_time = 0
        distance_failures = 0
        distance_failure_blink_state = True
    except:
        stdout("Failed to initialize distance sensor")
        desk_state = DESK_UNKNOWN


DESK_RAISED = "raised"
DESK_LOWERED = "lowered"
DESK_UNKNOWN = "unknown"

distance_sensor = None  # type: Union[adafruit_vl53l0x.VL53L0X, None]
distance_mm = 0
distance_last_time = 0
distance_failures = 0
distance_failure_blink_state = True

# TODO(Noah): Expose desk_state by writing it to the EEPROM
desk_state = DESK_UNKNOWN
desk_state_became_unknown_time = 0
desk_raised_last_time = time.monotonic()
desk_blink_state = False
desk_blink_state_loop_cycles = 0
desk_blinking_led_positions = []
initialize_distance_sensor()

stdout("... configuring hub ...")
usb = capablerobot_usbhub.USBHub(i2c1, i2c2)

stdout("... configuring leds ...")
BRIGHT = 20
led_pwr = capablerobot_tlc59116.TLC59116(i2c1, 0x61, pwm=BRIGHT)
led_data = capablerobot_tlc59116.TLC59116(i2c1, 0x62, pwm=BRIGHT)

stdout()
stdout("Unit SKU : %s" % usb.unit_sku)
stdout("Revision : %s" % usb.unit_revision)
stdout("  Serial : %s" % usb.unit_serial)
stdout()

upstream_state = "reset"
upstream_last_time = boot_time


def reset():
    global upstream_state, upstream_last_time

    usb.reset()
    usb.configure()
    usb.set_mcp_config()

    ## Light the host data LED orange to show the reset is occuring
    led_data.rgb(0, (LED_BRIGHT, int(LED_BRIGHT / 2), 0), update=True)
    time.sleep(0.5)

    ## Reset the upstream timeout to ensure that the next
    ## reset can only occurs after the specified timeout
    upstream_state = "reset"
    upstream_last_time = time.monotonic()

    usb.set_last_poll_time(time.monotonic())


while True:
    if time.monotonic() > distance_last_time + DISTANCE_READ_INTERVAL_S:
        try:
            prev_distance = distance_mm
            prev_desk_state = desk_state
            distance_mm = int(round(distance_sensor.range))
            distance_last_time = time.monotonic()
            if 80 <= distance_mm <= 95:
                desk_state = DESK_LOWERED
                desk_state_became_unknown_time = 0
            elif 160 <= distance_mm <= 200:
                desk_state = DESK_RAISED
                desk_state_became_unknown_time = 0
                desk_raised_last_time = time.monotonic()
            else:
                # If the state has been unknown for the past 30 seconds, record it as unknown
                # The 30-second requirement prevents the state becoming unknown while the desk is being moved
                if desk_state_became_unknown_time == 0:
                    desk_state_became_unknown_time = time.monotonic()
                elif time.monotonic() - desk_state_became_unknown_time > 30:
                    desk_state = DESK_UNKNOWN

            if prev_desk_state is not desk_state:
                stdout("Desk state: {}".format(desk_state))

            if desk_state == DESK_UNKNOWN and distance_mm != prev_distance:
                stdout("Distance: {}mm".format(distance_mm))
        except:
            distance_failures += 1
            if distance_failures == 3 or distance_failures % 10 == 0:
                stdout("Re-initializing distance sensor")
                initialize_distance_sensor()

    time.sleep(usb.config["loop_delay"])

    ## Look for data from the Host computer via special USB4715 registers
    try:
        usb.poll_for_host_comms()
    except RuntimeError:
        stdout(time.monotonic(), "--- RESET due to loop delay ---")
        reset()
        continue

    ## Internal heartbeat LED
    led3.value = not led3.value

    if usb.config["external_heartbeat"]:
        if led3.value:
            led_data.aux(0, update=False)
        else:
            led_data.aux(250, update=False)
    elif led3.value:
        ## If the configuration was changed while the LED is on,
        ## we still need to turn it off when the next update happens.
        led_data.aux(0, update=False)

    data_state = usb.data_state()

    if data_state is None:
        continue

    # TODO(Noah): Improve back-and-forth blinking performance
    desk_blinking_led_positions = []

    if (
        desk_state == DESK_LOWERED
        and time.monotonic() > desk_raised_last_time + MAX_SITTING_TIME_S
    ):
        remind_user_to_stand = True
        desk_blink_state_loop_cycles = desk_blink_state_loop_cycles + 1
        if desk_blink_state_loop_cycles >= 5:
            desk_blink_state = not desk_blink_state
            desk_blink_state_loop_cycles = 0
    else:
        remind_user_to_stand = False
        desk_blink_state_loop_cycles = 0
        desk_blink_state = False

    ## Set the data LEDs based on the detected per-port speeds
    for idx, speed in enumerate(usb.speeds):
        color = (0, 0, 0)

        if idx > 0 and data_state[idx - 1] == False:
            ## If port data is disabled, light the LED orange
            color = (LED_BRIGHT, int(LED_BRIGHT / 2), 0)
        elif speed == 0b01:
            color = (0, 0, LED_BRIGHT)
        elif speed == 0b10:
            color = (0, LED_BRIGHT, 0)
        elif speed == 0b11:
            color = (LED_BRIGHT, LED_BRIGHT, LED_BRIGHT)
        elif remind_user_to_stand:
            desk_blinking_led_positions.append(idx)
            if desk_blink_state:
                color = COLOR_YELLOW

        if idx == 0:
            if speed == 0b00:
                ## If the upstream port is disconnected, light the
                ## LED red and record that the link is down
                color = (LED_BRIGHT, 0, 0)
                upstream_state = "down"
            else:
                upstream_last_time = time.monotonic()
                upstream_state = "up"

            if distance_failures > 0:
                if distance_failures % 10 == 0:
                    distance_failure_blink_state = not distance_failure_blink_state

                if distance_failure_blink_state:
                    color = COLOR_YELLOW
            elif desk_state == DESK_UNKNOWN:
                # Set the LED color to blue
                color = (0, 0, LED_BRIGHT)

        led_data.rgb(idx, color, update=False)

    led_data.update()

    power_state = usb.power_state()

    ## Set the power LEDs based on the measured per-port current
    ucs_currents = usb.power_measure(raw=True, rescale=2)
    ucs_status = usb.power_errors()

    for idx, current in enumerate(ucs_currents):

        ## With rescaling, raw reading may be above 255 (max value for LED), so cap it
        if current > 255:
            current == 255

        if idx == 0:
            color = (0, 0, int(current / 4))
        else:
            if idx in desk_blinking_led_positions:
                if desk_blink_state:
                    color = (0, 0, 0)
                else:
                    color = COLOR_YELLOW
            elif power_state[idx - 1] == False:
                ## If port power is disabled, light the LED orange
                color = (LED_BRIGHT, int(LED_BRIGHT / 2), 0)
            elif "ERR" in ucs_status[idx - 1] or "ALERT" in ucs_status[idx - 1]:
                ## UCS is reporting an alert, light the LED red
                color = (LED_BRIGHT, 0, 0)
            elif "CC" in ucs_status[idx - 1]:
                ## UCS is reporting constant current mode, light the LED green
                color = (0, LED_BRIGHT, 0)
            else:
                ## Otherwise, light blue with intensity based on measured power draw
                color = (0, 0, current)

        led_pwr.rgb(idx, color, update=False)

    led_pwr.update()

    if usb.config["reset_on_link_loss"] and upstream_state == "down":
        if time.monotonic() - upstream_last_time > usb.config["link_loss_delay"]:
            stdout("--- RESET DUE TO LINK LOSS ---")
            reset()
