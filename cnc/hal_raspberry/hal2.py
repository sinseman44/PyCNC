import time

from cnc.hal_raspberry import rpgpio
from cnc.pulses import *
from cnc.config import *

US_IN_SECONDS = 1000000

gpio = rpgpio.GPIO()
dma = rpgpio.DMAGPIO()
pwm = rpgpio.DMAPWM()
watchdog = rpgpio.DMAWatchdog()

class stepper:
    ''' double H bridge stepper motor driver '''

    STEPPER_ENABLED = 0
    STEPPER_DISABLED = 1

    STATE_DIR_DIRECT = 0
    STATE_DIR_INV = 1
    
    AXE_X = 0
    AXE_Y = 1
    AXE_Z = 2

    MODE_FULL = 0
    MODE_HALF = 1

    HALF_STEP_SEQ = [[1,0,0,0], # Phase A
                     [1,0,1,0], # Phase AB
                     [0,0,1,0], # Phase B
                     [0,1,1,0], # Phase A'B
                     [0,1,0,0], # Phase A'
                     [0,1,0,1], # Phase A'B'
                     [0,0,0,1], # Phase B'
                     [1,0,0,1]] # Phase AB'

    FULL_STEP_SEQ = [[1,0,0,0], # Phase A
                     [0,0,1,0], # Phase B
                     [0,1,0,0], # Phase A'
                     [0,0,0,1]] # Phase B'

    def __init__(self, axe=AXE_X, pins=[]):
        ''' contructor '''
        self.state = self.STEPPER_DISABLED 
        self.dir = self.STATE_DIR_DIRECT
        self.axe = axe
        self.seq = self.FULL_STEP_SEQ[:]
        self.pins = pins
        self.step_number = 0
        self.current_seq_num = 0 
        self.number_of_steps = 0

    def init(self):
        ''' init gpios '''
        self.state = self.STEPPER_ENABLED 
        for pin in self.pins:
            logging.debug("set pin {} out".format(pin))
            gpio.init(pin, rpgpio.GPIO.MODE_OUTPUT)

    def set_steps_number(self, number=0):
        ''' set the number of steps of the motor '''
        self.number_of_steps = number

    def set_mode(self, mode):
        ''' set operationnal mode '''
        if mode > self.MODE_HALF:
            raise ValueError
        elif mode == self.MODE_FULL:
            self.seq = self.FULL_STEP_SEQ[:]
            if self.dir == self.STATE_DIR_INV:
                self.seq.reverse()
        elif mode == self.MODE_HALF:
            self.seq = self.HALF_STEP_SEQ[:]
            if self.dir == self.STATE_DIR_INV:
                self.seq.reverse()

    def set_dir(self, direction):
        ''' set direction '''
        if direction > self.STATE_DIR_INV:
            raise ValueError
        elif direction == self.STATE_DIR_INV and self.dir == self.STATE_DIR_DIRECT:
            #logging.debug("SET DIR INV")
            self.dir = direction
            #logging.debug("BEFORE: {}".format(self.seq))
            self.seq.reverse()
            #logging.debug("AFTER: {}".format(self.seq))
            if self.current_seq_num > 0:
                self.current_seq_num = len(self.seq) - 1 - self.current_seq_num
        elif direction == self.STATE_DIR_DIRECT and self.dir == self.STATE_DIR_INV:
            #logging.debug("SET DIR DIRECT")
            self.dir = direction
            #logging.debug("BEFORE: {}".format(self.seq))
            self.seq.reverse()
            #logging.debug("AFTER: {}".format(self.seq))
            if self.current_seq_num > 0:
                self.current_seq_num = len(self.seq) - 1 - self.current_seq_num
        else:
            logging.warning("direction already set")

    def inc_seq_num(self):
        ''' increment current sequence number '''
        if self.current_seq_num >= len(self.seq) - 1:
            self.current_seq_num = 0
        else:
            self.current_seq_num += 1

    def dec_seq_num(self):
        ''' decrement current sequence number '''
        if self.current_seq_num == 0:
            self.current_seq_num = len(self.seq) - 1
        else:
            self.current_seq_num -= 1

    def get_current_seq(self):
        ''' get current sequence '''
        return self.seq[self.current_seq_num]

    def disable(self):
        ''' disable stepper '''
        self.state = self.STEPPER_DISABLED
        for pin in self.pins:
            gpio.clear(pin)

    def enable(self):
        ''' enable stepper '''
        self.state = self.STEPPER_ENABLED

    def debug(self):
        ''' debug mode '''
        logging.debug("state = {} - \
dir = {} - \
axe = {} - \
seq = {} - \
pins = {} - \
step_number = {} - \
current_seq_num = {} - \
number_of_steps = {}".format(self.state, 
                             self.dir,
                             self.axe,
                             self.seq,
                             self.pins,
                             self.step_number,
                             self.current_seq_num,
                             self.number_of_steps))


stepper_x = stepper(axe=stepper.AXE_X, pins=STEPPER_STEP_PINS_X)
stepper_y = stepper(axe=stepper.AXE_Y, pins=STEPPER_STEP_PINS_Y)

def init():
    """ Initialize GPIO pins and machine itself.
    """
    logging.info("initialisation of gpios ...")
    # Init X stepper
    stepper_x.init()
    stepper_x.set_mode(mode = stepper.MODE_HALF)
    stepper_x.set_steps_number(STEPPER_STEPS_PER_REV_X)
    if STEPPER_INVERTED_X:
        stepper_x.set_dir(stepper.STATE_DIR_INV)
    stepper_x.enable()
    stepper_x.debug()
    # Init Y stepper
    stepper_y.init()
    stepper_y.set_mode(mode = stepper.MODE_HALF)
    stepper_y.set_steps_number(STEPPER_STEPS_PER_REV_Y)
    if STEPPER_INVERTED_Y:
        stepper_y.set_dir(stepper.STATE_DIR_INV)
    stepper_y.enable()
    stepper_y.debug()
    # Init pen pin
    gpio.init(PEN_PIN, rpgpio.GPIO.MODE_OUTPUT)
    gpio.clear(PEN_PIN)
    # Watchdog start
    watchdog.start()


def spindle_control(percent):
    """ Spindle control implementation.
    :param percent: spindle speed in percent 0..100. If 0, stop the spindle.
    """
    logging.debug("spindle control not implemented")

def fan_control(on_off):
    """
    Cooling fan control.
    :param on_off: boolean value if fan is enabled.
    """
    logging.debug("fan control not implemented")

def pen_control(up_down):
    """
    Pen control.
    :param on_off: boolean value if pen is up or down.
    """
    if up_down:
        logging.info("Pen is up ...")
        pwm.add_pin(PEN_PIN, 5)
        time.sleep(0.25)
        pwm.add_pin(PEN_PIN, 0)
    else:
        logging.info("Pen is down ...")
        pwm.add_pin(PEN_PIN, 12.5)
        time.sleep(0.25)
        pwm.add_pin(PEN_PIN, 0)

def extruder_heater_control(percent):
    """ Extruder heater control.
    :param percent: heater power in percent 0..100. 0 turns heater off.
    """
    logging.debug("extruder heater control not implemented")

def bed_heater_control(percent):
    """ Hot bed heater control.
    :param percent: heater power in percent 0..100. 0 turns heater off.
    """
    logging.debug("bed heater control not implemented")

def get_extruder_temperature():
    """ Measure extruder temperature.
    :return: temperature in Celsius.
    """
    logging.debug("extruder temperature not implemented")

def get_bed_temperature():
    """ Measure bed temperature.
    :return: temperature in Celsius.
    """
    logging.debug("bed temperature not implemented")

def disable_steppers():
    """ Disable all steppers until any movement occurs.
    """
    stepper_x.disable()
    stepper_y.disable()

def calibrate(x, y, z):
    """ Move head to home position till end stop switch will be triggered.
    Do not return till all procedures are completed.
    :param x: boolean, True to calibrate X axis.
    :param y: boolean, True to calibrate Y axis.
    :param z: boolean, True to calibrate Z axis.
    :return: boolean, True if all specified end stops were triggered.
    """
    logging.debug("calibrate not implemented")

def move(generator):
    """ Move head to specified position
    :param generator: PulseGenerator object.
    """
    # Fill buffer right before currently running(previous sequence) dma
    # this mode implements kind of round buffer, but protects if CPU is not
    # powerful enough to calculate buffer in advance, faster then machine
    # moving. In this case machine would safely paused between commands until
    # calculation is done.

    # G1 X50 F100 : permet de faire un mouvement de 50mm selon l'axe X lent (100mm/min soit 1.66mm/s).

    # 4 control blocks per 32 bytes
    bytes_per_iter = 4 * dma.control_block_size()
    # prepare and run dma
    dma.clear() # should just clear current address, but not stop current DMA
    prevx = 0
    prevy = 0
    is_ran = False
    instant = INSTANT_RUN
    st = time.time()
    current_cb = 0
    k = 0
    k0 = 0
    idx = 0
    for direction, tx, ty, tz, te in generator:
        if current_cb is not None:
            #logging.debug("{} + {} => result = {}".format(dma.current_address(), bytes_per_iter, dma.current_address() + bytes_per_iter))
            while dma.current_address() + bytes_per_iter >= current_cb:
                time.sleep(0.001)
                current_cb = dma.current_control_block()
                #logging.debug("current control block : {}".format(current_cb))
                if current_cb is None:
                    k0 = k
                    st = time.time()
                    break  # previous dma sequence has stopped
#        logging.debug("[{}] direction: {} - tx: {} - ty: {} - tz: {} - te: {}".format(idx, direction, tx, ty, tz, te))
        if direction:  # set up directions
            logging.debug("[{}] direction: {} - tx: {} - ty: {} - tz: {} - te: {}".format(idx, direction, tx, ty, tz, te))
            if tx > 0:
                logging.debug("TX Direct")
                stepper_x.set_dir(stepper.STATE_DIR_DIRECT)
            elif tx < 0:
                logging.debug("TX Inverse")
                stepper_x.set_dir(stepper.STATE_DIR_INV)
            if ty > 0:
                logging.debug("TY Direct")
                stepper_y.set_dir(stepper.STATE_DIR_DIRECT)
            elif ty < 0:
                logging.debug("TY Inverse")
                stepper_y.set_dir(stepper.STATE_DIR_INV)
            continue
        
        mask_x = 0
        mask_y = 0
        if tx is not None:
            # transform sec in microsec
            kx = int(round(tx * US_IN_SECONDS))
            # retreive current sequence to mask pins
            cur_seq_x = stepper_x.get_current_seq()
            #logging.debug("[TX] prev = {} - K = {} - diff : {} - cur_seq_x : {}".format(prevx, kx, kx - prevx, cur_seq_x)) 
            # mask pins
            for i, enable_x in enumerate(cur_seq_x):
                if enable_x:
                    mask_x += 1 << STEPPER_STEP_PINS_X[i]
            # set pulse with diff between current time and previous time
            #logging.debug("[TX] MASK: {} - TIME(us): {}".format(bin(mask_x), kx - prevx))
            dma.add_pulse(mask_x, kx - prevx)
            stepper_x.inc_seq_num()
            prevx = kx + STEPPER_PULSE_LENGTH_US
        if ty is not None:
            ky = int(round(ty * US_IN_SECONDS))
            cur_seq_y = stepper_y.get_current_seq()
            #logging.debug("[TY] prev = {} - K = {} - diff : {} - cur_seq_y : {}".format(prevy, ky, ky - prevy, cur_seq_y)) 
            for j, enable_y in enumerate(cur_seq_y):
                if enable_y:
                    mask_y += 1 << STEPPER_STEP_PINS_Y[j]
            #logging.debug("[TY] MASK: {} - TIME(us): {}".format(bin(mask_y), ky - prevy))
            dma.add_pulse(mask_y, ky - prevy)
            stepper_y.inc_seq_num()
            prevy = ky + STEPPER_PULSE_LENGTH_US
        # run stream ...
#        if not is_ran and instant and current_cb is None:
#            # TODO: wait 100 ms
#            time.sleep(0.1)
#            dma.run_stream()
#            is_ran = True
#            logging.debug("starting ... Addr: {}".format(dma.current_address()))
        idx += 1

    pt = time.time()
    if not is_ran:
        # after long command, we can fill short buffer, that why we may need to
        #  wait until long command finishes
        while dma.is_active():
            time.sleep(0.01)
        dma.run(False)
    else:
        logging.debug("finalize_stream ...")
        # stream mode can be activated only if previous command was finished.
        dma.finalize_stream()

    logging.info("prepared in " + str(round(pt - st, 2)) + "s, estimated in "
                 + str(round(generator.total_time_s(), 2)) + "s")

def join():
    """ Wait till motors work.
    """
    logging.info("hal join()")
    # wait till dma works
    while dma.is_active():
        time.sleep(0.01)

def deinit():
    """ De-initialize hardware.
    """
    logging.info("deinit")
    join()
    disable_steppers()
    pwm.remove_all()
    gpio.clear(PEN_PIN)
    watchdog.stop()

def watchdog_feed():
    """ Feed hardware watchdog.
    """
    watchdog.feed()
