"""
models.py
Contains Tecan Cavro model-specific classes that inherit from the `Syringe`
class in syringe.py.
"""
import time
import logging
import inspect

from math import sqrt
from time import sleep
from functools import wraps
from contextlib import contextmanager
import re

try:
    from gevent import monkey; monkey.patch_all(thread=False)
    from gevent import sleep
except:
    from time import sleep

from syringe import Syringe, SyringeError, SyringeTimeout

class XECavro(Syringe):
    """
    Class to control XCalibur pumps with distribution valves. Provides front-
    end validation and convenience functions (e.g. smartExtract) -- see
    individual docstrings for more information.
    """

    DIR_DICT = {'CW': ('I','O','B'), 'CCW': ('O','I','B')}

    '''
    Need to modify this to be from 20-600
    SPEED_CODES = {0: 6000, 1: 5600, 2: 5000, 3: 4400, 4: 3800, 5: 3200,
                   6: 2600, 7: 2200, 8: 2000, 9: 1800, 10: 1600, 11: 1400,
                   12: 1200, 13: 1000, 14: 800, 15: 600, 16: 400, 17: 200,
                   18: 190, 19: 180, 20: 170, 21: 160, 22: 150, 23: 140,
                   24: 130, 25: 120, 26: 110, 17: 100, 28: 90, 29: 80,
                   30: 70, 31: 60, 32: 50, 33: 40, 34: 30, 35: 20, 36: 18,
                   37: 16, 38: 14, 39: 12, 40: 10}
    '''

    def __init__(self, com_link, num_ports=3, syringe_ul=1000, direction='CW', waste_port=3,
                 debug=False, debug_log_path='.'):
        """
        Object initialization function.
        Args:
            `com_link` (Object) : instantiated TecanAPI subclass / transport
                                  layer (see transport.py)
                *Must have a `.sendRcv(cmd)` instance method to send a command
                    string and parse the reponse (see transport.py)
        Kwargs:
            `num_ports` (int) : number of ports on the distribution valve
                [default] - 9
            `syringe_ul` (int) : syringe volume in microliters
                [default] - 1000
            `waste_port` (int) : waste port for `extractToWaste`-like
                                 convenience functions
                [default] - 9 (factory default for init out port)
            `init_force` (int) : initialization force or speed
                0 [default] - full plunger force and default speed
                1 - half plunger force and default speed
                2 - one third plunger force and default speed
                10-40 - full force and speed code X
            `debug` (bool) : turns on debug file, which logs extensive debug
                             output to 'XECavro_debug.log' at
                             `debug_log_path`
                [default] - False
            `debug_log_path` : path to debug log file - only relevant if
                               `debug` == True.
                [default] - '' (cwd)
        """
        super(XECavro, self).__init__(com_link)
        self.num_ports = num_ports
        self.syringe_ul = syringe_ul
        self.direction = direction
        self.waste_port = waste_port
        self.state = {
            'plunger_pos': None,
            'speed': None,
            'backlash': None,
            'aux_input': None,
            'cur_output': None,
            'buffer_status': None,
            'firmware_version': None 
        }

        # Handle debug mode init
        self.debug = debug
        if self.debug:
            self.initDebugLogging(debug_log_path)

        # Command chaining state information
        self.cmd_chain = ''
        self.exec_time = 0
        self.sim_speed_change = False

        # Init functions
        self.getFirmwareVersion()
        self.getBufferStatus()
        self.getAuxInput()
        self.getCurOutputLine()
        self.getBacklashSteps()
        self.getPlungerPos()
        self.getSpeed()

        self.sim_state = {k: v for k, v in self.state.items()}

    def __repr__(self):
        return 'XECavro(firmware=%s)' % (self.state['firmware_version'])

    def __str__(self):
        s =  'Firmware: ' + str(self.sim_state['firmware_version']) + '\n'
        s += 'Speed           : ' + str(self.sim_state['speed']) + '\n'
        s += 'Backlash        : ' + str(self.sim_state['backlash']) + '\n'
        s += 'Aux Input       : ' + str(self.sim_state['aux_input']) + '\n'
        s += 'Cur Output      : ' + str(self.sim_state['cur_output']) + '\n'
        s += 'Buffer Status.  : ' + str(self.sim_state['buffer_status']) + '\n'
        s += 'Plunger Position: ' + str(self.sim_state['plunger_pos'])
        return s


    #########################################################################
    # Debug functions                                                       #
    #########################################################################

    def initDebugLogging(self, debug_log_path):
        """ Initialize logger and log file handler """

        self.logCall('Tecan XE1000', locals())
        fp = debug_log_path.rstrip('/') + '/tecan_xe1000_debug.log'
        hdlr = logging.FileHandler(fp)
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr)
        self.logger.setLevel(logging.DEBUG)

    def logCall(self, f_name, f_locals):
        """ Logs function params at call """

        if self.debug:
            self.logger.debug('-> {}: {}'.format(f_name, f_locals))

    def logDebug(self, msg):
        """ Handles debug logging if self.debug == True """

        if self.debug:
            self.logger.debug(msg)

    #########################################################################
    # Pump initialization                                                   #
    #########################################################################

    def init(self, init_speed=4, direction=None, in_port=None,
             out_port=None):
        """
        Initialize pump. Uses instance `self.init_force` and `self.direction`
        if not provided. Blocks until initialization is complete.
        """
        self.logCall(inspect.stack()[0][3], locals())

        #init_force = init_force if init_force is not None else self.init_force
        direction = direction if direction is not None else self.direction
        #out_port = out_port if out_port is not None else self.waste_port
        #in_port = in_port if in_port is not None else 0

        if direction == 'CW':
            self.initRightPort(init_speed)
            self.direction = 'CW'
        else:
            self.initLeftPort(init_speed)
            self.direction = 'CCW'

        # cmd_string = '{0}{1},{2},{3}'.format(
        #              self.__class__.DIR_DICT[direction][1],
        #              init_force, in_port, out_port)
        self.sendRcv(self.cmd_chain, execute=True)
        self.waitReady()
        return 0  # 0 seconds left to wait

    #########################################################################
    # Convenience functions                                                 #
    #########################################################################

    def extractToWaste(self, in_port, volume_ul, out_port=None,
                       speed_code=None, minimal_reset=False, flush=False):
        """
        Extracts `volume_ul` from `in_port`. If the relative plunger move
        exceeds the encoder range, the syringe will dispense to `out_port`,
        which defaults to `self.waste_port`. If `minimal_reset` is `True`,
        state updates upon execution will be based on simulations rather
        than polling. If `flush` is `True`, the contents of the syringe
        will be flushed to waste following the extraction.
        """
        self.logCall(inspect.stack()[0][3], locals())

        out_port = out_port if out_port is not None else self.waste_port
        #if speed_code is not None:
        #    self.setSpeed(speed_code)
        #self.cacheSimSpeeds()
        steps = self._ulToSteps(volume_ul)

        retry = False
        extracted = False

        while not extracted:
            # If the move is calculated to execeed 1000 encoder counts,
            # dispense to waste and then make relative plunger extract
            if (self.sim_state['plunger_pos'] + steps) > 1000 or retry:
                self.logDebug('extractToWaste: move would exceed 1000 '
                              'dumping to out port [{}]'.format(out_port))
                self.changePort(out_port, from_port=in_port)
                #self.setSpeed(40)
                self.movePlungerAbs(0)
                self.changePort(in_port, from_port=out_port)
                #self.restoreSimSpeeds()
            # Make relative plunger extract
            self.changePort(in_port)
            self.logDebug('extractToWaste: attempting relative extract '
                          '[steps: {}]'.format(steps))
            # Delay execution 200 ms to stop oscillations
            self.delayMillis(200)
            self.movePlungerRel(steps)
            if flush:
                self.dispenseToWaste()
            exec_time = self.executeChain(minimal_reset=True)
            extracted = True
            '''
            try:
                # If the move is calculated to execeed 1000 encoder counts,
                # dispense to waste and then make relative plunger extract
                if (self.sim_state['plunger_pos'] + steps) > 1000 or retry:
                    self.logDebug('extractToWaste: move would exceed 1000 '
                                  'dumping to out port [{}]'.format(out_port))
                    self.changePort(out_port, from_port=in_port)
                    #self.setSpeed(40)
                    self.movePlungerAbs(0)
                    self.changePort(in_port, from_port=out_port)
                    #self.restoreSimSpeeds()
                # Make relative plunger extract
                self.changePort(in_port)
                self.logDebug('extractToWaste: attempting relative extract '
                              '[steps: {}]'.format(steps))
                # Delay execution 200 ms to stop oscillations
                self.delayMillis(200)
                self.movePlungerRel(steps)
                if flush:
                    self.dispenseToWaste()
                exec_time = self.executeChain(minimal_reset=True)
                extracted = True
            except SyringeError as e:
                if e.err_code in [2, 3, 4]:
                    self.logDebug('extractToWaste: caught SyringeError [{}], '
                                  'retrying.')
                    retry = True
                    self.resetChain()
                    self.waitReady()
                    continue
                else:
                    raise
            '''
        return exec_time

    def primePort(self, in_port, volume_ul, speed_code=None, out_port=None,
                  split_command=False):
        """
        Primes the line on `in_port` with `volume_ul`, which can
        exceed the maximum syringe volume. If `speed_code` is
        provided, the syringe speed will be appended to the
        beginning of the command chain. Blocks until priming is complete.
        """
        self.logCall(inspect.stack()[0][3], locals())

        if out_port is None:
            out_port = self.waste_port
        if speed_code is not None:
            self.setSpeed(speed_code)
        if volume_ul > self.syringe_ul:
            num_rounds = volume_ul / self.syringe_ul
            remainder_ul = volume_ul % self.syringe_ul
            self.changePort(out_port, from_port=in_port)
            self.movePlungerAbs(0)
            for x in xrange(num_rounds):
                self.changePort(in_port, from_port=out_port)
                self.movePlungerAbs(1000)
                self.changePort(out_port, from_port=in_port)
                self.movePlungerAbs(0)
                delay = self.executeChain()
                self.waitReady(delay)
            if remainder_ul != 0:
                self.changePort(in_port, from_port=out_port)
                self.movePlungerAbs(self._ulToSteps(remainder_ul))
                self.changePort(out_port, from_port=in_port)
                self.movePlungerAbs(0)
                delay = self.executeChain()
                self.waitReady(delay)
        else:
            self.changePort(out_port)
            self.movePlungerAbs(0)
            self.changePort(in_port, from_port=out_port)
            self.movePlungerAbs(self._ulToSteps(volume_ul))
            self.changePort(out_port, from_port=in_port)
            self.movePlungerAbs(0)
            delay = self.executeChain()
            self.waitReady(delay)

    #########################################################################
    # Command chain functions                                               #
    #########################################################################

    def executeChain(self, minimal_reset=False):
        """
        Executes and resets the current command chain (`self.cmd_chain`).
        Returns the estimated execution time (`self.exec_time`) for the chain.
        """
        self.logCall(inspect.stack()[0][3], locals())

        # Compensaate for reset time (tic/toc) prior to returning wait_time
        tic = time.time()
        self.sendRcv(self.cmd_chain, execute=True)
        exec_time = self.exec_time
        self.resetChain(on_execute=True, minimal_reset=minimal_reset)
        toc = time.time()
        wait_time = exec_time - (toc-tic)
        if wait_time < 0:
            wait_time = 0
        return wait_time

    def resetChain(self, on_execute=False, minimal_reset=False):
        """
        Resets the command chain (`self.cmd_chain`) and execution time
        (`self.exec_time`). Optionally updates `slope`
        state variables, speeds, and simulation state.
        Kwargs:
            `on_execute` (bool) : should be used to indicate whether or not
                                  the chain being reset was executed, which
                                  will cue slope and microstep state
                                  updating (as well as speed updating).
            `minimal_reset` (bool) : minimizes additional polling of the
                                     syringe pump and updates state based
                                     on simulated calculations. Should
                                     be extremely reliable but use with
                                     caution.
        """
        self.logCall(inspect.stack()[0][3], locals())

        self.cmd_chain = ''
        self.exec_time = 0
        if (on_execute and self.sim_speed_change):
            if minimal_reset:
                self.state = {k: v for k, v in self.sim_state.items()}
        self.sim_speed_change = False
        self.updateSimState()

    def updateSimState(self):
        """
        Copies the current state dictionary (`self.state`) to the
        simulation state dictionary (`self.sim_state`)
        """
        self.logCall(inspect.stack()[0][3], locals())
        self.sim_state = {k: v for k, v in self.state.items()}
    
    def cacheSimSpeeds(self):
        """
        Caches the simulation state speed settings when called. May
        be used for convenience functions in which speed settings
        need to be temporarily changed and then reverted
        """
        self.logCall(inspect.stack()[0][3], locals())

        #self._cached_start_speed = self.sim_state['start_speed']
        self._cached_top_speed = self.sim_state['top_speed']
        #self._cached_cutoff_speed = self.sim_state['cutoff_speed']

    def restoreSimSpeeds(self):
        """ Restores simulation speeds cached by `self.cacheSimSpeeds` """
        self.logCall(inspect.stack()[0][3], locals())

        #self.sim_state['start_speed'] = self._cached_start_speed
        self.sim_state['top_speed'] = self._cached_top_speed
        #self.sim_state['cutoff_speed'] = self._cached_cutoff_speed
        #self.setTopSpeed(self._cached_top_speed)
        if 20 <= self._cached_top_speed <= 600:
            self.setSpeed(self._cached_top_speed)
        #if 50 <= self._cached_cutoff_speed <= 2700:
        #    self.setCutoffSpeed(self._cached_cutoff_speed)
    
    def execWrap(func):
        """
        Decorator to wrap chainable commands, allowing for immediate execution
        of the wrapped command by passing in an `execute=True` kwarg.
        """
        @wraps(func)
        def addAndExec(self, *args, **kwargs):
            execute = False
            if 'execute' in kwargs:
                execute = kwargs.pop('execute')
            if 'minimal_reset' in kwargs:
                minimal_reset = kwargs.pop('minimal_reset')
            else:
                minimal_reset = False
            func(self, *args, **kwargs)
            if execute:
                return self.executeChain(minimal_reset=minimal_reset)
        return addAndExec

    #########################################################################
    # Chainable high level functions                                        #
    #########################################################################

    @execWrap
    def dispenseToWaste(self):
        """
        Dispense current syringe contents to waste.
        """
        self.logCall(inspect.stack()[0][3], locals())
        self.changePort(self.waste_port)
        self.movePlungerAbs(0)

    @execWrap
    def extract(self, from_port, volume_ul):
        """ Extract `volume_ul` from `from_port` """
        self.logCall(inspect.stack()[0][3], locals())

        steps = self._ulToSteps(volume_ul)
        self.changePort(from_port)
        self.movePlungerRel(steps)

    @execWrap
    def dispense(self, to_port, volume_ul):
        """ Dispense `volume_ul` from `to_port` """
        self.logCall(inspect.stack()[0][3], locals())

        steps = self._ulToSteps(volume_ul)
        self.changePort(to_port)
        self.movePlungerRel(-steps)

    #########################################################################
    # Chainable low level functions                                         #
    #########################################################################

    @execWrap
    def changePort(self, to_port, from_port=None):
        """
        Change port to `to_port`. If `from_port` is provided, the `direction`
        will be calculated to minimize travel time. `direction` may also be
        provided directly.
        Args:
            `to_port` (int) : port to which to change
        Kwargs:
            `from_port` (int) : originating port
            `direction` (str) : direction of valve movement
                'CW' - clockwise
                'CCW' - counterclockwise
        """
        self.logCall(inspect.stack()[0][3], locals())

        '''
        if not 0 < to_port <= self.num_ports:
            raise(ValueError('`to_port` [{0}] must be between 1 and '
                             '`num_ports` [{1}]'.format(to_port,
                                                        self.num_ports)))
        if not from_port:
            if self.sim_state['port']:
                from_port = self.sim_state['port']
            else:
                from_port = 1
        delta = to_port - from_port
        diff = -delta if abs(delta) >= 7 else delta
        direction = 'CCW' if diff < 0 else 'CW'
        '''
        to_port_str = ''
        if to_port == 1: 
            to_port_str = self.__class__.DIR_DICT[self.direction][0]
        elif to_port == 2:
            to_port_str = self.__class__.DIR_DICT[self.direction][1]
        elif to_port == 3:
            to_port_str = self.__class__.DIR_DICT[self.direction][2]

        #cmd_string = '{0}{1}'.format(self.__class__.DIR_DICT[direction][0],to_port)
        cmd_string = '{0}'.format(to_port_str)
        self.sim_state['port'] = to_port
        self.cmd_chain += cmd_string
        self.exec_time += 0.5

    @execWrap
    def movePlungerAbs(self, abs_position):
        """
        Moves the plunger to absolute position `abs_position`
        Args:
            `abs_position` (int) : absolute plunger position
                range (0-1000)
        """
        self.logCall(inspect.stack()[0][3], locals())
        if not 0 <= abs_position <= 1000:
            raise(ValueError('`abs_position` must be between 0 and 1000'
                             ' when operating in standard mode'
                             ''.format(self.port_num)))

        cmd_string = 'A{0}'.format(abs_position)

        cur_pos = self.sim_state['plunger_pos']
        delta_pos = cur_pos - abs_position

        self.sim_state['plunger_pos'] = abs_position
        self.cmd_chain += cmd_string
        self.exec_time += self._calcPlungerMoveTime(abs(delta_pos))

    @execWrap
    def movePlungerRel(self, rel_position):
        """
        Moves the plunger to relative position `rel_position`. There is no
        front-end error handling -- invalid relative moves will result in
        error code 3 from the XE1000, raising a `SyringeError`
        Args:
            `rel_position` (int) : relative plunger position
                if -1000 < rel_position < 0 : plunger moves up (relative dispense)
                if 1000 > rel_position > 0 : plunger moves down (relative extract)
        """
        self.logCall(inspect.stack()[0][3], locals())

        if (rel_position <= 0) and (rel_position >= -1000):
            cmd_string = 'D{0}'.format(abs(rel_position))
        elif (rel_position >= 0) and (rel_position <= 1000):
            cmd_string = 'P{0}'.format(rel_position)
        else: 
            raise(ValueError('`rel_position` must be between -1000 and 1000'
                             ''.format(self.rel_position)))

        self.state['plunger_pos'] += rel_position
        self.cmd_chain += cmd_string
        self.exec_time += self._calcPlungerMoveTime(abs(rel_position))

    #########################################################################
    # Command set commands                                                  #
    #########################################################################

    '''
    Initialization Commands
    '''
    @execWrap
    def initLeftPort(self, speed=4):
        '''
        Purpose: 
            Sets Input at Left Port, Output at Right Port
        Example: 
            Z10: initializes the plunger and valve drives to move the plunger at a rate of 
            10seconds/stroke
        Speed:
            Speed is defined as the time it takes the plungers and valve drives to move per stroke.
        Speed Range: 
            Int between 2 and 20. 
        Default:
            4
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 2 <= speed <= 20:
            raise(ValueError('initLeftPort `speed` [{0}] must be between 2 and 20'
                             ''.format(speed)))
        cmd_string = 'Z{0}'.format(speed)
        self.sim_state['speed'] += speed
        self.cmd_chain += cmd_string

    @execWrap
    def initRightPort(self, speed=4):
        '''
        Purpose: 
            Sets Input at Right Port, Output at Left Port
        Example: 
            Y10: initializes the plunger and valve drives to move the plunger at a rate of 
            10seconds/stroke
        Speed:
            Speed is defined as the time it takes the plungers and valve drives to move per stroke.
        Speed Range: 
            Int between 2 and 20. 
        Default:
            4
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 2 <= speed <= 20:
            raise(ValueError('initRightPort `speed` [{0}] must be between 2 and 20'
                             ''.format(speed)))
        cmd_string = 'Y{0}'.format(speed)
        self.sim_state['speed'] += speed
        self.cmd_chain += cmd_string

    '''
    Valve Commands
    '''

    @execWrap
    def moveValveToInputPos(self):
        '''
        Purpose:
            Moves the valve to the input position set by the Y and Z commands
        Example:
            If sent after Z command, initLeftPort(), the valve will be open to the syringe on the left side
            (as seen from the front of the pump). The input position will pull fluid into the syringe
        '''
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = 'I'
        self.cmd_chain += cmd_string

    @execWrap
    def moveValveToOutputPos(self):
        '''
        Purpose:
            Moves the valve to the output position set by the Y and Z commands
        Example:
            If sent after Z command, initLeftPort(), the valve will be open to the syringe on the right side
            (as seen from the front of the pump). The output position will dispense fluid out of the syringe
        '''
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = 'O'
        self.cmd_chain += cmd_string

    @execWrap
    def moveValveToBypassPos(self):
        '''
        CAUTION: 
            When the valve is in this positon do NOT move the syringe plunger. 
            Sending a plunger movement command as error 11 (plunger move not allowed)
        Purpose:
            Moves the valve to connect the input and output positions, bypassing the syringe. 
        Example:
            If sent after Z command, initLeftPort(), the valve will be open to the syringe on the right side
            (as seen from the front of the pump). The output position will dispense fluid out of the syringe
        '''
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = 'B'
        self.cmd_chain += cmd_string

    @execWrap
    def absolutePosition(self, position): 
        '''
        Purpose: 
            The command moves the plunger to the absolute position given between 0 and 1000.
        Example: 
            A300 moves the plunger to position 300.
        Position Range:
            0 - 1000
        Note: 
            Position 0 is the intialization position.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 0 <= position <= 1000:
            raise(ValueError('absolutePosition `position` [{0}] must be between 0 and 1000'
                             ''.format(position)))
        cmd_string = 'A{0}'.format(position)
        self.cmd_chain += cmd_string

    @execWrap
    def relativePickup(self, position): 
        '''
        Purpose: 
            The command moves the plunger down the number of steps commanded. The new absolute position is the 
            previous position + (position)
        Example: 
            The plunger is at 0.
            P300 moves the plunger to position to 0 + 300.
            P600 moves the plunger to position to 300 + 600. 
            The plunger is now at 900.
        Position Range:
            0 - 1000
        Note: 
            Returns error 3 if the final plunger position is greater than 1000.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 0 <= position <= 1000:
            raise(ValueError('relativePosition `position` [{0}] must be between 0 and 1000'
                             ''.format(position)))
        cmd_string = 'P{0}'.format(position)
        self.cmd_chain += cmd_string

    @execWrap
    def relativeDispense(self, position): 
        '''
        Purpose: 
            The command moves the plunger up the number of steps commanded. The new absolute position is the 
            previous position - (position)
        Example: 
            The plunger is at 1000.
            D300 moves the plunger to position to 1000 - 300.
            D600 moves the plunger to position to 700 - 600. 
            The plunger is now at 100.
        Position Range:
            0 - 1000
        Note: 
            Returns error 3 if the final plunger position is less than 0.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 0 <= position <= 1000:
            raise(ValueError('relativeDispense `position` [{0}] must be between 0 and 1000'
                             ''.format(position)))
        cmd_string = 'D{0}'.format(position)
        self.cmd_chain += cmd_string

    @execWrap
    def prime(self):
        '''
        Purpose: 
            Orders the pump to perform two complete aspiration/dispense cycles of the full syringe volume.
            One cycle is equivalent to the following command string: 
                A0IA1000OA0R 
        Note:
            If the plunger is not at the 0 position when the command is first issued, 
            it is moved there before the first complete prime cycle is performed.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = 'p'
        self.cmd_chain += cmd_string

    '''
    Control Commands 
    '''

    @execWrap
    def execute(self):
        '''
        Purpose: 
            Pump exectures a previously sent command string.
            Commands containing R at the end of a string will execute immediately.
            If the command or program string is sent without the R, it is placed in the buffer.
            Sending the command alone will execute the last unexecuted command in the buffer. 
            Sending another R will not repeate the program string.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = 'R'
        self.cmd_chain += cmd_string

    @execWrap
    def executeAgain(self):
        '''
        Purpose:
            Similar to execute, but executes the last executed command or program string.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = 'X'
        self.cmd_chain += cmd_string

    @execWrap
    def repeatCmdSeq(self, N):
        '''
        Purpose: 
            Executes a command sequence N times. 
        N Range:
            0 - 30000
        Note: 
            if 0 is sent, this command will perform indefinitely until a T, terminate(), command is sent.
        Example: 
            [A1000A0G10R] - This command moves the plunger to 1000 and back to 0 , 10 times.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 0 <= N <= 30000:
            raise(ValueError('executeN `N` [{0}] must be between 0 and 30000'
                             ''.format(N)))
        cmd_string = 'G{0}'.format(N)
        self.exec_time = self.exec_time * N
        self.cmd_chain += cmd_string

    @execWrap
    def markRepeatStart(self): 
        '''
        Purpose: 
            This flags a spot in a character sequence that we want to create a repeat cycle. 
        Note: 
            Both g and G commands can only appear once in a string.
        Example: 
            [IA1000M500OgD100M50G10R] - will execute D1000M50 , 10 times. 
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if ((self.cmd_chain.find('g') != -1)):
            raise(ValueError('executeN `g` can only appear once in a command sequence and there is currently a `g` present'))
        cmd_string = 'g'
        self.cmd_chain += cmd_string

    @execWrap
    def delayMillis(self, millis):
        '''
        Purpose: 
            Delays execution of a command in ms to the closest multiple of 5. 
        Note: 
            This command is typically used to allow time for liquid in the syringe and tubing to stop 
            oscillating, thereby enhancing precision. 
        Millis Range: 
            5 - 30000
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 0 <= millis <= 30000:
            raise(ValueError('delayMillis `millis` [{0}] must be between 0 and 30000'
                             ''.format(millis)))
        cmd_string = 'M{0}'.format(millis)
        self.cmd_chain += cmd_string

    @execWrap
    def haltExec(self, N=0):
        '''
        Purpose: 
            Halts the execution of a program string until an execute command is resent.
        N: 
            If N = 0, the program restarts when an R command is sent or an input to go low. (JP3-7)
            if N = 1, the program restarts when an R command is sent or an input to go high. (JP3-7)
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 0 <= N <= 1:
            raise(ValueError('haltExecution `N` [{0}] must be between 0 and 1'
                             ''.format(N)))
        cmd_string = 'N{0}'.format(N)
        self.cmd_chain += cmd_string

    @execWrap
    def terminate(self): 
        '''
        CAUTION: 
            When a plunger move is terminated, lost steps may result. 
            Reinitialization is recommend following a termination.
        Purpose: 
            Emergency stop. 
        Note: 
            This does not terminate Valve Move commands.
        '''
        self.logCall(inspect.stack()[0][3], locals())

        #self.needs_reinitialization == True
        cmd_string = 'T'
        self.cmd_chain += cmd_string

    @execWrap
    def auxOutput(self, N=0):
        '''
        Purpose:
            Set the TTL output line on JP3 pin 13. 
            On power up this is set to 0 (low).
        N Range:
            0 or 1
        Example: 
            J0 = low, GND
            J1 = high, +5V
        '''
        self.logCall(inspect.stack()[0][3], locals())
        if not 0 <= N <= 1:
            raise(ValueError('haltExecution `N` [{0}] must be between 0 and 1'
                             ''.format(N)))
        cmd_string = 'J{0}'.format(N)
        self.cmd_chain += cmd_string

    '''
    Set Commands
    '''

    @execWrap
    def backlashSteps(self, steps=15):
        '''
        Purpose: 
            Sets the number of backlash steps from 0 - 20. 
            When the syrings drive motor reverses direction, the carriage will not move until the backlash,
            due to mechanical play within the system, is compensated. To provide this compensation, during
            aspiration the plunger moves down additional steps, then backs up the set number of backlash steps.
            This ensures that the plunger is in the correct position to begin a dispense move. Note that a small 
            volume of fluid may flow out the "input" side of the valve during this operation.
        Steps Range: 
            0 - 20
        '''
        self.logCall(inspect.stack()[0][3], locals())
        if not 0 <= steps <= 20:
            raise(ValueError('backlashSteps `steps` [{0}] must be between 0 and 20'
                             ''.format(steps)))
        cmd_string = 'K{0}'.format(steps)
        self.cmd_chain += cmd_string

    @execWrap
    def setSpeed(self, speed=40):
        '''
        Purpose: 
            Sets the plunger speed. This specifies the approximate time required for one full stroke (one direction)
        Note: 
            The speed value is equivalent to 2 - 60 seconds per stroke in 0.1 second incremements. 
            S20 = 2 seconds/stroke
            S21 = 2.1 seconds/stroke
        '''
        self.logCall(inspect.stack()[0][3], locals())

        if not 20 <= speed <= 600:
            raise(ValueError('setSpeed `speed` [{0}] must be between 20 and 600'
                             ''.format(speed)))
        cmd_string = 'S{0}'.format(speed)
        self.sim_speed_change = True
        self.sim_state['speed'] += speed/10.0
        self.cmd_chain += cmd_string

    @execWrap
    def setInitializationGap(self, gap_distance):
        '''
        Purpose: 
            Sets the initialization gap or 0 position of the plunger.
            The syntax of this command is the following: 
                ZP10@0 
            Z: Initializes the pump
            P10: Tells the pump to move down 10 steps after initialization
            @0: Tells the pump to set the plinger move (10 steps) to the 0 position
        Gap Distance Range: 
            0 - 50
        Note: 
            A small initialization gap helps preserve the life of the Teflon seal.
            The gap also increases dead volume. The user needs to determine the 
            initialization gap that works best in the application.
        CAUTION: 
            In order to ensure the full 1000 steps of the plunger movement, 
            the initialization gap should NOT exceed 50 steps.
        '''
        self.logCall(inspect.stack()[0][3], locals())
        if not 0 <= gap_distance <= 50:
            raise(ValueError('setInitializationGap `gap_distance` [{0}] must be between 0 and 50'
                             ''.format(gap_distance)))
        cmd_string = 'ZP{0}@0'.format(gap_distance) 
        self.cmd_chain += cmd_string

    '''
    Report Commands
    '''
    def getPlungerPos(self):
        """ Returns the absolute plunger position as an int (0-1000) """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = '?'
        data = self.sendRcv(cmd_string)
        self.state['plunger_pos'] = int(data)
        return self.state['plunger_pos']
    
    def getSpeed(self):
        """ Returns the start speed as an int (in sec/stroke) """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = '?S'
        data = self.sendRcv(cmd_string)
        self.state['speed'] = int(data)*10.0
        return self.state['speed']

    def getBacklashSteps(self):
        """ Returns the backlash setting as an int """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = '?K'
        data = self.sendRcv(cmd_string)
        self.state['backlash'] = int(data)
        return self.state['backlash']

    def getAuxInput(self):
        """ Returns the aux input as an int 
            0 = low
            1 = high
        """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = '?I'
        data = self.sendRcv(cmd_string)
        self.state['aux_input'] = int(data)
        return self.state['aux_input']

    def getCurOutputLine(self):
        """ Returns the current output line setting as an int (JP3-13)
            0 = low
            1 = high
        """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = '?I'
        data = self.sendRcv(cmd_string)
        self.state['cur_output'] = int(data)
        return self.state['cur_output']

    def getBufferStatus(self):
        """ Returns the current buffer status.
            0 = empty buffer
            1 = not empty
        """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = 'F'
        data = self.sendRcv(cmd_string)
        self.state['buffer_status'] = int(data)
        return self.state['buffer_status']

    def getFirmwareVersion(self):
        """ Returns the XE 1000 firmware revision code
        """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = '&'
        data = self.sendRcv(cmd_string)
        self.state['firmware_version'] = (data)
        return self.state['firmware_version']

    def getBuffer(self):
        """ Returns the command string in the buffer
        """
        self.logCall(inspect.stack()[0][3], locals())

        cmd_string = '#'
        data = self.sendRcv(cmd_string)
        self.state['buffer'] = (data)
        return self.state['buffer']

    #########################################################################
    # Communication handlers and special functions                          #
    #########################################################################

    @contextmanager
    def _syringeErrorHandler(self):
        """
        Context manager to handle `SyringeError` based on error code. Right
        now this just handles error codes 7, 9, and 10 by initializing the
        pump and then re-running the previous command.
        """
        try:
            yield
        except SyringeError as e:
            self.logDebug('ErrorHandler: caught error code {}'.format(
                          e.err_code))
            if e.err_code in [7, 9, 10]:
                last_cmd = self.last_cmd
                self.resetChain()
                try:
                    self.logDebug('ErrorHandler: attempting re-init')
                    self.init()
                except SyringeError as e:
                    self.logDebug('ErrorHandler: Error during re-init '
                                  '[{}]'.format(e.err_code))
                    if e.err_code in [7, 9, 10]:
                        pass
                    else:
                        raise e
                self._waitReady()
                self.logDebug('ErrorHandler: resending last command {} '
                              ''.format(last_cmd))
                self.sendRcv(last_cmd)
            else:
                self.logDebug('ErrorHandler: error not in [7, 9, 10], '
                              're-raising [{}]'.format(e.err_code))
                self.resetChain()
                raise e
        except Exception as e:
            self.resetChain()
            raise e

    def waitReady(self, timeout=10, polling_interval=0.3, delay=None):
        """
        Waits a maximum of `timeout` seconds for the syringe to be
        ready to accept another set command, polling every `polling_interval`
        seconds. If a `delay` is provided, the function will sleep `delay`
        seconds prior to beginning polling.
        """
        self.logCall(inspect.stack()[0][3], locals())
        with self._syringeErrorHandler():
            self._waitReady(timeout=timeout, polling_interval=polling_interval,
                            delay=delay)

    def sendRcv(self, cmd_string, execute=False):
        """
        Send a raw command string and return a tuple containing the parsed
        response data: (Data, Ready). If the syringe is ready to accept
        another command, `Ready` with be 'True'.
        Args:
            `cmd_string` (bytestring) : a valid Tecan XCalibur command string
        Kwargs:
            `execute` : if 'True', the execute byte ('R') is appended to the
                        `cmd_string` prior to sending
        Returns:
            `parsed_reponse` (tuple) : parsed pump response tuple
        """
        self.logCall(inspect.stack()[0][3], locals())

        if execute:
            cmd_string += 'R'
        self.last_cmd = cmd_string
        self.logDebug('sendRcv: sending cmd_string: {}'.format(cmd_string))
        with self._syringeErrorHandler():
            parsed_response = super(XECavro, self)._sendRcv(cmd_string)
            self.logDebug('sendRcv: received response: {}'.format(
                          parsed_response))
            data = parsed_response[0]
            return data

    def _calcPlungerMoveTime(self, move_steps, buffertime=0.1):
        '''
        move_steps/1000 => number of strokes
        self.sim_state['speed'] => strokes/second
        0.1 => added buffertime for potential innacuraccies from this system.
        '''
        return move_steps/1000.0*self.sim_state['speed']+buffertime

    def _ulToSteps(self, volume_ul):
        """
        Converts a volume in microliters (ul) to encoder steps.
        Args:
            `volume_ul` (int) : volume in microliters
        """

        steps = volume_ul * (1000/self.syringe_ul)
        return steps
    '''
    def _simIncToPulses(self, speed_inc):
        """
        Updates simulation speeds given a speed increment setting (`speed_inc`)
        following XCalibur handling of speed changes (i.e. cutoff speed cannot
        be higher than top speed, so it is automatically adjusted on the pump)
        """
        top_speed = self.__class__.SPEED_CODES[speed_inc]
        self.sim_state['top_speed'] = top_speed
        if self.sim_state['start_speed'] > top_speed:
            self.sim_state['start_speed'] = top_speed
        if self.sim_state['cutoff_speed'] > top_speed:
            self.sim_state['cutoff_speed'] = top_speed
    '''


class XCaliburD(Syringe):
    """
    Class to control XCalibur pumps with distribution valves. Provides front-
    end validation and convenience functions (e.g. smartExtract) -- see
    individual docstrings for more information.
    """

    DIR_DICT = {'CW': ('I', 'Z'), 'CCW': ('O', 'Y')}

    SPEED_CODES = {0: 6000, 1: 5600, 2: 5000, 3: 4400, 4: 3800, 5: 3200,
                   6: 2600, 7: 2200, 8: 2000, 9: 1800, 10: 1600, 11: 1400,
                   12: 1200, 13: 1000, 14: 800, 15: 600, 16: 400, 17: 200,
                   18: 190, 19: 180, 20: 170, 21: 160, 22: 150, 23: 140,
                   24: 130, 25: 120, 26: 110, 17: 100, 28: 90, 29: 80,
                   30: 70, 31: 60, 32: 50, 33: 40, 34: 30, 35: 20, 36: 18,
                   37: 16, 38: 14, 39: 12, 40: 10}

    def __init__(self, com_link, num_ports=9, syringe_ul=1000, direction='CW',
                 microstep=False, waste_port=9, slope=14, init_force=0,
                 debug=False, debug_log_path='.'):
        """
        Object initialization function.
        Args:
            `com_link` (Object) : instantiated TecanAPI subclass / transport
                                  layer (see transport.py)
                *Must have a `.sendRcv(cmd)` instance method to send a command
                    string and parse the reponse (see transport.py)
        Kwargs:
            `num_ports` (int) : number of ports on the distribution valve
                [default] - 9
            `syringe_ul` (int) : syringe volume in microliters
                [default] - 1000
            `microstep` (bool) : whether or not to operate in microstep mode
                [default] - False (factory default)
            `waste_port` (int) : waste port for `extractToWaste`-like
                                 convenience functions
                [default] - 9 (factory default for init out port)
            `slope` (int) : slope setting
                [default] - 14 (factory default)
            `init_force` (int) : initialization force or speed
                0 [default] - full plunger force and default speed
                1 - half plunger force and default speed
                2 - one third plunger force and default speed
                10-40 - full force and speed code X
            `debug` (bool) : turns on debug file, which logs extensive debug
                             output to 'xcaliburd_debug.log' at
                             `debug_log_path`
                [default] - False
            `debug_log_path` : path to debug log file - only relevant if
                               `debug` == True.
                [default] - '' (cwd)
        """
        super(XCaliburD, self).__init__(com_link)
        self.num_ports = num_ports
        self.syringe_ul = syringe_ul
        self.direction = direction
        self.waste_port = waste_port
        self.init_force = init_force
        self.state = {
            'plunger_pos': None,
            'port': None,
            'microstep': microstep,
            'start_speed': None,
            'top_speed': None,
            'cutoff_speed': None,
            'slope': slope
        }

        # Handle debug mode init
        self.debug = debug
        if self.debug:
            self.initDebugLogging(debug_log_path)

        self.setMicrostep(on=microstep)

        # Command chaining state information
        self.cmd_chain = ''
        self.exec_time = 0
        self.sim_speed_change = False
        self.sim_state = {k: v for k,v in self.state.items()}

        # Init functions
        self.updateSpeeds()
        self.getPlungerPos()
        self.getCurPort()
        self.updateSimState()

    #########################################################################
    # Debug functions                                                       #
    #########################################################################

    def initDebugLogging(self, debug_log_path):
        """ Initialize logger and log file handler """

        self.logger = logging.getLogger('XCaliburD')
        fp = debug_log_path.rstrip('/') + '/xcaliburd_debug.log'
        hdlr = logging.FileHandler(fp)
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr)
        self.logger.setLevel(logging.DEBUG)

    def logCall(self, f_name, f_locals):
        """ Logs function params at call """

        if self.debug:
            self.logger.debug('-> {}: {}'.format(f_name, f_locals))

    def logDebug(self, msg):
        """ Handles debug logging if self.debug == True """

        if self.debug:
            self.logger.debug(msg)

    #########################################################################
    # Pump initialization                                                   #
    #########################################################################

    def init(self, init_force=None, direction=None, in_port=None,
             out_port=None):
        """
        Initialize pump. Uses instance `self.init_force` and `self.direction`
        if not provided. Blocks until initialization is complete.
        """
        self.logCall('init', locals())

        init_force = init_force if init_force is not None else self.init_force
        direction = direction if direction is not None else self.direction
        out_port = out_port if out_port is not None else self.waste_port
        in_port = in_port if in_port is not None else 0

        cmd_string = '{0}{1},{2},{3}'.format(
                     self.__class__.DIR_DICT[direction][1],
                     init_force, in_port, out_port)
        self.sendRcv(cmd_string, execute=True)
        self.waitReady()
        return 0  # 0 seconds left to wait

    #########################################################################
    # Convenience functions                                                 #
    #########################################################################

    def extractToWaste(self, in_port, volume_ul, out_port=None,
                       speed_code=None, minimal_reset=False, flush=False):
        """
        Extracts `volume_ul` from `in_port`. If the relative plunger move
        exceeds the encoder range, the syringe will dispense to `out_port`,
        which defaults to `self.waste_port`. If `minimal_reset` is `True`,
        state updates upon execution will be based on simulations rather
        than polling. If `flush` is `True`, the contents of the syringe
        will be flushed to waste following the extraction.
        """
        self.logCall('extractToWaste', locals())

        out_port = out_port if out_port is not None else self.waste_port
        if speed_code is not None:
            self.setSpeed(speed_code)
        self.cacheSimSpeeds()
        steps = self._ulToSteps(volume_ul)

        retry = False
        extracted = False

        while not extracted:
            try:
                # If the move is calculated to execeed 3000 encoder counts,
                # dispense to waste and then make relative plunger extract
                if (self.sim_state['plunger_pos'] + steps) > 3000 or retry:
                    self.logDebug('extractToWaste: move would exceed 3000 '
                                  'dumping to out port [{}]'.format(out_port))
                    self.changePort(out_port, from_port=in_port)
                    self.setSpeed(0)
                    self.movePlungerAbs(0)
                    self.changePort(in_port, from_port=out_port)
                    self.restoreSimSpeeds()
                # Make relative plunger extract
                self.changePort(in_port)
                self.logDebug('extractToWaste: attempting relative extract '
                              '[steps: {}]'.format(steps))
                # Delay execution 200 ms to stop oscillations
                self.delayExec(200)
                self.movePlungerRel(steps)
                if flush:
                    self.dispenseToWaste()
                exec_time = self.executeChain(minimal_reset=True)
                extracted = True
            except SyringeError as e:
                if e.err_code in [2, 3, 4]:
                    self.logDebug('extractToWaste: caught SyringeError [{}], '
                                  'retrying.')
                    retry = True
                    self.resetChain()
                    self.waitReady()
                    continue
                else:
                    raise
        return exec_time

    def primePort(self, in_port, volume_ul, speed_code=None, out_port=None,
                  split_command=False):
        """
        Primes the line on `in_port` with `volume_ul`, which can
        exceed the maximum syringe volume. If `speed_code` is
        provided, the syringe speed will be appended to the
        beginning of the command chain. Blocks until priming is complete.
        """
        self.logCall('primePort', locals())

        if out_port is None:
            out_port = self.waste_port
        if speed_code is not None:
            self.setSpeed(speed_code)
        if volume_ul > self.syringe_ul:
            num_rounds = volume_ul / self.syringe_ul
            remainder_ul = volume_ul % self.syringe_ul
            self.changePort(out_port, from_port=in_port)
            self.movePlungerAbs(0)
            for x in xrange(num_rounds):
                self.changePort(in_port, from_port=out_port)
                self.movePlungerAbs(3000)
                self.changePort(out_port, from_port=in_port)
                self.movePlungerAbs(0)
                delay = self.executeChain()
                self.waitReady(delay)
            if remainder_ul != 0:
                self.changePort(in_port, from_port=out_port)
                self.movePlungerAbs(self._ulToSteps(remainder_ul))
                self.changePort(out_port, from_port=in_port)
                self.movePlungerAbs(0)
                delay = self.executeChain()
                self.waitReady(delay)
        else:
            self.changePort(out_port)
            self.movePlungerAbs(0)
            self.changePort(in_port, from_port=out_port)
            self.movePlungerAbs(self._ulToSteps(volume_ul))
            self.changePort(out_port, from_port=in_port)
            self.movePlungerAbs(0)
            delay = self.executeChain()
            self.waitReady(delay)

    #########################################################################
    # Command chain functions                                               #
    #########################################################################

    def executeChain(self, minimal_reset=False):
        """
        Executes and resets the current command chain (`self.cmd_chain`).
        Returns the estimated execution time (`self.exec_time`) for the chain.
        """
        self.logCall('executeChain', locals())

        # Compensaate for reset time (tic/toc) prior to returning wait_time
        tic = time.time()
        self.sendRcv(self.cmd_chain, execute=True)
        exec_time = self.exec_time
        self.resetChain(on_execute=True, minimal_reset=minimal_reset)
        toc = time.time()
        wait_time = exec_time - (toc-tic)
        if wait_time < 0:
            wait_time = 0
        return wait_time

    def resetChain(self, on_execute=False, minimal_reset=False):
        """
        Resets the command chain (`self.cmd_chain`) and execution time
        (`self.exec_time`). Optionally updates `slope` and `microstep`
        state variables, speeds, and simulation state.
        Kwargs:
            `on_execute` (bool) : should be used to indicate whether or not
                                  the chain being reset was executed, which
                                  will cue slope and microstep state
                                  updating (as well as speed updating).
            `minimal_reset` (bool) : minimizes additional polling of the
                                     syringe pump and updates state based
                                     on simulated calculations. Should
                                     be extremely reliable but use with
                                     caution.
        """
        self.logCall('resetChain', locals())

        self.cmd_chain = ''
        self.exec_time = 0
        if (on_execute and self.sim_speed_change):
            if minimal_reset:
                self.state = {k: v for k,v in self.sim_state.items()}
            else:
                self.state['slope'] = self.sim_state['slope']
                self.state['microstep'] = self.sim_state['microstep']
                self.updateSpeeds()
                self.getCurPort()
                self.getPlungerPos()
        self.sim_speed_change = False
        self.updateSimState()

    def updateSimState(self):
        """
        Copies the current state dictionary (`self.state`) to the
        simulation state dictionary (`self.sim_state`)
        """
        self.logCall('updateSimState', locals())

        self.sim_state = {k: v for k,v in self.state.items()}

    def cacheSimSpeeds(self):
        """
        Caches the simulation state speed settings when called. May
        be used for convenience functions in which speed settings
        need to be temporarily changed and then reverted
        """
        self.logCall('cacheSimSpeeds', locals())

        self._cached_start_speed = self.sim_state['start_speed']
        self._cached_top_speed = self.sim_state['top_speed']
        self._cached_cutoff_speed = self.sim_state['cutoff_speed']

    def restoreSimSpeeds(self):
        """ Restores simulation speeds cached by `self.cacheSimSpeeds` """
        self.logCall('restoreSimSpeeds', locals())

        self.sim_state['start_speed'] = self._cached_start_speed
        self.sim_state['top_speed'] = self._cached_top_speed
        self.sim_state['cutoff_speed'] = self._cached_cutoff_speed
        self.setTopSpeed(self._cached_top_speed)
        if 50 <= self._cached_start_speed <= 1000:
            self.setStartSpeed(self._cached_start_speed)
        if 50 <= self._cached_cutoff_speed <= 2700:
            self.setCutoffSpeed(self._cached_cutoff_speed)

    def execWrap(func):
        """
        Decorator to wrap chainable commands, allowing for immediate execution
        of the wrapped command by passing in an `execute=True` kwarg.
        """
        @wraps(func)
        def addAndExec(self, *args, **kwargs):
            execute = False
            if 'execute' in kwargs:
                execute = kwargs.pop('execute')
            if 'minimal_reset' in kwargs:
                minimal_reset = kwargs.pop('minimal_reset')
            else:
                minimal_reset = False
            func(self, *args, **kwargs)
            if execute:
                return self.executeChain(minimal_reset=minimal_reset)
        return addAndExec

    #########################################################################
    # Chainable high level functions                                        #
    #########################################################################

    @execWrap
    def dispenseToWaste(self, retain_port=True):
        """
        Dispense current syringe contents to waste. If `retain_port` is true,
        the syringe will be returned to the original port after the dump.
        """
        self.logCall('dispenseToWaste', locals())
        if retain_port:
            orig_port = self.sim_state['port']
        self.changePort(self.waste_port)
        self.setSpeed(0)
        self.movePlungerAbs(0)
        self.restoreSimSpeeds()
        if retain_port:
            self.changePort(orig_port)

    @execWrap
    def extract(self, from_port, volume_ul):
        """ Extract `volume_ul` from `from_port` """
        self.logCall('extract', locals())

        steps = self._ulToSteps(volume_ul)
        self.changePort(from_port)
        self.movePlungerRel(steps)

    @execWrap
    def dispense(self, to_port, volume_ul):
        """ Dispense `volume_ul` from `to_port` """
        self.logCall('dispense', locals())

        steps = self._ulToSteps(volume_ul)
        self.changePort(to_port)
        self.movePlungerRel(-steps)

    #########################################################################
    # Chainable low level functions                                         #
    #########################################################################

    @execWrap
    def changePort(self, to_port, from_port=None, direction='CW'):
        """
        Change port to `to_port`. If `from_port` is provided, the `direction`
        will be calculated to minimize travel time. `direction` may also be
        provided directly.
        Args:
            `to_port` (int) : port to which to change
        Kwargs:
            `from_port` (int) : originating port
            `direction` (str) : direction of valve movement
                'CW' - clockwise
                'CCW' - counterclockwise
        """
        self.logCall('changePort', locals())

        if not 0 < to_port <= self.num_ports:
            raise(ValueError('`to_port` [{0}] must be between 1 and '
                             '`num_ports` [{1}]'.format(to_port,
                             self.num_ports)))
        if not from_port:
            if self.sim_state['port']:
                from_port = self.sim_state['port']
            else:
                from_port = 1
        delta = to_port - from_port
        diff = -delta if abs(delta) >= 7 else delta
        direction = 'CCW' if diff < 0 else 'CW'
        cmd_string = '{0}{1}'.format(self.__class__.DIR_DICT[direction][0],
                                     to_port)
        self.sim_state['port'] = to_port
        self.cmd_chain += cmd_string
        self.exec_time += 0.2

    @execWrap
    def movePlungerAbs(self, abs_position):
        """
        Moves the plunger to absolute position `abs_position`
        Args:
            `abs_position` (int) : absolute plunger position
                (0-24000) in microstep mode
                (0-3000) in standard mode
        """
        self.logCall('movePlungerAbs', locals())

        if self.sim_state['microstep']:
            if not 0 <= abs_position <= 24000:
                raise(ValueError('`abs_position` must be between 0 and 40000'
                                 ' when operating in microstep mode'.format(
                                 self.port_num)))
        else:
            if not 0 <= abs_position <= 3000:
                raise(ValueError('`abs_position` must be between 0 and 40000'
                                 ' when operating in microstep mode'.format(
                                 self.port_num)))
        cmd_string = 'A{0}'.format(abs_position)
        cur_pos = self.sim_state['plunger_pos']
        delta_pos = cur_pos-abs_position
        self.sim_state['plunger_pos'] = abs_position
        self.cmd_chain += cmd_string
        self.exec_time += self._calcPlungerMoveTime(abs(delta_pos))

    @execWrap
    def movePlungerRel(self, rel_position):
        """
        Moves the plunger to relative position `rel_position`. There is no
        front-end error handling -- invalid relative moves will result in
        error code 3 from the XCalibur, raising a `SyringeError`
        Args:
            `rel_position` (int) : relative plunger position
                if rel_position < 0 : plunger moves up (relative dispense)
                if rel_position > 0 : plunger moves down (relative extract)
        """
        self.logCall('movePlungerRel', locals())

        if rel_position < 0:
            cmd_string = 'D{0}'.format(abs(rel_position))
        else:
            cmd_string = 'P{0}'.format(rel_position)
        self.sim_state['plunger_pos'] += rel_position
        self.cmd_chain += cmd_string
        self.exec_time += self._calcPlungerMoveTime(abs(rel_position))

    #########################################################################
    # Command set commands                                                  #
    #########################################################################

    @execWrap
    def setSpeed(self, speed_code):
        """ Set top speed by `speed_code` (see OEM docs) """
        self.logCall('setSpeed', locals())

        if not 0 <= speed_code <= 40:
            raise(ValueError('`speed_code` [{0}] must be between 0 and 40'
                             ''.format(speed_code)))
        cmd_string = 'S{0}'.format(speed_code)
        self.sim_speed_change = True
        self._simIncToPulses(speed_code)
        self.cmd_chain += cmd_string

    @execWrap
    def setStartSpeed(self, pulses_per_sec):
        """ Set start speed in `pulses_per_sec` [50-1000] """
        self.logCall('setStartSpeed', locals())

        cmd_string = 'v{0}'.format(pulses_per_sec)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    @execWrap
    def setTopSpeed(self, pulses_per_sec):
        """ Set top speed in `pulses_per_sec` [5-6000] """
        self.logCall('setTopSpeed', locals())

        cmd_string = 'V{0}'.format(pulses_per_sec)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    @execWrap
    def setCutoffSpeed(self, pulses_per_sec):
        """ Set cutoff speed in `pulses_per_sec` [50-2700] """
        self.logCall('setCutoffSpeed', locals())

        cmd_string = 'c{0}'.format(pulses_per_sec)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    @execWrap
    def setSlope(self, slope_code, chain=False):
        self.logCall('setSlope', locals())

        if not 1 <= slope_code <= 20:
            raise(ValueError('`slope_code` [{0}] must be between 1 and 20'
                             ''.format(slope_code)))
        cmd_string = 'L{0}'.format(slope_code)
        self.sim_speed_change = True
        self.cmd_chain += cmd_string

    # Chainable control commands

    @execWrap
    def repeatCmdSeq(self, num_repeats):
        self.logCall('repeatCmdSeq', locals())

        if not 0 < num_repeats < 30000:
            raise(ValueError('`num_repeats` [{0}] must be between 0 and 30000'
                             ''.format(num_repeats)))
        cmd_string = 'G{0}'.format(num_repeats)
        self.cmd_chain += cmd_string
        self.exec_time *= num_repeats

    @execWrap
    def markRepeatStart(self):
        self.logCall('markRepeatStart', locals())

        cmd_string = 'g'
        self.cmd_chain += cmd_string

    @execWrap
    def delayExec(self, delay_ms):
        """ Delays command execution for `delay` milliseconds """
        self.logCall('delayExec', locals())

        if not 0 < delay_ms < 30000:
            raise(ValueError('`delay` [{0}] must be between 0 and 40000 ms'
                             ''.format(delay_ms)))
        cmd_string = 'M{0}'.format(delay_ms)
        self.cmd_chain += cmd_string

    @execWrap
    def haltExec(self, input_pin=0):
        """
        Used within a command string to halt execution until another [R]
        command is sent, or until TTL pin `input_pin` goes low
        Kwargs:
            `input_pin` (int) : input pin code corresponding to the desired
                                TTL input signal pin on the XCalibur
                0 - either 1 or 2
                1 - input 1 (J4 pin 7)
                2 - input 2 (J4 pin 8)
        """
        self.logCall('haltExec', locals())

        if not 0 <= input_pin < 2:
            raise(ValueError('`input_pin` [{0}] must be between 0 and 2'
                             ''.format(input_sig)))
        cmd_string = 'H{0}'.format(input_sig)
        return self.sendRcv(cmd_string)

    #########################################################################
    # Report commands (cannot be chained)                                   #
    #########################################################################

    def updateSpeeds(self):
        self.logCall('updateSpeeds', locals())

        self.getStartSpeed()
        self.getTopSpeed()
        self.getCutoffSpeed()

    def getPlungerPos(self):
        """ Returns the absolute plunger position as an int (0-3000) """
        self.logCall('getPlungerPos', locals())

        cmd_string = '?'
        data = self.sendRcv(cmd_string)
        self.state['plunger_pos'] = int(data)
        return self.state['plunger_pos']

    def getStartSpeed(self):
        """ Returns the start speed as an int (in pulses/sec) """
        self.logCall('getStartSpeed', locals())

        cmd_string = '?1'
        data = self.sendRcv(cmd_string)
        self.state['start_speed'] = int(data)
        return self.state['start_speed']

    def getTopSpeed(self):
        """ Returns the top speed as an int (in pulses/sec) """
        self.logCall('getTopSpeed', locals())

        cmd_string = '?2'
        data = self.sendRcv(cmd_string)
        self.state['top_speed'] = int(data)
        return self.state['top_speed']

    def getCutoffSpeed(self):
        """ Returns the cutoff speed as an int (in pulses/sec) """
        self.logCall('getCutoffSpeed', locals())

        cmd_string = '?3'
        data = self.sendRcv(cmd_string)
        self.state['cutoff_speed'] = int(data)
        return self.state['cutoff_speed']

    def getEncoderPos(self):
        """ Returns the current encoder count on the plunger axis """
        self.logCall('getEncoderPos', locals())

        cmd_string = '?4'
        data = self.sendRcv(cmd_string)
        return int(data)

    def getCurPort(self):
        """ Returns the current port position (1-num_ports) """
        self.logCall('getCurPort', locals())

        cmd_string = '?6'
        data = self.sendRcv(cmd_string)
        with self._syringeErrorHandler():
            try:
                port = int(data)
            except ValueError:
                raise SyringeError(7, self.__class__.ERROR_DICT)
            self.state['port'] = port
            return port

    def getBufferStatus(self):
        """ Returns the current cmd buffer status (0=empty, 1=non-empty) """
        self.logCall('getBufferStatus', locals())

        cmd_string = '?10'
        data = self.sendRcv(cmd_string)
        return int(data)

    #########################################################################
    # Config commands                                                       #
    #########################################################################

    def setMicrostep(self, on=False):
        """ Turns microstep mode on or off """
        self.logCall('setMicrostep', locals())

        cmd_string = 'N{0}'.format(int(on))
        self.sendRcv(cmd_string, execute=True)
        self.microstep = on

    #########################################################################
    # Control commands                                                      #
    #########################################################################

    def terminateCmd(self):
        self.logCall('terminateCommand', locals())

        cmd_string = 'T'
        return self.sendRcv(cmd_string, execute=True)

    #########################################################################
    # Communication handlers and special functions                          #
    #########################################################################

    @contextmanager
    def _syringeErrorHandler(self):
        """
        Context manager to handle `SyringeError` based on error code. Right
        now this just handles error codes 7, 9, and 10 by initializing the
        pump and then re-running the previous command.
        """
        try:
            yield
        except SyringeError as e:
            self.logDebug('ErrorHandler: caught error code {}'.format(
                          e.err_code))
            if e.err_code in [7, 9, 10]:
                last_cmd = self.last_cmd
                self.resetChain()
                try:
                    self.logDebug('ErrorHandler: attempting re-init')
                    self.init()
                except SyringeError as e:
                    self.logDebug('ErrorHandler: Error during re-init '
                                  '[{}]'.format(e.err_code))
                    if e.err_code in [7, 9, 10]:
                        pass
                    else:
                        raise e
                self._waitReady()
                self.logDebug('ErrorHandler: resending last command {} '
                              ''.format(last_cmd))
                self.sendRcv(last_cmd)
            else:
                self.logDebug('ErrorHandler: error not in [7, 9, 10], '
                              're-raising [{}]'.format(e.err_code))
                self.resetChain()
                raise e
        except Exception as e:
            self.resetChain()
            raise e

    def waitReady(self, timeout=10, polling_interval=0.3, delay=None):
        """
        Waits a maximum of `timeout` seconds for the syringe to be
        ready to accept another set command, polling every `polling_interval`
        seconds. If a `delay` is provided, the function will sleep `delay`
        seconds prior to beginning polling.
        """
        self.logCall('waitReady', locals())
        with self._syringeErrorHandler():
            self._waitReady(timeout=timeout, polling_interval=polling_interval,
                            delay=delay)

    def sendRcv(self, cmd_string, execute=False):
        """
        Send a raw command string and return a tuple containing the parsed
        response data: (Data, Ready). If the syringe is ready to accept
        another command, `Ready` with be 'True'.
        Args:
            `cmd_string` (bytestring) : a valid Tecan XCalibur command string
        Kwargs:
            `execute` : if 'True', the execute byte ('R') is appended to the
                        `cmd_string` prior to sending
        Returns:
            `parsed_reponse` (tuple) : parsed pump response tuple
        """
        self.logCall('sendRcv', locals())

        if execute:
            cmd_string += 'R'
        self.last_cmd = cmd_string
        self.logDebug('sendRcv: sending cmd_string: {}'.format(cmd_string))
        with self._syringeErrorHandler():
            parsed_response = super(XCaliburD, self)._sendRcv(cmd_string)
            self.logDebug('sendRcv: received response: {}'.format(
                          parsed_response))
            data = parsed_response[0]
            return data

    def _calcPlungerMoveTime(self, move_steps):
        """
        Calculates plunger move time using equations provided by Tecan.
        Assumes that all input values have been validated
        """
        sd = self.sim_state
        start_speed = sd['start_speed']
        top_speed = sd['top_speed']
        cutoff_speed = sd['cutoff_speed']
        slope = sd['slope']
        microstep = sd['microstep']

        slope *= 2500.0
        if microstep:
            move_steps = move_steps / 8.0
        theo_top_speed = sqrt((4.0 * move_steps*slope) + start_speed ** 2.0)
        # If theoretical top speed will not exceed cutoff speed
        if theo_top_speed < cutoff_speed:
            move_t = theo_top_speed - (start_speed/slope)
        else:
            theo_top_speed = sqrt(((2.0*move_steps*slope) +
                             ((start_speed**2.0+cutoff_speed**2.0)/2.0)))
        # If theoretical top speed with exceed cutoff speed but not
        # reach the set top speed
        if cutoff_speed < theo_top_speed < top_speed:
            move_t = ((1 / slope) * (2.0 * theo_top_speed - start_speed -
                     cutoff_speed))
        # If start speed, top speed, and cutoff speed are all the same
        elif start_speed == top_speed == cutoff_speed:
            move_t = (2.0 * move_steps) / top_speed
        # Otherwise, calculate time spent in each phase (start, constant,
        # ramp down)
        else:
            ramp_up_halfsteps = ((top_speed ** 2.0 - start_speed ** 2.0) /
                                (2.0 * slope))
            ramp_down_halfsteps = ((top_speed ** 2.0 - cutoff_speed ** 2.0) /
                                  (2.0 * slope))
            if (ramp_up_halfsteps + ramp_down_halfsteps) < (2.0 * top_speed):
                ramp_up_t = (top_speed - start_speed) / slope
                ramp_down_t = (top_speed - cutoff_speed) / slope
                constant_halfsteps = (2.0 * move_steps - ramp_up_halfsteps -
                                     ramp_down_halfsteps)
                constant_t = constant_halfsteps / top_speed
                move_t = ramp_up_t + ramp_down_t + constant_t
        return move_t

    def _ulToSteps(self, volume_ul, microstep=None):
        """
        Converts a volume in microliters (ul) to encoder steps.
        Args:
            `volume_ul` (int) : volume in microliters
        Kwargs:
            `microstep` (bool) : whether to convert to standard steps or
                                 microsteps
        """
        if not microstep: microstep = self.state['microstep']
        if microstep:
            steps = volume_ul * (24000/self.syringe_ul)
        else:
            steps = volume_ul * (3000/self.syringe_ul)
        return steps

    def _simIncToPulses(self, speed_inc):
        """
        Updates simulation speeds given a speed increment setting (`speed_inc`)
        following XCalibur handling of speed changes (i.e. cutoff speed cannot
        be higher than top speed, so it is automatically adjusted on the pump)
        """
        top_speed = self.__class__.SPEED_CODES[speed_inc]
        self.sim_state['top_speed'] = top_speed
        if self.sim_state['start_speed'] > top_speed:
            self.sim_state['start_speed'] = top_speed
        if self.sim_state['cutoff_speed'] > top_speed:
            self.sim_state['cutoff_speed'] = top_speed