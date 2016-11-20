#!/usr/bin/env python
#
# backend for serial IO for POSIX compatible systems, like Linux, OSX
#
# This file is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2001-2015 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause
#
# parts based on code from Grant B. Edwards  <grante@visi.com>:
#  ftp://ftp.visi.com/users/grante/python/PosixSerial.py
#
# references: http://www.easysw.com/~mike/serial/serial.html

import errno
import fcntl
import os
import select
import struct
import sys
import termios
import time

import serial
from serial.serialutil import SerialBase, SerialException, to_bytes, portNotOpenError, writeTimeoutError


class PlatformSpecificBase(object):
    BAUDRATE_CONSTANTS = {}

    def number_to_device(self, port_number):
        sys.stderr.write("""\
don't know how to number ttys on this system.
! Use an explicit path (eg /dev/ttyS1) or send this information to
! the author of this module:

sys.platform = %r
os.name = %r
serialposix.py version = %s

also add the device name of the serial port and where the
counting starts for the first serial port.
e.g. 'first serial port: /dev/ttyS0'
and with a bit luck you can get this module running...
""" % (sys.platform, os.name, serial.VERSION))
        raise NotImplementedError('no number-to-device mapping defined on this platform')

    def _set_special_baudrate(self, baudrate):
        raise NotImplementedError('non-standard baudrates are not supported on this platform')

    def _set_rs485_mode(self, rs485_settings):
        raise NotImplementedError('RS485 not supported on this platform')

# try to detect the OS so that a device can be selected...
# this code block should supply a device() and set_special_baudrate() function
# for the platform
plat = sys.platform.lower()

if plat[:5] == 'linux':    # Linux (confirmed)
    import array

    # baudrate ioctls
    TCGETS2 = 0x802C542A
    TCSETS2 = 0x402C542B
    BOTHER = 0o010000

    # RS485 ioctls
    TIOCGRS485 = 0x542E
    TIOCSRS485 = 0x542F
    SER_RS485_ENABLED = 0b00000001
    SER_RS485_RTS_ON_SEND = 0b00000010
    SER_RS485_RTS_AFTER_SEND = 0b00000100
    SER_RS485_RX_DURING_TX = 0b00010000

    class PlatformSpecific(PlatformSpecificBase):
        BAUDRATE_CONSTANTS = {
            0:       0o000000,  # hang up
            50:      0o000001,
            75:      0o000002,
            110:     0o000003,
            134:     0o000004,
            150:     0o000005,
            200:     0o000006,
            300:     0o000007,
            600:     0o000010,
            1200:    0o000011,
            1800:    0o000012,
            2400:    0o000013,
            4800:    0o000014,
            9600:    0o000015,
            19200:   0o000016,
            38400:   0o000017,
            57600:   0o010001,
            115200:  0o010002,
            230400:  0o010003,
            460800:  0o010004,
            500000:  0o010005,
            576000:  0o010006,
            921600:  0o010007,
            1000000: 0o010010,
            1152000: 0o010011,
            1500000: 0o010012,
            2000000: 0o010013,
            2500000: 0o010014,
            3000000: 0o010015,
            3500000: 0o010016,
            4000000: 0o010017
        }

        def number_to_device(self, port_number):
            return '/dev/ttyS%d' % (port_number,)

        def _set_special_baudrate(self, baudrate):
            # right size is 44 on x86_64, allow for some growth
            buf = array.array('i', [0] * 64)
            try:
                # get serial_struct
                fcntl.ioctl(self.fd, TCGETS2, buf)
                # set custom speed
                buf[2] &= ~termios.CBAUD
                buf[2] |= BOTHER
                buf[9] = buf[10] = baudrate

                # set serial_struct
                fcntl.ioctl(self.fd, TCSETS2, buf)
            except IOError as e:
                raise ValueError('Failed to set custom baud rate (%s): %s' % (baudrate, e))

        def _set_rs485_mode(self, rs485_settings):
            buf = array.array('i', [0] * 8)  # flags, delaytx, delayrx, padding
            try:
                fcntl.ioctl(self.fd, TIOCGRS485, buf)
                if rs485_settings is not None:
                    if rs485_settings.loopback:
                        buf[0] |= SER_RS485_RX_DURING_TX
                    else:
                        buf[0] &= ~SER_RS485_RX_DURING_TX
                    if rs485_settings.rts_level_for_tx:
                        buf[0] |= SER_RS485_RTS_ON_SEND
                    else:
                        buf[0] &= ~SER_RS485_RTS_ON_SEND
                    if rs485_settings.rts_level_for_rx:
                        buf[0] |= SER_RS485_RTS_AFTER_SEND
                    else:
                        buf[0] &= ~SER_RS485_RTS_AFTER_SEND
                    buf[1] = int(rs485_settings.delay_rts_before_send * 1000)
                    buf[2] = int(rs485_settings.delay_rts_after_send * 1000)
                else:
                    buf[0] = 0  # clear SER_RS485_ENABLED
                fcntl.ioctl(self.fd, TIOCSRS485, buf)
            except IOError as e:
                raise ValueError('Failed to set RS485 mode: %s' % (e,))


elif plat == 'cygwin':       # cygwin/win32 (confirmed)

    class PlatformSpecific(PlatformSpecificBase):
        BAUDRATE_CONSTANTS = {
            128000: 0x01003,
            256000: 0x01005,
            500000: 0x01007,
            576000: 0x01008,
            921600: 0x01009,
            1000000: 0x0100a,
            1152000: 0x0100b,
            1500000: 0x0100c,
            2000000: 0x0100d,
            2500000: 0x0100e,
            3000000: 0x0100f
        }

        def number_to_device(self, port_number):
            return '/dev/com%d' % (port_number + 1,)


elif plat[:7] == 'openbsd':    # OpenBSD
    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/cua%02d' % (port_number,)

elif plat[:3] == 'bsd' or plat[:7] == 'freebsd':
    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/cuad%d' % (port_number,)

elif plat[:6] == 'darwin':   # OS X
    import array
    IOSSIOSPEED = 0x80045402  # _IOW('T', 2, speed_t)

    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/cuad%d' % (port_number,)

        osx_version = os.uname()[2].split('.')
        # Tiger or above can support arbitrary serial speeds
        if int(osx_version[0]) >= 8:
            def _set_special_baudrate(self, baudrate):
                # use IOKit-specific call to set up high speeds
                buf = array.array('i', [baudrate])
                fcntl.ioctl(self.fd, IOSSIOSPEED, buf, 1)


elif plat[:6] == 'netbsd':   # NetBSD 1.6 testing by Erk
    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/dty%02d' % (port_number,)

elif plat[:4] == 'irix':     # IRIX (partially tested)
    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/ttyf%d' % (port_number + 1,)  # XXX different device names depending on flow control

elif plat[:2] == 'hp':       # HP-UX (not tested)
    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/tty%dp0' % (port_number + 1,)

elif plat[:5] == 'sunos':    # Solaris/SunOS (confirmed)
    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/tty%c' % (ord('a') + port_number,)

elif plat[:3] == 'aix':      # AIX
    class PlatformSpecific(PlatformSpecificBase):
        def number_to_device(self, port_number):
            return '/dev/tty%d' % (port_number,)

else:
    class PlatformSpecific(PlatformSpecificBase):
        pass

# whats up with "aix", "beos", ....
# they should work, just need to know the device names.


# load some constants for later use.
# try to use values from termios, use defaults from linux otherwise
TIOCMGET = getattr(termios, 'TIOCMGET', 0x5415)
TIOCMBIS = getattr(termios, 'TIOCMBIS', 0x5416)
TIOCMBIC = getattr(termios, 'TIOCMBIC', 0x5417)
TIOCMSET = getattr(termios, 'TIOCMSET', 0x5418)

# TIOCM_LE = getattr(termios, 'TIOCM_LE', 0x001)
TIOCM_DTR = getattr(termios, 'TIOCM_DTR', 0x002)
TIOCM_RTS = getattr(termios, 'TIOCM_RTS', 0x004)
# TIOCM_ST = getattr(termios, 'TIOCM_ST', 0x008)
# TIOCM_SR = getattr(termios, 'TIOCM_SR', 0x010)

TIOCM_CTS = getattr(termios, 'TIOCM_CTS', 0x020)
TIOCM_CAR = getattr(termios, 'TIOCM_CAR', 0x040)
TIOCM_RNG = getattr(termios, 'TIOCM_RNG', 0x080)
TIOCM_DSR = getattr(termios, 'TIOCM_DSR', 0x100)
TIOCM_CD = getattr(termios, 'TIOCM_CD', TIOCM_CAR)
TIOCM_RI = getattr(termios, 'TIOCM_RI', TIOCM_RNG)
# TIOCM_OUT1 = getattr(termios, 'TIOCM_OUT1', 0x2000)
# TIOCM_OUT2 = getattr(termios, 'TIOCM_OUT2', 0x4000)
if hasattr(termios, 'TIOCINQ'):
    TIOCINQ = termios.TIOCINQ
else:
    TIOCINQ = getattr(termios, 'FIONREAD', 0x541B)
TIOCOUTQ = getattr(termios, 'TIOCOUTQ', 0x5411)

TIOCM_zero_str = struct.pack('I', 0)
TIOCM_RTS_str = struct.pack('I', TIOCM_RTS)
TIOCM_DTR_str = struct.pack('I', TIOCM_DTR)

TIOCSBRK = getattr(termios, 'TIOCSBRK', 0x5427)
TIOCCBRK = getattr(termios, 'TIOCCBRK', 0x5428)

CMSPAR = 0o10000000000  # Use "stick" (mark/space) parity


class Serial(SerialBase, PlatformSpecific):
    """\
    Serial port class POSIX implementation. Serial port configuration is
    done with termios and fcntl. Runs on Linux and many other Un*x like
    systems.
    """

    def open(self):
        """\
        Open port with current settings. This may throw a SerialException
        if the port cannot be opened."""
        if self._port is None:
            raise SerialException("Port must be configured before it can be used.")
        if self.is_open:
            raise SerialException("Port is already open.")
        self.fd = None
        # open
        try:
            self.fd = os.open(self.portstr, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as msg:
            self.fd = None
            raise SerialException(msg.errno, "could not open port %s: %s" % (self._port, msg))
        #~ fcntl.fcntl(self.fd, fcntl.F_SETFL, 0)  # set blocking

        try:
            self._reconfigure_port(force_update=True)
        except:
            try:
                os.close(self.fd)
            except:
                # ignore any exception when closing the port
                # also to keep original exception that happened when setting up
                pass
            self.fd = None
            raise
        else:
            self.is_open = True
        if not self._dsrdtr:
            self._update_dtr_state()
        if not self._rtscts:
            self._update_rts_state()
        self.reset_input_buffer()

    def _reconfigure_port(self, force_update=False):
        """Set communication parameters on opened port."""
        if self.fd is None:
            raise SerialException("Can only operate on a valid file descriptor")
        custom_baud = None

        vmin = vtime = 0                # timeout is done via select
        if self._inter_byte_timeout is not None:
            vmin = 1
            vtime = int(self._inter_byte_timeout * 10)
        try:
            orig_attr = termios.tcgetattr(self.fd)
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = orig_attr
        except termios.error as msg:      # if a port is nonexistent but has a /dev file, it'll fail here
            raise SerialException("Could not configure port: %s" % msg)
        # set up raw mode / no echo / binary
        cflag |= (termios.CLOCAL | termios.CREAD)
        lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE |
                   termios.ECHOK | termios.ECHONL |
                   termios.ISIG | termios.IEXTEN)  # |termios.ECHOPRT
        for flag in ('ECHOCTL', 'ECHOKE'):  # netbsd workaround for Erk
            if hasattr(termios, flag):
                lflag &= ~getattr(termios, flag)

        oflag &= ~(termios.OPOST | termios.ONLCR | termios.OCRNL)
        iflag &= ~(termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IGNBRK)
        if hasattr(termios, 'IUCLC'):
            iflag &= ~termios.IUCLC
        if hasattr(termios, 'PARMRK'):
            iflag &= ~termios.PARMRK

        # setup baud rate
        try:
            ispeed = ospeed = getattr(termios, 'B%s' % (self._baudrate))
        except AttributeError:
            try:
                ispeed = ospeed = self.BAUDRATE_CONSTANTS[self._baudrate]
            except KeyError:
                #~ raise ValueError('Invalid baud rate: %r' % self._baudrate)
                # may need custom baud rate, it isn't in our list.
                ispeed = ospeed = getattr(termios, 'B38400')
                try:
                    custom_baud = int(self._baudrate)  # store for later
                except ValueError:
                    raise ValueError('Invalid baud rate: %r' % self._baudrate)
                else:
                    if custom_baud < 0:
                        raise ValueError('Invalid baud rate: %r' % self._baudrate)

        # setup char len
        cflag &= ~termios.CSIZE
        if self._bytesize == 8:
            cflag |= termios.CS8
        elif self._bytesize == 7:
            cflag |= termios.CS7
        elif self._bytesize == 6:
            cflag |= termios.CS6
        elif self._bytesize == 5:
            cflag |= termios.CS5
        else:
            raise ValueError('Invalid char len: %r' % self._bytesize)
        # setup stop bits
        if self._stopbits == serial.STOPBITS_ONE:
            cflag &= ~(termios.CSTOPB)
        elif self._stopbits == serial.STOPBITS_ONE_POINT_FIVE:
            cflag |= (termios.CSTOPB)  # XXX same as TWO.. there is no POSIX support for 1.5
        elif self._stopbits == serial.STOPBITS_TWO:
            cflag |= (termios.CSTOPB)
        else:
            raise ValueError('Invalid stop bit specification: %r' % self._stopbits)
        # setup parity
        iflag &= ~(termios.INPCK | termios.ISTRIP)
        if self._parity == serial.PARITY_NONE:
            cflag &= ~(termios.PARENB | termios.PARODD)
        elif self._parity == serial.PARITY_EVEN:
            cflag &= ~(termios.PARODD)
            cflag |= (termios.PARENB)
        elif self._parity == serial.PARITY_ODD:
            cflag |= (termios.PARENB | termios.PARODD)
        elif self._parity == serial.PARITY_MARK and plat[:5] == 'linux':
            cflag |= (termios.PARENB | CMSPAR | termios.PARODD)
        elif self._parity == serial.PARITY_SPACE and plat[:5] == 'linux':
            cflag |= (termios.PARENB | CMSPAR)
            cflag &= ~(termios.PARODD)
        else:
            raise ValueError('Invalid parity: %r' % self._parity)
        # setup flow control
        # xonxoff
        if hasattr(termios, 'IXANY'):
            if self._xonxoff:
                iflag |= (termios.IXON | termios.IXOFF)  # |termios.IXANY)
            else:
                iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
        else:
            if self._xonxoff:
                iflag |= (termios.IXON | termios.IXOFF)
            else:
                iflag &= ~(termios.IXON | termios.IXOFF)
        # rtscts
        if hasattr(termios, 'CRTSCTS'):
            if self._rtscts:
                cflag |= (termios.CRTSCTS)
            else:
                cflag &= ~(termios.CRTSCTS)
        elif hasattr(termios, 'CNEW_RTSCTS'):   # try it with alternate constant name
            if self._rtscts:
                cflag |= (termios.CNEW_RTSCTS)
            else:
                cflag &= ~(termios.CNEW_RTSCTS)
        # XXX should there be a warning if setting up rtscts (and xonxoff etc) fails??

        # buffer
        # vmin "minimal number of characters to be read. 0 for non blocking"
        if vmin < 0 or vmin > 255:
            raise ValueError('Invalid vmin: %r ' % vmin)
        cc[termios.VMIN] = vmin
        # vtime
        if vtime < 0 or vtime > 255:
            raise ValueError('Invalid vtime: %r' % vtime)
        cc[termios.VTIME] = vtime
        # activate settings
        if force_update or [iflag, oflag, cflag, lflag, ispeed, ospeed, cc] != orig_attr:
            termios.tcsetattr(
                    self.fd,
                    termios.TCSANOW,
                    [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

        # apply custom baud rate, if any
        if custom_baud is not None:
            self._set_special_baudrate(custom_baud)

        if self._rs485_mode is not None:
            self._set_rs485_mode(self._rs485_mode)

    def close(self):
        """Close port"""
        if self.is_open:
            if self.fd is not None:
                os.close(self.fd)
                self.fd = None
            self.is_open = False

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    @property
    def in_waiting(self):
        """Return the number of bytes currently in the input buffer."""
        #~ s = fcntl.ioctl(self.fd, termios.FIONREAD, TIOCM_zero_str)
        s = fcntl.ioctl(self.fd, TIOCINQ, TIOCM_zero_str)
        return struct.unpack('I', s)[0]

    # select based implementation, proved to work on many systems
    def read(self, size=1):
        """\
        Read size bytes from the serial port. If a timeout is set it may
        return less characters as requested. With no timeout it will block
        until the requested number of bytes is read.
        """
        if not self.is_open:
            raise portNotOpenError
        read = bytearray()
        timeout = self._timeout
        while len(read) < size:
            try:
                start_time = time.time()
                ready, _, _ = select.select([self.fd], [], [], timeout)
                # If select was used with a timeout, and the timeout occurs, it
                # returns with empty lists -> thus abort read operation.
                # For timeout == 0 (non-blocking operation) also abort when
                # there is nothing to read.
                if not ready:
                    break   # timeout
                buf = os.read(self.fd, size - len(read))
                # read should always return some data as select reported it was
                # ready to read when we get to this point.
                if not buf:
                    # Disconnected devices, at least on Linux, show the
                    # behavior that they are always ready to read immediately
                    # but reading returns nothing.
                    raise SerialException('device reports readiness to read but returned no data (device disconnected or multiple access on port?)')
                read.extend(buf)
                if timeout is not None:
                    timeout -= time.time() - start_time
                    if timeout <= 0:
                        break
            except OSError as e:
                # this is for Python 3.x where select.error is a subclass of
                # OSError ignore EAGAIN errors. all other errors are shown
                if e.errno != errno.EAGAIN:
                    raise SerialException('read failed: %s' % (e,))
            except select.error as e:
                # this is for Python 2.x
                # ignore EAGAIN errors. all other errors are shown
                # see also http://www.python.org/dev/peps/pep-3151/#select
                if e[0] != errno.EAGAIN:
                    raise SerialException('read failed: %s' % (e,))
        return bytes(read)

    def write(self, data):
        """Output the given byte string over the serial port."""
        if not self.is_open:
            raise portNotOpenError
        d = to_bytes(data)
        tx_len = len(d)
        if self._write_timeout is not None and self._write_timeout > 0:
            timeout = time.time() + self._write_timeout
        else:
            timeout = None
        while tx_len > 0:
            try:
                n = os.write(self.fd, d)
                if timeout:
                    # when timeout is set, use select to wait for being ready
                    # with the time left as timeout
                    timeleft = timeout - time.time()
                    if timeleft < 0:
                        raise writeTimeoutError
                    _, ready, _ = select.select([], [self.fd], [], timeleft)
                    if not ready:
                        raise writeTimeoutError
                else:
                    # wait for write operation
                    _, ready, _ = select.select([], [self.fd], [], None)
                    if not ready:
                        raise SerialException('write failed (select)')
                d = d[n:]
                tx_len -= n
            except SerialException:
                raise
            except OSError as v:
                if v.errno != errno.EAGAIN:
                    raise SerialException('write failed: %s' % (v,))
                # still calculate and check timeout
                if timeout and timeout - time.time() < 0:
                    raise writeTimeoutError
        return len(data)

    def flush(self):
        """\
        Flush of file like objects. In this case, wait until all data
        is written.
        """
        if not self.is_open:
            raise portNotOpenError
        termios.tcdrain(self.fd)

    def reset_input_buffer(self):
        """Clear input buffer, discarding all that is in the buffer."""
        if not self.is_open:
            raise portNotOpenError
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def reset_output_buffer(self):
        """\
        Clear output buffer, aborting the current output and discarding all
        that is in the buffer.
        """
        if not self.is_open:
            raise portNotOpenError
        termios.tcflush(self.fd, termios.TCOFLUSH)

    def send_break(self, duration=0.25):
        """\
        Send break condition. Timed, returns to idle state after given
        duration.
        """
        if not self.is_open:
            raise portNotOpenError
        termios.tcsendbreak(self.fd, int(duration / 0.25))

    def _update_break_state(self):
        """\
        Set break: Controls TXD. When active, no transmitting is possible.
        """
        if self._break_state:
            fcntl.ioctl(self.fd, TIOCSBRK)
        else:
            fcntl.ioctl(self.fd, TIOCCBRK)

    def _update_rts_state(self):
        """Set terminal status line: Request To Send"""
        if self._rts_state:
            fcntl.ioctl(self.fd, TIOCMBIS, TIOCM_RTS_str)
        else:
            fcntl.ioctl(self.fd, TIOCMBIC, TIOCM_RTS_str)

    def _update_dtr_state(self):
        """Set terminal status line: Data Terminal Ready"""
        if self._dtr_state:
            fcntl.ioctl(self.fd, TIOCMBIS, TIOCM_DTR_str)
        else:
            fcntl.ioctl(self.fd, TIOCMBIC, TIOCM_DTR_str)

    @property
    def cts(self):
        """Read terminal status line: Clear To Send"""
        if not self.is_open:
            raise portNotOpenError
        s = fcntl.ioctl(self.fd, TIOCMGET, TIOCM_zero_str)
        return struct.unpack('I', s)[0] & TIOCM_CTS != 0

    @property
    def dsr(self):
        """Read terminal status line: Data Set Ready"""
        if not self.is_open:
            raise portNotOpenError
        s = fcntl.ioctl(self.fd, TIOCMGET, TIOCM_zero_str)
        return struct.unpack('I', s)[0] & TIOCM_DSR != 0

    @property
    def ri(self):
        """Read terminal status line: Ring Indicator"""
        if not self.is_open:
            raise portNotOpenError
        s = fcntl.ioctl(self.fd, TIOCMGET, TIOCM_zero_str)
        return struct.unpack('I', s)[0] & TIOCM_RI != 0

    @property
    def cd(self):
        """Read terminal status line: Carrier Detect"""
        if not self.is_open:
            raise portNotOpenError
        s = fcntl.ioctl(self.fd, TIOCMGET, TIOCM_zero_str)
        return struct.unpack('I', s)[0] & TIOCM_CD != 0

    # - - platform specific - - - -

    @property
    def out_waiting(self):
        """Return the number of bytes currently in the output buffer."""
        #~ s = fcntl.ioctl(self.fd, termios.FIONREAD, TIOCM_zero_str)
        s = fcntl.ioctl(self.fd, TIOCOUTQ, TIOCM_zero_str)
        return struct.unpack('I', s)[0]

    def nonblocking(self):
        """internal - not portable!"""
        if not self.is_open:
            raise portNotOpenError
        fcntl.fcntl(self.fd, fcntl.F_SETFL, os.O_NONBLOCK)

    def fileno(self):
        """\
        For easier use of the serial port instance with select.
        WARNING: this function is not portable to different platforms!
        """
        if not self.is_open:
            raise portNotOpenError
        return self.fd

    def set_input_flow_control(self, enable=True):
        """\
        Manually control flow - when software flow control is enabled.
        This will send XON (true) or XOFF (false) to the other device.
        WARNING: this function is not portable to different platforms!
        """
        if not self.is_open:
            raise portNotOpenError
        if enable:
            termios.tcflow(self.fd, termios.TCION)
        else:
            termios.tcflow(self.fd, termios.TCIOFF)

    def set_output_flow_control(self, enable=True):
        """\
        Manually control flow of outgoing data - when hardware or software flow
        control is enabled.
        WARNING: this function is not portable to different platforms!
        """
        if not self.is_open:
            raise portNotOpenError
        if enable:
            termios.tcflow(self.fd, termios.TCOON)
        else:
            termios.tcflow(self.fd, termios.TCOOFF)


class PosixPollSerial(Serial):
    """\
    Poll based read implementation. Not all systems support poll properly.
    However this one has better handling of errors, such as a device
    disconnecting while it's in use (e.g. USB-serial unplugged).
    """

    def read(self, size=1):
        """\
        Read size bytes from the serial port. If a timeout is set it may
        return less characters as requested. With no timeout it will block
        until the requested number of bytes is read.
        """
        if self.fd is None:
            raise portNotOpenError
        read = bytearray()
        poll = select.poll()
        poll.register(self.fd, select.POLLIN | select.POLLERR | select.POLLHUP | select.POLLNVAL)
        if size > 0:
            while len(read) < size:
                # print "\tread(): size",size, "have", len(read)    #debug
                # wait until device becomes ready to read (or something fails)
                for fd, event in poll.poll(self._timeout * 1000):
                    if event & (select.POLLERR | select.POLLHUP | select.POLLNVAL):
                        raise SerialException('device reports error (poll)')
                    #  we don't care if it is select.POLLIN or timeout, that's
                    #  handled below
                buf = os.read(self.fd, size - len(read))
                read.extend(buf)
                if ((self._timeout is not None and self._timeout >= 0) or
                        (self._inter_byte_timeout is not None and self._inter_byte_timeout > 0)) and not buf:
                    break   # early abort on timeout
        return bytes(read)


class VTIMESerial(Serial):
    """\
    Implement timeout using vtime of tty device instead of using select.
    This means that no inter character timeout can be specified and that
    the error handling is degraded.

    Overall timeout is disabled when inter-character timeout is used.
    """

    def _reconfigure_port(self, force_update=True):
        """Set communication parameters on opened port."""
        super(VTIMESerial, self)._reconfigure_port()
        fcntl.fcntl(self.fd, fcntl.F_SETFL, 0) # clear O_NONBLOCK

        if self._inter_byte_timeout is not None:
            vmin = 1
            vtime = int(self._inter_byte_timeout * 10)
        else:
            vmin = 0
            vtime = int(self._timeout * 10)
        try:
            orig_attr = termios.tcgetattr(self.fd)
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = orig_attr
        except termios.error as msg:      # if a port is nonexistent but has a /dev file, it'll fail here
            raise serial.SerialException("Could not configure port: %s" % msg)

        if vtime < 0 or vtime > 255:
            raise ValueError('Invalid vtime: %r' % vtime)
        cc[termios.VTIME] = vtime
        cc[termios.VMIN] = vmin

        termios.tcsetattr(
                self.fd,
                termios.TCSANOW,
                [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])


    def read(self, size=1):
        """\
        Read size bytes from the serial port. If a timeout is set it may
        return less characters as requested. With no timeout it will block
        until the requested number of bytes is read.
        """
        if not self.is_open:
            raise portNotOpenError
        read = bytearray()
        while len(read) < size:
            buf = os.read(self.fd, size - len(read))
            if not buf:
                break
            read.extend(buf)
        return bytes(read)


if __name__ == '__main__':
    s = Serial(0,
               baudrate=19200,         # baud rate
               bytesize=serial.EIGHTBITS,     # number of data bits
               parity=serial.PARITY_EVEN,     # enable parity checking
               stopbits=serial.STOPBITS_ONE,  # number of stop bits
               timeout=3,              # set a timeout value, None for waiting forever
               xonxoff=0,              # enable software flow control
               rtscts=0,               # enable RTS/CTS flow control
               )
    s.rts = True
    s.dtr = True
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.write('hello')
    sys.stdout.write('%r\n' % s.read(5))
    sys.stdout.write('%s\n' % s.inWaiting())
    del s