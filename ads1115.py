from micropython import const

#Addressing(ADDR_pin)
ADDR_GND = const(0b1001000)
ADDR_VDD = const(0b1001001)
ADDR_SDA = const(0b1001010)
ADDR_SCL = const(0b1001011)

#Operational status or single-shot conversion start
_OS_0 = const(0b0) #W: No effect | R: Device is currently performing a conversion
_OS_1 = const(0b1) #W: Start a single conversion | R: Device is not currently performing a conversion

#Multiplexer(MUX_INp_INn)
MUX_A0_A1  = const(0b000) #default
MUX_A0_A3  = const(0b001)
MUX_A1_A3  = const(0b010)
MUX_A2_A3  = const(0b011)
MUX_A0_GND = const(0b100)
MUX_A1_GND = const(0b101)
MUX_A2_GND = const(0b110)
MUX_A3_GND = const(0b111)

#Programming gain amplifier(PGA_v_mv)
PGA_6_144 = const((0b000, 6.144))
PGA_4_096 = const((0b001, 4.096))
PGA_2_048 = const((0b010, 2.048)) #default
PGA_1_024 = const((0b011, 1.024))
PGA_0_512 = const((0b100, 0.512))
PGA_0_256 = const((0b101, 0.256))

#Device operating mode
_MODE_CONTINUOUS   = const(0b0)
_MODE_SINGLE_SHOT  = const(0b1) #default

#Data rate(DR_SPS)
DR_8   = const(0b000)
DR_16  = const(0b001)
DR_32  = const(0b010)
DR_64  = const(0b011)
DR_128 = const(0b100) #default
DR_250 = const(0b101)
DR_475 = const(0b110)
DR_860 = const(0b111)

#Comparator mode
COMP_MODE_TRADITIONAL = const(0b0) #default
COMP_MODE_WINDOW      = const(0b1)

#Comparator polarity
COMP_POL_LOW  = const(0b0) #default
COMP_POL_HIGH = const(0b1)

#Latching comparator(COMP_LAT_state)
COMP_LAT_OFF = const(0b0) #default
COMP_LAT_ON  = const(0b1)

#Comparator queue and disable(COMP_QUE_mode)
COMP_QUE_AFTER_ONE_CONVERSION  = const(0b00)
COMP_QUE_AFTER_TWO_CONVERSION  = const(0b01)
COMP_QUE_AFTER_FOUR_CONVERSION = const(0b10)
COMP_QUE_OFF                   = const(0b11) #default



#Register address pointer
_ADDR_POINT_CONVERSION = const(0b00)
_ADDR_POINT_CONFIG     = const(0b01)
_ADDR_POINT_LO_THRESH  = const(0b10)
_ADDR_POINT_HI_THRESH  = const(0b11)

class ThreshOutOfRange(Exception):
    def __init__(self, val, pga_range):
        super().__init__("The argument value is out of PGA range. Argument: {}. PGA range: +-{}".format(val, pga_range))

class ADS1115:
    def __init__(self, i2c,
                 addr=ADDR_GND,
                 mux=MUX_A0_A1,
                 pga=PGA_2_048,
                 data_rate=DR_128,
                 comp_mode=COMP_MODE_TRADITIONAL,
                 comp_pol=COMP_POL_LOW,
                 comp_lat=COMP_LAT_OFF,
                 comp_que=COMP_QUE_OFF):
        self.i2c = i2c
        self.init(addr=addr,
                  mux=mux,
                  pga=pga,
                  data_rate=data_rate,
                  comp_mode=comp_mode,
                  comp_pol=comp_pol,
                  comp_lat=comp_lat,
                  comp_que=comp_que)
    
    def init(self, addr=ADDR_GND,
                 mux=MUX_A0_A1,
                 pga=PGA_2_048,
                 data_rate=DR_128,
                 comp_mode=COMP_MODE_TRADITIONAL,
                 comp_pol=COMP_POL_LOW,
                 comp_lat=COMP_LAT_OFF,
                 comp_que=COMP_QUE_OFF):
        self.addr = addr
        self.mux = mux
        self.pga = pga
        self.data_rate = data_rate
        self.comp_mode = comp_mode
        self.comp_pol = comp_pol
        self.comp_lat = comp_lat
        self.comp_que = comp_que
    
    def start_conversion(self):
        self._config(_OS_1, self.mux, self.pga, _MODE_CONTINUOUS, self.data_rate, self.comp_mode, self.comp_pol, self.comp_lat, self.comp_que)
        
    def one_conversion(self):
        self._config(_OS_1, self.mux, self.pga, _MODE_SINGLE_SHOT, self.data_rate, self.comp_mode, self.comp_pol, self.comp_lat, self.comp_que)
    
    def end_conversion(self):
        self.one_conversion()
    
    def read_v(self):
        val = self._read()
        if val >= 0x8000: val -= 0xFFFF
        return val * self.pga[1] / 2**15
    
    def read_mv(self):
        return self.read_v() * 1000
    
    def set_lo_thresh_v(self, val):
        self._set_thresh(_ADDR_POINT_LO_THRESH, val)
        
    def set_hi_thresh_v(self, val):
        self._set_thresh(_ADDR_POINT_HI_THRESH, val)
        
    def set_lo_thresh_mv(self, val):
        self._set_thresh(_ADDR_POINT_LO_THRESH, val / 1000)
        
    def set_hi_thresh_mv(self, val):
        self._set_thresh(_ADDR_POINT_HI_THRESH, val / 1000)
    
    def _config(self, os, mux, pga, mode, dr, cm, cp, cl, cq):
        cmd = bytearray(3)
        cmd[0] = _ADDR_POINT_CONFIG
        cmd[1] = (os << 7) | (mux << 4) | (pga[0] << 1) | mode
        cmd[2] = (dr << 5) | (cm << 4) | (cp << 3) | (cl << 2) | cq
        self._write(cmd)
        self._write([_ADDR_POINT_CONVERSION])
    
    def _read(self):
        data = self.i2c.readfrom(self.addr, 2)
        return (data[0] << 8) | data[1]
    
    def _write(self, cmd):
        self.i2c.writeto(self.addr, bytearray(cmd))
    
    def _set_thresh(self, thresh, val):
        if val > self.pga[1] or val < -self.pga[1]: raise ThreshOutOfRange(val, self.pga[1])
        b = int(val * (2**15 - 1) / self.pga[1])
        if b < 0: b += 0xFFFF
        cmd = bytearray(3)
        cmd[0] = thresh
        cmd[1] = b >> 8
        cmd[2] = b & 0x00FF
        self._write(cmd)