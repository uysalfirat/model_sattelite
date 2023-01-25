import sys
import time
import sys, time
import navio.adc
import navio.util
import navio.ms5611
import navio.ublox
import navio.pwm
import navio.rcinput

#receiver
navio.util.check_apm()
rcin = navio.rcinput.RCInput()

#gnss
if __name__ == "__main__":

    ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

    ubl.configure_poll_port()
    ubl.configure_poll(navio.ublox.CLASS_CFG, navio.ublox.MSG_CFG_USB)
    #ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)
    ubl.configure_port(port=navio.ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=navio.ublox.PORT_USB, inMask=1, outMask=1)
    ubl.configure_port(port=navio.ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_poll_port()
    ubl.configure_poll_port(navio.ublox.PORT_SERIAL1)
    ubl.configure_poll_port(navio.ublox.PORT_SERIAL2)
    ubl.configure_poll_port(navio.ublox.PORT_USB)
    ubl.configure_solution_rate(rate_ms=1000)
    ubl.set_preferred_dynamic_model(None)
    ubl.set_preferred_usePPP(None)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSLLH, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_PVT, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_STATUS, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SOL, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELNED, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SVINFO, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELECEF, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSECEF, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_RAW, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SFRB, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SVSI, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_ALM, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_EPH, 1)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_TIMEGPS, 5)
    ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_CLOCK, 5)
    #ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_DGPS, 5)

#adc
navio.util.check_apm()
adc = navio.adc.ADC()
results = [0] * adc.channel_count

#barometre
navio.util.check_apm()
baro = navio.ms5611.MS5611()
baro.initialize()

#servo
navio.util.check_apm()
PWM_OUTPUT = 0
SERVO_MIN = 1.250
SERVO_MAX = 1.750
with navio.pwm.PWM(PWM_OUTPUT) as pwm:
    pwm.set_period(50)
    pwm.enable()


while(True):
    #receiver
    period = rcin.read(2)
    print (period)
    time.sleep(1)

    #barometre
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()
	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()
	baro.calculatePressureAndTemperature()
	print ("Temperature(C): %.6f" % (baro.TEMP), "Pressure(millibar): %.6f" % (baro.PRES))
	time.sleep(1)

    #adc
    s = ''
    for i in range (0, adc.channel_count):
        results[i] = adc.read(i)
        s += 'A{0}: {1:6.4f}V '.format(i, results[i] / 1000)
    print(s)
    time.sleep(0.5)

    #gnss
    msg = ubl.receive_message()
        if msg is None:
            if opts.reopen:
                ubl.close()
                ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                continue
            print(empty)
            break
        #print(msg.name())
        if msg.name() == "NAV_POSLLH":
            outstr = str(msg).split(",")[1:]
            outstr = "".join(outstr)
            print(outstr)
        if msg.name() == "NAV_STATUS":
            outstr = str(msg).split(",")[1:2]
            outstr = "".join(outstr)
            print(outstr)
        #print(str(msg))

    #servo
    pwm.set_duty_cycle(SERVO_MIN)
	time.sleep(1)

