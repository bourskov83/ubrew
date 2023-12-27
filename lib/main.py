#imports

import uasyncio
from microdot_asyncio import Microdot, send_file, Response
import machine, onewire, ds18x20
import binascii
import ujson
from mqtt_as import MQTTClient
from mqtt_as import config as mqtt_config
from PID import PID
import tm1637


class Vessel:
    def __init__(self, outputPin, ds18x20, sensorRom, mqttClient, sensorOffset: float = 0, kP=0,kI=0,kD=0):
        # Output definitions
        self.pwm = machine.PWM(machine.Pin(outputPin))
        self.pwm.freq(1)

        # Sensors
        self.ds18x20 = ds18x20
        self.sensorRom = sensorRom
        self.sensorOffset = sensorOffset

        # PID init
        self.pid = PID(kP, kI, kD, scale='ms')
        self._setPoint: float = 0
        self.currentTemperature = 0
        self.pid.output_limits = (0,1023) # match PWM duty range


        # MQTT
        self.mqtt = mqttClient


    def read_temperature(self):
        self.currentTemperature = self.ds18x20.read_temp(bytes.fromhex(self.sensorRom)) + self.sensorOffset
        return self.currentTemperature
    

    def control_update(self):
        output = self.pid(self.currentTemperature)
        self.pwm.duty(int(output))

        
    @property
    def setpoint(self):
        return self._setPoint

    @setpoint.setter
    def setpoint(self, value):
        self._setPoint = value
        self.pid.setpoint = value
    
    def pid_auto_mode(self, auto_mode: bool = True, last_output: float = 0):
        if auto_mode:
            self.pid.set_auto_mode(True, last_output=last_output)
        else:
            self.pid.auto_mode = False




# Pin definitions
ONEWIRE_PIN = 32
HEATING_PIN = 4
displayPinClk = 14
displayPinDio = 27

# Initialize Watchdog
wdt = machine.WDT(timeout=5000)



with open('config.json') as f:
    config = ujson.loads(f.read())
with open('setpoint.json') as f:
    setpoint = ujson.loads(f.read())


#heating = machine.Pin(HEATING_PIN, machine.Pin.OUT)
#heating = machine.PWM(HEATING_PIN)
#heating.freq(1)

displayMlt = tm1637.TM1637(clk=machine.Pin(displayPinClk),dio=machine.Pin(displayPinDio))

    
#mqtt_config['server'] = config['wifi']['mqtt_server']
#mqtt_config['ssid'] = config['wifi']['ssid']
#mqtt_config['wifi_pw'] = config['wifi']['wifi_pw']


#MQTTClient.DEBUG = True # type: ignore
#mqttclient = MQTTClient(mqtt_config)


owPin = machine.Pin(ONEWIRE_PIN)
ds_sensors = ds18x20.DS18X20(onewire.OneWire(owPin))
ds_roms = ds_sensors.scan()
#temp_sensors = config['sensors']
#for i in temp_sensors:
#    i['value'] = "NA"

#print(temp_sensors)


mlt = Vessel(4,ds_sensors,config['sensors']['mlt']['rom'],None,config['sensors']['mlt']['offset'])
mlt.setpoint = 40


#mltPid = PID(1,3,0.2, setpoint=50, scale='ms')
#mltPid.output_limits=(0,1023)
#mltPid()

#async def mqtt_connect(client):
#    await client.connect()

#    await client.publish("fermcontrol/ipaddr", str(client._sta_if.ifconfig()[0]),qos=1)


async def temp_sensor_read():
    while True:
        ds_sensors.convert_temp()
        await uasyncio.sleep_ms(800)
        
 #       for sensor in temp_sensors:
 #           try:
 #               sensor['value'] = ds_sensors.read_temp(bytes.fromhex(sensor['rom'])) + sensor['offset']
 #           except Exception as E:
 #               print(f"Error reading {sensor['name']} Exception: {E}")
 #               sensor['value'] = "NA"
             
        
#async def mqtt_update(client):
#    while True:
#        await uasyncio.sleep(10)

#        for i in temp_sensors:
#            if i['value'] != "NA":
#                await client.publish(f"fermcontrol/{i['name']}/temperature/state",str(round(i['value'],1)),qos=1)
#        await client.publish("fermcontrol/cooling/state",str(cooling.value()),qos=1)
#        await client.publish("fermcontrol/heating/state",str(heating.value()),qos=1)
#        await client.publish("fermcontrol/chambersetpoint/state",setpoint['chamberSetpoint'],qos=1)
#        await client.publish("fermcontrol/vesselsetpoint/state",setpoint['vesselSetpoint'],qos=1)
        
#
#
#app = Microdot()
#
#@app.route('/')
#async def home(request):
#    return send_file('/static/index.html')
#
#
#@app.route('/api/values', methods=['GET'])
#async def values(request):
#    payload = {item['name']: item['value'] for item in temp_sensors}
#    payload['coolingState'] = bool(cooling.value())
#    payload['heatingState'] = bool(heating.value())
#
#    return Response(headers={'Content-Type' : 'application/json','Access-Control-Allow-Origin':'*'}, body=ujson.dumps(payload))
#
#
#
#@app.route('/api/setpoint', methods=['GET'])
#async def getSetpoint(request):
#    if 'action' in request.args:
#        if request.args['action'] == 'update':
#            setpoint['chamberSetpoint'] = request.args['chamber']
#            setpoint['vesselSetpoint'] = request.args['vessel']
#            with open('setpoint.json', "w") as f:
#                ujson.dump(setpoint, f)
#
#    payload = setpoint
#    return Response(headers={'Content-Type' : 'application/json','Access-Control-Allow-Origin':'*'}, body=ujson.dumps(payload))
#
#
#async def start_server():
#    await app.start_server(port=80)
#
#
#
#async def showResult():
#    while True:
#        await uasyncio.sleep(1)
#        result = {'sensorReadings':temp_sensors, "cooling":cooling.value(), "heating":heating.value(), "setpoint":setpoint['chamberSetpoint']}
#        print(result)
async def updateOutput():
    while True:
        await uasyncio.sleep_ms(500)
        mlt.read_temperature()
        mlt.control_update()

        print(mlt.currentTemperature)
        print(mlt.pwm.duty())

        #print(temp_sensors)
        #print(f"MLT HEAT: {heating.value()}")
       # mlt_temp = next((sensor['value'] for sensor in temp_sensors if sensor['name'] == 'mlt'))
       # if mlt_temp != "NA":
       #     control=mltPid(mlt_temp)
       #     print(control)
       #     heating.duty(int(control))
       #     displayMlt.temperature(int(mlt_temp))

#        if mlt_temp <= 40:
#            heating.on()
#        else:
#            heating.off()


        

#        coolStart = int(setpoint['chamberSetpoint']) + int(config['cooling']['hysteresis']) / 2 
#        coolStop = int(setpoint['chamberSetpoint']) - int(config['cooling']['hysteresis']) / 2 + 0.3
#        heatStart = int(setpoint['chamberSetpoint']) - int(config['heating']['hysteresis']) / 2 
#        heatStop = int(setpoint['chamberSetpoint']) + int(config['heating']['hysteresis']) / 2 - 0.3
#        
#        chamber_value = next((sensor['value'] for sensor in temp_sensors if sensor['name'] == 'chamber'))
#        
#        if chamber_value >= coolStart:
#            if not cooling.value():
#                cooling.on()
#        elif chamber_value <= coolStop:
#            if cooling.value():
 #              cooling.off()
#
 #       if chamber_value <= heatStart:
 #           if not heating.value():
 #               heating.on()
 #       elif chamber_value >= heatStop:
 #           if heating.value():
 #              heating.off()

        wdt.feed()

loop = uasyncio.get_event_loop()    
loop.create_task(temp_sensor_read())
#loop.create_task(start_server())
#loop.create_task(mqtt_connect(mqttclient))
#loop.create_task(mqtt_update(mqttclient))
#loop.create_task(showResult())
loop.create_task(updateOutput())
loop.run_forever()