#imports

import uasyncio
from microdot_asyncio import Microdot, send_file, Response
import machine, onewire, ds18x20
import binascii
import ujson
from mqtt_as import MQTTClient
from mqtt_as import config as mqtt_config
from PID import PID



# Pin definitions
ONEWIRE_PIN = 32
COOLING_PIN = 19
HEATING_PIN = 2

# Initialize Watchdog
wdt = machine.WDT(timeout=5000)



with open('config.json') as f:
    config = ujson.loads(f.read())
with open('setpoint.json') as f:
    setpoint = ujson.loads(f.read())


cooling = machine.Pin(COOLING_PIN, machine.Pin.OUT)
heating = machine.Pin(HEATING_PIN, machine.Pin.OUT)

    
mqtt_config['server'] = config['wifi']['mqtt_server']
mqtt_config['ssid'] = config['wifi']['ssid']
mqtt_config['wifi_pw'] = config['wifi']['wifi_pw']


MQTTClient.DEBUG = True # type: ignore
mqttclient = MQTTClient(mqtt_config)


owPin = machine.Pin(ONEWIRE_PIN)
ds_sensors = ds18x20.DS18X20(onewire.OneWire(owPin))
ds_roms = ds_sensors.scan()
temp_sensors = config['sensors']
for i in temp_sensors:
    i['value'] = "NA"

print(temp_sensors)


async def mqtt_connect(client):
    await client.connect()

    await client.publish("fermcontrol/ipaddr", str(client._sta_if.ifconfig()[0]),qos=1)


async def temp_sensor_read():
    while True:
        ds_sensors.convert_temp()
        await uasyncio.sleep_ms(1000)
        
        for sensor in temp_sensors:
            try:
                sensor['value'] = ds_sensors.read_temp(bytes.fromhex(sensor['rom'])) + sensor['offset']
            except Exception as E:
                print(f"Error reading {sensor['name']} Exception: {E}")
                sensor['value'] = "NA"
             
        
async def mqtt_update(client):
    while True:
        await uasyncio.sleep(10)

        for i in temp_sensors:
            if i['value'] != "NA":
                await client.publish(f"fermcontrol/{i['name']}/temperature/state",str(round(i['value'],1)),qos=1)
        await client.publish("fermcontrol/cooling/state",str(cooling.value()),qos=1)
        await client.publish("fermcontrol/heating/state",str(heating.value()),qos=1)
        await client.publish("fermcontrol/chambersetpoint/state",setpoint['chamberSetpoint'],qos=1)
        await client.publish("fermcontrol/vesselsetpoint/state",setpoint['vesselSetpoint'],qos=1)
        


app = Microdot()

@app.route('/')
async def home(request):
    return send_file('/static/index.html')


@app.route('/api/values', methods=['GET'])
async def values(request):
    payload = {item['name']: item['value'] for item in temp_sensors}
    payload['coolingState'] = bool(cooling.value())
    payload['heatingState'] = bool(heating.value())

    return Response(headers={'Content-Type' : 'application/json','Access-Control-Allow-Origin':'*'}, body=ujson.dumps(payload))



@app.route('/api/setpoint', methods=['GET'])
async def getSetpoint(request):
    if 'action' in request.args:
        if request.args['action'] == 'update':
            setpoint['chamberSetpoint'] = request.args['chamber']
            setpoint['vesselSetpoint'] = request.args['vessel']
            with open('setpoint.json', "w") as f:
                ujson.dump(setpoint, f)

    payload = setpoint
    return Response(headers={'Content-Type' : 'application/json','Access-Control-Allow-Origin':'*'}, body=ujson.dumps(payload))


async def start_server():
    await app.start_server(port=80)



async def showResult():
    while True:
        await uasyncio.sleep(1)
        result = {'sensorReadings':temp_sensors, "cooling":cooling.value(), "heating":heating.value(), "setpoint":setpoint['chamberSetpoint']}
        print(result)
async def updateOutput():
    while True:
        await uasyncio.sleep_ms(1000)

        coolStart = int(setpoint['chamberSetpoint']) + int(config['cooling']['hysteresis']) / 2 
        coolStop = int(setpoint['chamberSetpoint']) - int(config['cooling']['hysteresis']) / 2 + 0.3
        heatStart = int(setpoint['chamberSetpoint']) - int(config['heating']['hysteresis']) / 2 
        heatStop = int(setpoint['chamberSetpoint']) + int(config['heating']['hysteresis']) / 2 - 0.3
        
        chamber_value = next((sensor['value'] for sensor in temp_sensors if sensor['name'] == 'chamber'))
        
        if chamber_value >= coolStart:
            if not cooling.value():
                cooling.on()
        elif chamber_value <= coolStop:
            if cooling.value():
               cooling.off()

        if chamber_value <= heatStart:
            if not heating.value():
                heating.on()
        elif chamber_value >= heatStop:
            if heating.value():
               heating.off()

        wdt.feed()

loop = uasyncio.get_event_loop()    
loop.create_task(temp_sensor_read())
loop.create_task(start_server())
loop.create_task(mqtt_connect(mqttclient))
loop.create_task(mqtt_update(mqttclient))
#loop.create_task(showResult())
loop.create_task(updateOutput())
loop.run_forever()



