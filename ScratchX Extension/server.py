from flask import Flask, request, Response
from xml.etree.ElementTree import *
app = Flask(__name__)

#main = Element('btn_status')
#main.text = "OFF"

global_led_status = ['0']*4
global_btn_status = [[]]*4
global_btn_status[0] = [1,1,1]

@app.route('/')
def hello_world():
    return 'Hello, World!'

@app.route('/led',methods=['GET','POST'])
def led_status_change():
    global global_led_status
    status = int(request.args.get('st'))
    led_number = int(request.args.get('ln'))
    if status == 1:
        global_led_status[led_number] = '1'
    elif status == 0:
        global_led_status[led_number] = '0'
    else:
        global_led_status[led_number] = 'None'
    return "LED {1} is now:{0}".format(status,led_number)

@app.route('/led_status',methods=['GET'])
def get_led_status():
    global global_led_status
    r_string = ''
    for status in global_led_status:
        r_string += str(status)
    return r_string

@app.route('/btn',methods=['GET'])
def btn_status_change():
    global global_btn_status
    btn_number = int(request.args.get('bt'))
    global_btn_status[btn_number].append(1)
    return "LED {1} is now:{0}".format(status, led_number)

@app.route('/btn_status',methods=['GET'])
def btn_status():
    global global_btn_status
    btn_number = int(request.args.get('bt'))
    if(len(global_btn_status[btn_number]) > 0):
        global_btn_status[btn_number].pop()
        return 1
    else:
        return 0

if __name__ == '__main__':
    app.run(host="0.0.0.0", threaded=True)
